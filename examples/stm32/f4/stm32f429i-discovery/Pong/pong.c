/*
 * Pong Game with Dual HC-SR05 Ultrasonic Gesture Control
 * STM32F429I-DISC1
 * * Sensor 1: PB6 (Trigger), PB7 (Echo) - Player 1
 * Sensor 2: PE2 (Trigger), PE3 (Echo) - Player 2
 * * Modificaciones:
 * - Implementado filtro Low-Pass (EMA) para eliminar el "jitter" de las paletas.
 * - Añadida funcionalidad de REINICIO de juego al pulsar el botón USER (B1/PA0).
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

/* ============================================================================
 * SENSOR CONFIGURATION
 * ============================================================================ */

/* Sensor 1 - Player 1 (Left) */
#define TRIG1_PORT GPIOB
#define TRIG1_PIN  GPIO6
#define ECHO1_PORT GPIOB
#define ECHO1_PIN  GPIO7

/* Sensor 2 - Player 2 (Right) */
#define TRIG2_PORT GPIOE
#define TRIG2_PIN  GPIO2
#define ECHO2_PORT GPIOE
#define ECHO2_PIN  GPIO3

/* Sensor timing constants */
#define TIMEOUT_ECHO_START_US  15000UL
#define TIMEOUT_ECHO_END_US    35000UL
#define MIN_VALID_PULSE_US     116UL
#define MAX_VALID_PULSE_US     26200UL
#define TRIGGER_PULSE_US       10
#define SENSOR_SETTLING_US     2

/* Distance mapping for paddles */
#define MIN_SENSOR_DISTANCE_MM  50      // 5 cm minimum
#define MAX_SENSOR_DISTANCE_MM  400     // 40 cm maximum
#define SENSOR_DEADZONE_MM      100      // Ignore small changes

/* Timer warmup */
#define TIMER_WARMUP_MS         52000//60000   // 60 segundos

/* ============================================================================
 * BUTTON CONFIGURATION (User Button - B1/PA0)
 * ============================================================================ */

#define USER_BUTTON_PORT GPIOA
#define USER_BUTTON_PIN  GPIO0

/* ============================================================================
 * GAME CONFIGURATION
 * ============================================================================ */

#define LCD_WIDTH       240
#define LCD_HEIGHT      320

/* Game objects */
#define PADDLE_WIDTH    6
#define PADDLE_HEIGHT   50
#define BALL_SIZE       8
#define PADDLE_OFFSET   0      // Distance from edge

/* Game physics */
#define BALL_SPEED_X    6//3
#define BALL_SPEED_Y    4//2
#define MAX_BALL_SPEED  12//6
#define WINNING_SCORE   5

/* Update rates */
#define GAME_UPDATE_MS      16      // ~60 FPS
#define SENSOR_UPDATE_MS    30      // ~33 Hz sensor reading

/* FILTRO DE SUAVIZADO */
#define FILTER_ALPHA    0.5f    // Factor de filtro

/* ============================================================================
 * GAME STRUCTURES
 * ============================================================================ */

typedef struct {
    int16_t x, y;
    int16_t vx, vy;
} Ball;

typedef struct {
    int16_t y;          // Posición visual (entero)
    float y_filtered;   // Posición interna suavizada (decimal)
    uint16_t distance_mm;
    uint8_t score;
} Paddle;

typedef enum {
    SENSOR_OK = 0,
    SENSOR_TIMEOUT,
    SENSOR_INVALID
} sensor_error_t;

typedef struct {
    uint16_t distance_mm;
    sensor_error_t error;
} sensor_reading_t;

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

static Ball ball;
static Paddle player1;  // Left paddle (Sensor 1)
static Paddle player2;  // Right paddle (Sensor 2)
static volatile uint8_t game_running = 0;
static volatile uint8_t timer_ready = 0;

/* ============================================================================
 * TIMER FUNCTIONS
 * ============================================================================ */

static inline uint32_t timer_diff(uint32_t start, uint32_t end)
{
    return (end >= start) ? (end - start) : ((0xFFFFFFFF - start) + end + 1);
}

static inline void delay_us(uint32_t us)
{
    if (us == 0) return;
    uint32_t start = timer_get_counter(TIM5);
    while (timer_diff(start, timer_get_counter(TIM5)) < us);
}

static inline void delay_ms(uint32_t ms)
{
    msleep(ms);  // Use clock.c systick-based delay
}

/* ============================================================================
 * HARDWARE INITIALIZATION
 * ============================================================================ */

static void tim5_setup(void)
{
    rcc_periph_reset_pulse(RST_TIM5);
    timer_disable_counter(TIM5);
    
    timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM5, 83);  // 84 MHz / 84 = 1 MHz (1 µs)
    
    timer_disable_preload(TIM5);
    timer_continuous_mode(TIM5);
    timer_set_period(TIM5, 0xFFFFFFFF);
    timer_set_counter(TIM5, 0);
    
    timer_enable_counter(TIM5);
}

static void gpio_sensor_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_TIM5);
    
    /* Sensor 1 */
    gpio_mode_setup(TRIG1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG1_PIN);
    gpio_set_output_options(TRIG1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG1_PIN);
    gpio_clear(TRIG1_PORT, TRIG1_PIN);
    gpio_mode_setup(ECHO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO1_PIN);
    
    /* Sensor 2 */
    gpio_mode_setup(TRIG2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG2_PIN);
    gpio_set_output_options(TRIG2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG2_PIN);
    gpio_clear(TRIG2_PORT, TRIG2_PIN);
    gpio_mode_setup(ECHO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO2_PIN);
}

// NUEVA FUNCIÓN: Configuración del botón de usuario
static void gpio_button_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    /* User Button (B1) on PA0. Input with Pull-down */
    gpio_mode_setup(USER_BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, USER_BUTTON_PIN);
}

/* ============================================================================
 * SENSOR READING
 * ============================================================================ */

static uint32_t read_pulse_raw(uint32_t trig_port, uint16_t trig_pin,
                                 uint32_t echo_port, uint16_t echo_pin)
{
    uint32_t start_time, end_time, timeout_start;
    
    /* Send trigger pulse */
    gpio_clear(trig_port, trig_pin);
    delay_us(SENSOR_SETTLING_US);
    gpio_set(trig_port, trig_pin);
    delay_us(TRIGGER_PULSE_US);
    gpio_clear(trig_port, trig_pin);
    
    /* Wait for echo rising edge */
    timeout_start = timer_get_counter(TIM5);
    while (gpio_get(echo_port, echo_pin) == 0) {
        if (timer_diff(timeout_start, timer_get_counter(TIM5)) > TIMEOUT_ECHO_START_US) {
            return 0;
        }
    }
    
    start_time = timer_get_counter(TIM5);
    
    /* Wait for echo falling edge */
    while (gpio_get(echo_port, echo_pin) != 0) {
        if (timer_diff(start_time, timer_get_counter(TIM5)) > TIMEOUT_ECHO_END_US) {
            return 0;
        }
    }
    
    end_time = timer_get_counter(TIM5);
    return timer_diff(start_time, end_time);
}

static void read_sensor(uint32_t trig_port, uint16_t trig_pin,
                        uint32_t echo_port, uint16_t echo_pin,
                        sensor_reading_t *reading)
{
    reading->error = SENSOR_OK;
    reading->distance_mm = 0;
    
    uint32_t pulse = read_pulse_raw(trig_port, trig_pin, echo_port, echo_pin);
    
    if (pulse == 0) {
        reading->error = SENSOR_TIMEOUT;
        return;
    }
    
    if (pulse < MIN_VALID_PULSE_US || pulse > MAX_VALID_PULSE_US) {
        reading->error = SENSOR_INVALID;
        return;
    }
    
    /* Convert to millimeters: distance_mm = (pulse_us * 343) / 2000 */
    reading->distance_mm = (uint16_t)((pulse * 343UL) / 2000UL);
}

/* ============================================================================
 * TIMER WARMUP
 * ============================================================================ */

static void timer_warmup(void)
{
    char buf[64];
    
    console_puts("\n========================================\n");
    console_puts("  TIMER WARMUP - 60 SECONDS\n");
    console_puts("========================================\n");
    console_puts("El timer necesita estabilizarse para\n");
    console_puts("generar pulsos precisos de 10 us.\n\n");
    console_puts("Preparando sensores...\n\n");
    console_puts("Progreso: 0%\n");
    
    uint32_t start = mtime();
    uint32_t last_update = 0;
    
    while ((mtime() - start) < TIMER_WARMUP_MS) {
        uint32_t elapsed = mtime() - start;
        
        if (elapsed - last_update >= 1000) {
            last_update = elapsed;
            uint8_t progress = (elapsed * 100) / TIMER_WARMUP_MS;
            
            console_puts("\033[1A\033[2K\r");
            snprintf(buf, sizeof(buf), "Progreso: %u%% (%lu/%lu seg)\n",
                     progress, elapsed/1000, TIMER_WARMUP_MS/1000);
            console_puts(buf);
        }
    }
    
    timer_ready = 1;
    console_puts("\n*** SENSORES LISTOS ***\n\n");
}

/* ============================================================================
 * GAME LOGIC
 * ============================================================================ */

static void game_init(void)
{
    /* Initialize ball */
    ball.x = LCD_WIDTH / 2 - BALL_SIZE / 2;
    ball.y = LCD_HEIGHT / 2 - BALL_SIZE / 2;
    // La dirección inicial puede ser aleatoria o fija
    ball.vx = (rand() % 2 == 0) ? BALL_SPEED_X : -BALL_SPEED_X;
    ball.vy = BALL_SPEED_Y;
    
    /* Initialize paddles */
    player1.y = LCD_HEIGHT / 2 - PADDLE_HEIGHT / 2;
    player1.y_filtered = (float)player1.y;
    player1.score = 0;
    player1.distance_mm = 200;
    
    player2.y = LCD_HEIGHT / 2 - PADDLE_HEIGHT / 2;
    player2.y_filtered = (float)player2.y;
    player2.score = 0;
    player2.distance_mm = 200;
    
    game_running = 1;
}

static void update_paddle_from_sensor(Paddle *paddle, sensor_reading_t *reading)
{
    if (reading->error != SENSOR_OK) {
        return;
    }
    
    paddle->distance_mm = reading->distance_mm;
    
    /* 1. Clampeo de lectura cruda */
    uint16_t dist = reading->distance_mm;
    if (dist < MIN_SENSOR_DISTANCE_MM) dist = MIN_SENSOR_DISTANCE_MM;
    if (dist > MAX_SENSOR_DISTANCE_MM) dist = MAX_SENSOR_DISTANCE_MM;
    
    /* 2. Calcular posición Objetivo (Target Y) */
    uint16_t range_sensor = MAX_SENSOR_DISTANCE_MM - MIN_SENSOR_DISTANCE_MM;
    uint16_t range_screen = LCD_HEIGHT - PADDLE_HEIGHT;
    uint16_t offset = dist - MIN_SENSOR_DISTANCE_MM;
    
    float target_y = (float)((offset * range_screen) / range_sensor);
    
    /* 3. APLICAR FILTRO DE SUAVIZADO (Low Pass Filter) */
    paddle->y_filtered = (FILTER_ALPHA * target_y) + ((1.0f - FILTER_ALPHA) * paddle->y_filtered);
    
    /* 4. Convertir a coordenadas de pantalla */
    int16_t final_y = (int16_t)paddle->y_filtered;
    
    /* Clamp to screen bounds */
    if (final_y < 0) final_y = 0;
    if (final_y > range_screen) final_y = range_screen;
    
    /* 5. Aplicar pequeña Deadzone visual (1 pixel) para evitar parpadeo */
    if (abs(paddle->y - final_y) > 1) {
        paddle->y = final_y;
    }
}

static void game_update(void)
{
    /* Update ball position */
    ball.x += ball.vx;
    ball.y += ball.vy;
    
    /* Ball collision with top/bottom */
    if (ball.y <= 0) {
        ball.y = 0;
        ball.vy = -ball.vy;
    }
    if (ball.y >= LCD_HEIGHT - BALL_SIZE) {
        ball.y = LCD_HEIGHT - BALL_SIZE;
        ball.vy = -ball.vy;
    }
    
    /* Ball collision with player 1 paddle */
    if (ball.vx < 0 && ball.x <= PADDLE_OFFSET + PADDLE_WIDTH) {
        if (ball.y + BALL_SIZE >= player1.y && 
            ball.y <= player1.y + PADDLE_HEIGHT) {
            ball.x = PADDLE_OFFSET + PADDLE_WIDTH;
            ball.vx = -ball.vx;
            
            /* Add spin based on hit position */
            int16_t hit_offset = (ball.y + BALL_SIZE/2) - (player1.y + PADDLE_HEIGHT/2);
            ball.vy += hit_offset / 8;
            if (ball.vy > MAX_BALL_SPEED) ball.vy = MAX_BALL_SPEED;
            if (ball.vy < -MAX_BALL_SPEED) ball.vy = -MAX_BALL_SPEED;
        }
    }
    
    /* Ball collision with player 2 paddle */
    if (ball.vx > 0 && ball.x + BALL_SIZE >= LCD_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH) {
        if (ball.y + BALL_SIZE >= player2.y && 
            ball.y <= player2.y + PADDLE_HEIGHT) {
            ball.x = LCD_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH - BALL_SIZE;
            ball.vx = -ball.vx;
            
            int16_t hit_offset = (ball.y + BALL_SIZE/2) - (player2.y + PADDLE_HEIGHT/2);
            ball.vy += hit_offset / 8;
            if (ball.vy > MAX_BALL_SPEED) ball.vy = MAX_BALL_SPEED;
            if (ball.vy < -MAX_BALL_SPEED) ball.vy = -MAX_BALL_SPEED;
        }
    }
    
    /* Score points */
    if (ball.x < 0) {
        player2.score++;
        ball.x = LCD_WIDTH / 2 - BALL_SIZE / 2;
        ball.y = LCD_HEIGHT / 2 - BALL_SIZE / 2;
        ball.vx = BALL_SPEED_X;
        ball.vy = BALL_SPEED_Y;
    }
    
    if (ball.x > LCD_WIDTH) {
        player1.score++;
        ball.x = LCD_WIDTH / 2 - BALL_SIZE / 2;
        ball.y = LCD_HEIGHT / 2 - BALL_SIZE / 2;
        ball.vx = -BALL_SPEED_X;
        ball.vy = BALL_SPEED_Y;
    }
    
    /* Check win condition */
    if (player1.score >= WINNING_SCORE || player2.score >= WINNING_SCORE) {
        game_running = 0;
    }
}

static void game_render(void)
{
    char buf[32];
    
    /* Clear screen */
    gfx_fillScreen(LCD_BLACK);
    
    /* Draw center line */
    for (int16_t i = 0; i < LCD_HEIGHT; i += 15) {
        gfx_fillRect(LCD_WIDTH / 2 - 2, i, 4, 10, LCD_GREY);
    }
    
    /* Draw paddles */
    gfx_fillRect(PADDLE_OFFSET, player1.y, PADDLE_WIDTH, PADDLE_HEIGHT, LCD_GREEN);
    gfx_fillRect(LCD_WIDTH - PADDLE_OFFSET - PADDLE_WIDTH, player2.y, 
                 PADDLE_WIDTH, PADDLE_HEIGHT, LCD_RED);
    
    /* Draw ball */
    gfx_fillRect(ball.x, ball.y, BALL_SIZE, BALL_SIZE, LCD_WHITE);
    
    /* Draw scores */
    gfx_setTextSize(3);
    gfx_setTextColor(LCD_GREEN, LCD_BLACK);
    gfx_setCursor(LCD_WIDTH / 2 - 50, 20);
    snprintf(buf, sizeof(buf), "%d", player1.score);
    gfx_puts(buf);
    
    gfx_setTextColor(LCD_RED, LCD_BLACK);
    gfx_setCursor(LCD_WIDTH / 2 + 30, 20);
    snprintf(buf, sizeof(buf), "%d", player2.score);
    gfx_puts(buf);
    
    /* Draw game over message */
    if (!game_running) {
        gfx_fillRoundRect(9, LCD_HEIGHT/2 - 30, 220, 60, 10, LCD_BLUE);
        gfx_setTextSize(2);
        gfx_setTextColor(LCD_YELLOW, LCD_BLUE);
        gfx_setCursor(10, LCD_HEIGHT/2 - 10);
        if (player1.score > player2.score) {
            gfx_puts("PLAYER 1 WINS");
        } else {
            gfx_puts("PLAYER 2 WINS");
        }
        
        gfx_setTextSize(1);
        gfx_setTextColor(LCD_WHITE, LCD_BLACK);
        gfx_setCursor(LCD_WIDTH/4-10, LCD_HEIGHT - 10);
        gfx_puts("BOTON USER reinicia");
    }
    
    /* Show frame */
    lcd_show_frame();
}

/* ============================================================================
 * MAIN
 * ============================================================================ */

int main(void)
{
    sensor_reading_t reading1, reading2;
    uint32_t last_sensor_update = 0;
    uint32_t last_game_update = 0;
    
    /* System initialization */
    clock_setup();
    console_setup(115200);
    
    console_puts("\n========================================\n");
    console_puts("  PONG - Dual Sensor Gesture Control\n");
    console_puts("========================================\n");
    console_puts("Player 1: Sensor 1 (PB6/PB7)\n");
    console_puts("Player 2: Sensor 2 (PE2/PE3)\n");
    console_puts("Primer jugador a 5 puntos gana!\n\n");
    
    /* Initialize hardware */
    gpio_sensor_setup();
    tim5_setup();
    gpio_button_setup(); // Configuración del botón USER (PA0)
    
    /* Timer warmup */
    timer_warmup();
    
    console_puts("Inicializando SDRAM...\n");
    sdram_init();
    
    console_puts("Inicializando LCD...\n");
    lcd_spi_init();
    
    console_puts("Inicializando graficos...\n");
    gfx_init(lcd_draw_pixel, LCD_WIDTH, LCD_HEIGHT);
    
    console_puts("\n*** JUEGO INICIADO ***\n\n");
    
    /* Initialize game */
    game_init();
    game_render();
    
    /* Main game loop */
    while (1) {
        uint32_t now = mtime();
        
        /* Update sensors at ~33 Hz */
        if (now - last_sensor_update >= SENSOR_UPDATE_MS) {
            last_sensor_update = now;
            
            /* Read both sensors */
            read_sensor(TRIG1_PORT, TRIG1_PIN, ECHO1_PORT, ECHO1_PIN, &reading1);
            delay_ms(20);  // Small delay between sensors
            read_sensor(TRIG2_PORT, TRIG2_PIN, ECHO2_PORT, ECHO2_PIN, &reading2);
            
            /* Update paddle positions */
            update_paddle_from_sensor(&player1, &reading1);
            update_paddle_from_sensor(&player2, &reading2);
        }
        
        /* Update game logic at ~60 Hz */
        if (game_running && now - last_game_update >= GAME_UPDATE_MS) {
            last_game_update = now;
            game_update();
            game_render();
        } else if (!game_running) {
            /* Game over - wait for restart or show final screen */
            game_render();
            
            /* Lógica de REINICIO al pulsar el botón USER (B1/PA0) */
            if (gpio_get(USER_BUTTON_PORT, USER_BUTTON_PIN) != 0) {
                // Debounce simple
                delay_ms(50);
                if (gpio_get(USER_BUTTON_PORT, USER_BUTTON_PIN) != 0) {
                    console_puts("*** REINICIANDO JUEGO ***\n\n");
                    game_init(); // Reinicia el juego
                    game_render();
                }
            }
            delay_ms(100);
        }
    }
    
    return 0;
}