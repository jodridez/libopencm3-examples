/*
 * Dual HY-SRF05 Ultrasonic Sensors - High-Speed Version
 * STM32F429I-DISC1
 * 
 * Sensor 1: PB6 (Trigger), PB7 (Echo)
 * Sensor 2: PE2 (Trigger), PE3 (Echo)
 * 
 * OPTIMIZADO PARA:
 * - Control gestual de alta velocidad (juego Pong)
 * - Aritmética entera (sin floats)
 * - Muestreo rápido (~50Hz por sensor, 25Hz sistema completo)
 * - Tiempo de estabilización inicial del timer
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include "console.h"

/* ============================================================================
 * CONFIGURACIÓN DE HARDWARE
 * ============================================================================ */

/* Sensor 1 */
#define TRIG1_PORT GPIOB
#define TRIG1_PIN  GPIO6
#define ECHO1_PORT GPIOB
#define ECHO1_PIN  GPIO7

/* Sensor 2 */
#define TRIG2_PORT GPIOE
#define TRIG2_PIN  GPIO2
#define ECHO2_PORT GPIOE
#define ECHO2_PIN  GPIO3

/* ============================================================================
 * CONSTANTES DE TEMPORIZACIÓN (ARITMÉTICA ENTERA)
 * ============================================================================ */

/* Timeouts para detección de ECHO (en microsegundos) */
#define TIMEOUT_ECHO_START_US  15000UL  // 15ms para inicio de echo
#define TIMEOUT_ECHO_END_US    35000UL  // 35ms máximo para echo (~6m)

/* Límites físicos del sensor HY-SRF05 */
#define MIN_DISTANCE_MM        20       // 2 cm = 20 mm
#define MAX_DISTANCE_MM        4500     // 450 cm = 4500 mm
#define MIN_VALID_PULSE_US     116UL    // ~2cm
#define MAX_VALID_PULSE_US     26200UL  // ~450cm

/* Timing del sensor */
#define TRIGGER_PULSE_US       10       // Pulso trigger de 10 µs
#define SENSOR_SETTLING_US     2        // Tiempo de estabilización
#define INTER_SENSOR_DELAY_MS  20       // Delay mínimo entre sensores (antes: 60ms)
#define INTER_CYCLE_DELAY_MS   20       // Delay mínimo entre ciclos (antes: 60ms)

/* 
 * Conversión velocidad del sonido (aritmética entera):
 * Velocidad = 343 m/s = 0.0343 cm/µs
 * Distancia (mm) = (tiempo_us * 343) / 2000
 * Simplificado: distancia_mm = (tiempo_us * 343) / 2000
 * 
 * Para evitar overflow en multiplicación:
 * - tiempo_us máximo: 26200
 * - 26200 * 343 = 8,986,600 (cabe en uint32_t)
 */
#define SOUND_SPEED_NUMERATOR   343     // m/s
#define SOUND_SPEED_DENOMINATOR 2000    // Factor de conversión a mm

/* Delay entre actualizaciones de pantalla (más rápido) */
#define DISPLAY_UPDATE_MS      50       // 20 Hz de actualización

/* Tiempo de estabilización inicial del timer */
#define TIMER_WARMUP_MS        60000    // 60 segundos para estabilización completa

/* ============================================================================
 * TIPOS DE DATOS
 * ============================================================================ */

typedef enum {
    SENSOR_OK = 0,
    SENSOR_TIMEOUT_START,
    SENSOR_TIMEOUT_END,
    SENSOR_OUT_OF_RANGE,
    SENSOR_INVALID_PULSE
} sensor_error_t;

typedef struct {
    uint16_t distance_mm;     // Distancia en milímetros (entero)
    sensor_error_t error;
    uint32_t pulse_us;
} sensor_reading_t;

/* ============================================================================
 * VARIABLES GLOBALES
 * ============================================================================ */

static volatile uint8_t timer_ready = 0;  // Flag de timer estabilizado

/* ============================================================================
 * CONFIGURACIÓN DEL RELOJ Y TIMERS
 * ============================================================================ */

/**
 * @brief Configura el reloj del sistema a 168 MHz
 */
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_TIM5);
}

/**
 * @brief Configura TIM5 como temporizador de 32-bit a 1 MHz
 * 
 * TIM5CLK = 84 MHz (2 × APB1), Prescaler = 83 → 1 MHz (1 µs)
 * 
 * NOTA: El timer necesita ~60 segundos para estabilizarse completamente
 * y producir pulsos de trigger precisos de 10 µs.
 */
static void tim5_setup(void)
{
    rcc_periph_reset_pulse(RST_TIM5);
    timer_disable_counter(TIM5);
    
    timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM5, 83);  // 84 MHz / 84 = 1 MHz
    
    timer_disable_preload(TIM5);
    timer_continuous_mode(TIM5);
    timer_set_period(TIM5, 0xFFFFFFFF);
    timer_set_counter(TIM5, 0);
    
    timer_enable_counter(TIM5);
}

/* ============================================================================
 * CONFIGURACIÓN DE GPIO
 * ============================================================================ */

static void gpio_setup(void)
{
    /* SENSOR 1 */
    gpio_mode_setup(TRIG1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG1_PIN);
    gpio_set_output_options(TRIG1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG1_PIN);
    gpio_clear(TRIG1_PORT, TRIG1_PIN);
    gpio_mode_setup(ECHO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO1_PIN);
    
    /* SENSOR 2 */
    gpio_mode_setup(TRIG2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG2_PIN);
    gpio_set_output_options(TRIG2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG2_PIN);
    gpio_clear(TRIG2_PORT, TRIG2_PIN);
    gpio_mode_setup(ECHO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO2_PIN);
}

/* ============================================================================
 * FUNCIONES DE TEMPORIZACIÓN (OPTIMIZADAS)
 * ============================================================================ */

/**
 * @brief Calcula diferencia de tiempo con manejo de overflow
 */
static inline uint32_t timer_diff(uint32_t start, uint32_t end)
{
    return (end >= start) ? (end - start) : ((0xFFFFFFFF - start) + end + 1);
}

/**
 * @brief Delay en microsegundos (versión ultra-optimizada)
 */
static inline void delay_us(uint32_t us)
{
    if (us == 0) return;
    
    uint32_t start = timer_get_counter(TIM5);
    
    /* Espera activa optimizada */
    while (timer_diff(start, timer_get_counter(TIM5)) < us);
}

/**
 * @brief Delay en milisegundos (optimizado)
 */
static inline void delay_ms(uint32_t ms)
{
    uint32_t start = timer_get_counter(TIM5);
    uint32_t target_us = ms * 1000UL;
    
    while (timer_diff(start, timer_get_counter(TIM5)) < target_us);
}

/* ============================================================================
 * FUNCIONES DE MEDICIÓN (ARITMÉTICA ENTERA)
 * ============================================================================ */

/**
 * @brief Lee el pulso ECHO del sensor (versión optimizada)
 * 
 * @return 0 si hay error, duración del pulso en µs si es exitoso
 */
static uint32_t hcsr05_read_pulse_raw(uint32_t trig_port, uint16_t trig_pin, 
                                       uint32_t echo_port, uint16_t echo_pin)
{
    uint32_t start_time, end_time, timeout_start;
    
    /* Enviar pulso de trigger (10 µs) */
    gpio_clear(trig_port, trig_pin);
    delay_us(SENSOR_SETTLING_US);
    gpio_set(trig_port, trig_pin);
    delay_us(TRIGGER_PULSE_US);
    gpio_clear(trig_port, trig_pin);
    
    /* Esperar flanco de subida con timeout */
    timeout_start = timer_get_counter(TIM5);
    while (gpio_get(echo_port, echo_pin) == 0) {
        if (timer_diff(timeout_start, timer_get_counter(TIM5)) > TIMEOUT_ECHO_START_US) {
            return 0;  // Timeout
        }
    }
    
    start_time = timer_get_counter(TIM5);
    
    /* Esperar flanco de bajada con timeout */
    while (gpio_get(echo_port, echo_pin) != 0) {
        if (timer_diff(start_time, timer_get_counter(TIM5)) > TIMEOUT_ECHO_END_US) {
            return 0;  // Timeout
        }
    }
    
    end_time = timer_get_counter(TIM5);
    
    return timer_diff(start_time, end_time);
}

/**
 * @brief Convierte tiempo de pulso a distancia en milímetros (aritmética entera)
 * 
 * Fórmula: distancia_mm = (pulse_us * 343) / 2000
 */
static inline uint16_t pulse_to_distance_mm(uint32_t pulse_us)
{
    /* Evitar overflow: verificar límites antes de multiplicar */
    if (pulse_us > MAX_VALID_PULSE_US) {
        return 0;
    }
    
    /* Cálculo entero: (pulse_us * 343) / 2000 */
    uint32_t distance = (pulse_us * SOUND_SPEED_NUMERATOR) / SOUND_SPEED_DENOMINATOR;
    
    return (uint16_t)distance;
}

/**
 * @brief Lee el sensor y retorna la medición completa
 */
static void hcsr05_read(uint32_t trig_port, uint16_t trig_pin,
                        uint32_t echo_port, uint16_t echo_pin,
                        sensor_reading_t *reading)
{
    reading->error = SENSOR_OK;
    reading->pulse_us = 0;
    reading->distance_mm = 0;
    
    /* Leer pulso crudo */
    uint32_t pulse = hcsr05_read_pulse_raw(trig_port, trig_pin, echo_port, echo_pin);
    
    if (pulse == 0) {
        reading->error = SENSOR_TIMEOUT_START;
        return;
    }
    
    /* Validar rango del pulso */
    if (pulse < MIN_VALID_PULSE_US || pulse > MAX_VALID_PULSE_US) {
        reading->error = SENSOR_INVALID_PULSE;
        reading->pulse_us = pulse;
        return;
    }
    
    /* Convertir a distancia */
    reading->pulse_us = pulse;
    reading->distance_mm = pulse_to_distance_mm(pulse);
    
    /* Verificar rango físico */
    if (reading->distance_mm < MIN_DISTANCE_MM || 
        reading->distance_mm > MAX_DISTANCE_MM) {
        reading->error = SENSOR_OUT_OF_RANGE;
    }
}

/* ============================================================================
 * FUNCIONES DE UTILIDAD
 * ============================================================================ */

static const char* get_error_string(sensor_error_t error)
{
    switch (error) {
        case SENSOR_OK:              return "OK";
        case SENSOR_TIMEOUT_START:   return "NO ECHO";
        case SENSOR_TIMEOUT_END:     return "TIMEOUT";
        case SENSOR_OUT_OF_RANGE:    return "OUT RANGE";
        case SENSOR_INVALID_PULSE:   return "INVALID";
        default:                     return "ERROR";
    }
}

/**
 * @brief Calentamiento del timer para estabilización
 */
static void timer_warmup(void)
{
    char buf[64];
    uint32_t elapsed_ms = 0;
    uint32_t last_update = 0;
    
    console_puts("\n*** ESTABILIZANDO TIMER ***\n");
    console_puts("El timer necesita ~60 segundos para\n");
    console_puts("generar pulsos precisos de 10 us.\n\n");
    console_puts("Progreso: 0%\n");
    
    uint32_t start_time = timer_get_counter(TIM5);
    
    while (elapsed_ms < TIMER_WARMUP_MS) {
        elapsed_ms = timer_diff(start_time, timer_get_counter(TIM5)) / 1000;
        
        /* Actualizar cada segundo */
        if (elapsed_ms - last_update >= 1000) {
            last_update = elapsed_ms;
            uint8_t progress = (elapsed_ms * 100) / TIMER_WARMUP_MS;
            
            console_puts("\033[1A\033[2K\r");  // Subir y limpiar línea
            snprintf(buf, sizeof(buf), "Progreso: %u%% (%lu/%u seg)\n", 
                    progress, elapsed_ms/1000, TIMER_WARMUP_MS/1000);
            console_puts(buf);
        }
        
        delay_ms(100);
    }
    
    timer_ready = 1;
    console_puts("\n*** TIMER LISTO ***\n\n");
    delay_ms(500);
}

/* ============================================================================
 * PROGRAMA PRINCIPAL
 * ============================================================================ */

int main(void)
{
    char buf[128];
    uint32_t measurement_count = 0;
    sensor_reading_t reading1, reading2;
    
    /* Configuración del sistema */
    clock_setup();
    gpio_setup();
    tim5_setup();
    
    /* Inicializar consola */
    console_setup(115200);
    
    console_puts("\n==========================================\n");
    console_puts("  HY-SRF05 High-Speed Gesture Control\n");
    console_puts("==========================================\n");
    console_puts("Sensor 1: PB6 (Trig), PB7 (Echo)\n");
    console_puts("Sensor 2: PE2 (Trig), PE3 (Echo)\n");
    console_puts("Rango: 2-450 cm\n");
    console_puts("Modo: Juego Pong (50 Hz)\n");
    console_puts("Aritmetica: Entera (sin floats)\n\n");
    
    /* Pequeño delay inicial */
    delay_ms(100);
    
    /* Fase de calentamiento del timer */
    timer_warmup();
    
    console_puts("Iniciando mediciones rapidas...\n\n");
    
    /* Preparar pantalla para actualización rápida */
    console_puts("\n\n\n\n\n\n");
    
    uint32_t last_display_update = timer_get_counter(TIM5);
    
    while (1) {
        measurement_count++;
        
        /* Leer ambos sensores rápidamente */
        hcsr05_read(TRIG1_PORT, TRIG1_PIN, ECHO1_PORT, ECHO1_PIN, &reading1);
        delay_ms(INTER_SENSOR_DELAY_MS);
        
        hcsr05_read(TRIG2_PORT, TRIG2_PIN, ECHO2_PORT, ECHO2_PIN, &reading2);
        delay_ms(INTER_CYCLE_DELAY_MS);
        
        /* Actualizar pantalla solo cada DISPLAY_UPDATE_MS (reducir latencia) */
        uint32_t now = timer_get_counter(TIM5);
        if (timer_diff(last_display_update, now) >= (DISPLAY_UPDATE_MS * 1000)) {
            last_display_update = now;
            
            /* Mover cursor arriba */
            console_puts("\033[6A");
            
            /* Línea 1: Info rápida */
            console_puts("\033[2K\r");
            snprintf(buf, sizeof(buf), "[#%lu] Freq: ~%u Hz", 
                    measurement_count, 
                    1000 / (INTER_SENSOR_DELAY_MS * 2 + INTER_CYCLE_DELAY_MS));
            console_puts(buf);
            
            /* Línea 2: Separador */
            console_puts("\n\033[2K\r");
            console_puts("--------------------------------");
            
            /* Línea 3: Sensor 1 */
            console_puts("\n\033[2K\r");
            if (reading1.error == SENSOR_OK) {
                snprintf(buf, sizeof(buf), "S1: %4u mm (%3u cm) [%5lu us]", 
                        reading1.distance_mm, 
                        reading1.distance_mm / 10,
                        reading1.pulse_us);
            } else {
                snprintf(buf, sizeof(buf), "S1: %s", get_error_string(reading1.error));
            }
            console_puts(buf);
            
            /* Línea 4: Sensor 2 */
            console_puts("\n\033[2K\r");
            if (reading2.error == SENSOR_OK) {
                snprintf(buf, sizeof(buf), "S2: %4u mm (%3u cm) [%5lu us]", 
                        reading2.distance_mm,
                        reading2.distance_mm / 10,
                        reading2.pulse_us);
            } else {
                snprintf(buf, sizeof(buf), "S2: %s", get_error_string(reading2.error));
            }
            console_puts(buf);
            
            /* Línea 5: Diferencia */
            console_puts("\n\033[2K\r");
            if (reading1.error == SENSOR_OK && reading2.error == SENSOR_OK) {
                int16_t diff = (int16_t)reading1.distance_mm - (int16_t)reading2.distance_mm;
                snprintf(buf, sizeof(buf), "Diff: %+5d mm", diff);
            } else {
                snprintf(buf, sizeof(buf), "                ");
            }
            console_puts(buf);
            
            /* Línea 6: Timer info */
            console_puts("\n\033[2K\r");
            snprintf(buf, sizeof(buf), "Timer: %lu us", timer_get_counter(TIM5));
            console_puts(buf);
        }
    }
    
    return 0;
}