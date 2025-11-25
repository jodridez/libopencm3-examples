/*
 * Dual HY-SRF05 Ultrasonic Sensors - Timer-Based Version
 * STM32F429I-DISC1
 * 
 * Sensor 1: PB6 (Trigger), PB7 (Echo)
 * Sensor 2: PE2 (Trigger), PE3 (Echo)
 * 
 * Timer Configuration:
 * - TIM5: 32-bit free-running timer @ 1 MHz (1 µs resolution)
 * - Clock: APB1 = 42 MHz, TIM5CLK = 84 MHz (x2 multiplier)
 * - Prescaler: 83 (84 MHz / 84 = 1 MHz)
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
 * CONSTANTES DE TEMPORIZACIÓN
 * ============================================================================ */

/* Timeouts para detección de ECHO */
#define TIMEOUT_ECHO_START_US  15000UL  // 15ms para inicio de echo
#define TIMEOUT_ECHO_END_US    35000UL  // 35ms máximo para echo (~6m)

/* Límites físicos del sensor HY-SRF05 */
#define MIN_DISTANCE_CM        2.0f
#define MAX_DISTANCE_CM        450.0f
#define MIN_VALID_PULSE_US     116UL    // ~2cm
#define MAX_VALID_PULSE_US     26200UL  // ~450cm

/* Timing del sensor */
#define TRIGGER_PULSE_US       10       // Pulso trigger
#define SENSOR_SETTLING_US     2        // Tiempo de estabilización
#define INTER_SENSOR_DELAY_MS  60       // Delay entre sensores
#define INTER_CYCLE_DELAY_MS   60       // Delay entre ciclos

/* Velocidad del sonido: 343 m/s a 20°C */
#define US_PER_CM              58.24f   // 1 / (0.0343 / 2)

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
    float distance_cm;
    sensor_error_t error;
    uint32_t pulse_us;
} sensor_reading_t;

/* ============================================================================
 * CONFIGURACIÓN DEL RELOJ Y TIMERS
 * ============================================================================ */

/**
 * @brief Configura el reloj del sistema a 168 MHz
 * 
 * Clock Tree:
 * - SYSCLK: 168 MHz
 * - AHB (HCLK): 168 MHz / 1 = 168 MHz
 * - APB1: 168 MHz / 4 = 42 MHz
 * - APB2: 168 MHz / 2 = 84 MHz
 * 
 * TIM5 está en APB1:
 * - TIMPRE = 0 (default)
 * - APB1 prescaler != 1, por lo tanto:
 * - TIM5CLK = 2 × PCLK1 = 2 × 42 MHz = 84 MHz
 */
static void clock_setup(void)
{
    /* Configurar PLL para 168 MHz desde HSE de 8 MHz */
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    /* 
     * La función rcc_clock_setup_pll configura:
     * - AHB Prescaler = 1 (NODIV)
     * - APB1 Prescaler = 4 (DIV4)
     * - APB2 Prescaler = 2 (DIV2)
     * - Actualiza: rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency
     */
    
    /* Habilitar relojes de periféricos */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_TIM5);
}

/**
 * @brief Configura TIM5 como temporizador de 32-bit a 1 MHz
 * 
 * Configuración:
 * - TIM5CLK = 84 MHz (2 × APB1)
 * - Prescaler = 83 (para obtener 1 MHz)
 * - FCNT = 84 MHz / (83 + 1) = 1 MHz
 * - Resolución = 1 µs
 * - Período = 2^32 - 1 (máximo para 32-bit)
 * - Modo: Continuo, sin preload, conteo ascendente
 */
static void tim5_setup(void)
{
    /* Reset del periférico a valores por defecto */
    rcc_periph_reset_pulse(RST_TIM5);
    
    /* Deshabilitar el contador durante la configuración */
    timer_disable_counter(TIM5);
    
    /* 
     * Configurar modo del timer:
     * - Edge-aligned (no center-aligned)
     * - Clock division = 1 (CK_INT)
     * - Direction = UP (conteo ascendente)
     */
    timer_set_mode(TIM5, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    
    /*
     * Configurar prescaler para 1 MHz:
     * TIM5CLK = 84 MHz (verificado: 2 × 42 MHz de APB1)
     * Prescaler = (TIM5CLK / Frecuencia_deseada) - 1
     * Prescaler = (84,000,000 / 1,000,000) - 1 = 83
     */
    timer_set_prescaler(TIM5, 83);
    
    /* Deshabilitar preload del ARR (actualización inmediata) */
    timer_disable_preload(TIM5);
    
    /* Modo continuo (el timer se reinicia automáticamente) */
    timer_continuous_mode(TIM5);
    
    /* 
     * Configurar período máximo (32-bit):
     * Con 1 MHz, el timer hace overflow cada:
     * 2^32 µs = 4,294,967,296 µs ≈ 4295 segundos ≈ 71.6 minutos
     */
    timer_set_period(TIM5, 0xFFFFFFFF);
    
    /* Reiniciar el contador a 0 */
    timer_set_counter(TIM5, 0);
    
    /* Habilitar el contador */
    timer_enable_counter(TIM5);
}

/* ============================================================================
 * CONFIGURACIÓN DE GPIO
 * ============================================================================ */

/**
 * @brief Configura los pines GPIO para ambos sensores
 */
static void gpio_setup(void)
{
    /* Configurar SENSOR 1 */
    gpio_mode_setup(TRIG1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG1_PIN);
    gpio_set_output_options(TRIG1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG1_PIN);
    gpio_clear(TRIG1_PORT, TRIG1_PIN);
    
    /* Pull-down en ECHO para evitar lecturas flotantes */
    gpio_mode_setup(ECHO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO1_PIN);
    
    /* Configurar SENSOR 2 */
    gpio_mode_setup(TRIG2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG2_PIN);
    gpio_set_output_options(TRIG2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG2_PIN);
    gpio_clear(TRIG2_PORT, TRIG2_PIN);
    
    gpio_mode_setup(ECHO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO2_PIN);
}

/* ============================================================================
 * FUNCIONES DE TEMPORIZACIÓN
 * ============================================================================ */

/**
 * @brief Calcula diferencia de tiempo con manejo de overflow
 * @param start Tiempo inicial del contador TIM5
 * @param end Tiempo final del contador TIM5
 * @return Diferencia en microsegundos
 */
static inline uint32_t timer_diff(uint32_t start, uint32_t end)
{
    if (end >= start) {
        return end - start;
    } else {
        /* Overflow del contador de 32 bits */
        return (0xFFFFFFFF - start) + end + 1;
    }
}

/**
 * @brief Delay preciso en microsegundos usando TIM5
 * @param us Microsegundos a esperar (máximo ~4,294 segundos)
 */
static inline void delay_us(uint32_t us)
{
    if (us == 0) return;
    
    uint32_t start = timer_get_counter(TIM5);
    uint32_t target = start + us;
    
    /* Manejo de overflow del timer */
    if (target >= start) {
        /* No hay overflow */
        while (timer_get_counter(TIM5) < target) {
            __asm__("nop");
        }
    } else {
        /* Hay overflow: esperar hasta wrap-around */
        while (timer_get_counter(TIM5) >= start) {
            __asm__("nop");
        }
        /* Luego esperar hasta el target */
        while (timer_get_counter(TIM5) < target) {
            __asm__("nop");
        }
    }
}

/**
 * @brief Delay en milisegundos
 * @param ms Milisegundos a esperar
 */
static inline void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

/* ============================================================================
 * FUNCIONES DE MEDICIÓN DEL SENSOR
 * ============================================================================ */

/**
 * @brief Lee el pulso ECHO del sensor ultrasónico
 * @param trig_port Puerto GPIO del trigger
 * @param trig_pin Pin GPIO del trigger
 * @param echo_port Puerto GPIO del echo
 * @param echo_pin Pin GPIO del echo
 * @param reading Estructura donde se almacena el resultado
 */
static void hcsr05_read_pulse(uint32_t trig_port, uint16_t trig_pin, 
                               uint32_t echo_port, uint16_t echo_pin,
                               sensor_reading_t *reading)
{
    uint32_t start_time, end_time, timeout_start;
    
    /* Inicializar resultado */
    reading->error = SENSOR_OK;
    reading->pulse_us = 0;
    reading->distance_cm = -1.0f;
    
    /* Asegurar que el trigger está bajo */
    gpio_clear(trig_port, trig_pin);
    delay_us(SENSOR_SETTLING_US);
    
    /* Enviar pulso de trigger (10 µs) */
    gpio_set(trig_port, trig_pin);
    delay_us(TRIGGER_PULSE_US);
    gpio_clear(trig_port, trig_pin);
    
    /* Esperar flanco de subida del ECHO con timeout */
    timeout_start = timer_get_counter(TIM5);
    while (gpio_get(echo_port, echo_pin) == 0) {
        if (timer_diff(timeout_start, timer_get_counter(TIM5)) > TIMEOUT_ECHO_START_US) {
            reading->error = SENSOR_TIMEOUT_START;
            return;
        }
    }
    
    /* Capturar tiempo de inicio con TIM5 */
    start_time = timer_get_counter(TIM5);
    
    /* Esperar flanco de bajada del ECHO con timeout */
    while (gpio_get(echo_port, echo_pin) != 0) {
        if (timer_diff(start_time, timer_get_counter(TIM5)) > TIMEOUT_ECHO_END_US) {
            reading->error = SENSOR_TIMEOUT_END;
            return;
        }
    }
    
    /* Capturar tiempo final */
    end_time = timer_get_counter(TIM5);
    
    /* Calcular duración del pulso */
    reading->pulse_us = timer_diff(start_time, end_time);
    
    /* Validar rango del pulso */
    if (reading->pulse_us < MIN_VALID_PULSE_US || 
        reading->pulse_us > MAX_VALID_PULSE_US) {
        reading->error = SENSOR_INVALID_PULSE;
        return;
    }
    
    /* Calcular distancia en centímetros */
    reading->distance_cm = (float)reading->pulse_us / US_PER_CM;
    
    /* Verificar rango físico del sensor */
    if (reading->distance_cm < MIN_DISTANCE_CM || 
        reading->distance_cm > MAX_DISTANCE_CM) {
        reading->error = SENSOR_OUT_OF_RANGE;
    }
}

/* ============================================================================
 * FUNCIONES DE UTILIDAD
 * ============================================================================ */

/**
 * @brief Obtiene string descriptivo del error
 */
static const char* get_error_string(sensor_error_t error)
{
    switch (error) {
        case SENSOR_OK:              return "OK";
        case SENSOR_TIMEOUT_START:   return "TIMEOUT START";
        case SENSOR_TIMEOUT_END:     return "TIMEOUT END";
        case SENSOR_OUT_OF_RANGE:    return "OUT OF RANGE";
        case SENSOR_INVALID_PULSE:   return "INVALID PULSE";
        default:                     return "UNKNOWN";
    }
}

/* ============================================================================
 * PROGRAMA PRINCIPAL
 * ============================================================================ */

int main(void)
{
    char buf[128];
    uint32_t measurement_count = 0;
    sensor_reading_t reading1, reading2;
    
    /* Configurar reloj del sistema */
    clock_setup();
    
    /* Configurar GPIO */
    gpio_setup();
    
    /* Configurar TIM5 para temporización de 1 µs */
    tim5_setup();
    
    /* Inicializar consola UART */
    console_setup(115200);
    
    console_puts("\n==========================================\n");
    console_puts("  Dual HY-SRF05 - Timer-Based Version\n");
    console_puts("==========================================\n");
    console_puts("Sensor 1: PB6 (Trig), PB7 (Echo)\n");
    console_puts("Sensor 2: PE2 (Trig), PE3 (Echo)\n");
    snprintf(buf, sizeof(buf), "Rango: %.0f-%.0f cm\n", MIN_DISTANCE_CM, MAX_DISTANCE_CM);
    console_puts(buf);
    
    /* Mostrar información del timer */
    console_puts("\nTimer Configuration:\n");
    snprintf(buf, sizeof(buf), "- TIM5CLK: %lu Hz\n", rcc_apb1_frequency * 2);
    console_puts(buf);
    console_puts("- Prescaler: 83\n");
    console_puts("- Resolution: 1 us\n");
    console_puts("- Period: 32-bit (4295 sec)\n\n");
    
    /* Delay de estabilización inicial */
    delay_ms(100);
    
    console_puts("Iniciando mediciones...\n\n");
    
    /* Preparar líneas para actualización */
    console_puts("\n\n\n\n\n\n\n");
    
    while (1) {
        measurement_count++;
        
        /* Leer sensor 1 */
        hcsr05_read_pulse(TRIG1_PORT, TRIG1_PIN, ECHO1_PORT, ECHO1_PIN, &reading1);
        delay_ms(INTER_SENSOR_DELAY_MS);
        
        /* Leer sensor 2 */
        hcsr05_read_pulse(TRIG2_PORT, TRIG2_PIN, ECHO2_PORT, ECHO2_PIN, &reading2);
        delay_ms(INTER_CYCLE_DELAY_MS);
        
        /* Mover cursor 7 líneas arriba: ESC[7A */
        console_puts("\033[7A");
        
        /* Línea 1: Contador de mediciones */
        console_puts("\033[2K\r");
        snprintf(buf, sizeof(buf), "Medicion #%lu", measurement_count);
        console_puts(buf);
        
        /* Línea 2: Separador */
        console_puts("\n\033[2K\r");
        console_puts("----------------------------------------");
        
        /* Línea 3: Sensor 1 - Resultado */
        console_puts("\n\033[2K\r");
        if (reading1.error == SENSOR_OK) {
            snprintf(buf, sizeof(buf), "Sensor 1: %.2f cm (%.0f mm) [%lu us]", 
                    reading1.distance_cm, reading1.distance_cm * 10.0f, reading1.pulse_us);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 1: ERROR - %s", get_error_string(reading1.error));
        }
        console_puts(buf);
        
        /* Línea 4: Sensor 1 - Estado ECHO */
        console_puts("\n\033[2K\r");
        snprintf(buf, sizeof(buf), "          ECHO: %s", 
                gpio_get(ECHO1_PORT, ECHO1_PIN) ? "ALTO (!)" : "BAJO (OK)");
        console_puts(buf);
        
        /* Línea 5: Sensor 2 - Resultado */
        console_puts("\n\033[2K\r");
        if (reading2.error == SENSOR_OK) {
            snprintf(buf, sizeof(buf), "Sensor 2: %.2f cm (%.0f mm) [%lu us]", 
                    reading2.distance_cm, reading2.distance_cm * 10.0f, reading2.pulse_us);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 2: ERROR - %s", get_error_string(reading2.error));
        }
        console_puts(buf);
        
        /* Línea 6: Sensor 2 - Estado ECHO */
        console_puts("\n\033[2K\r");
        snprintf(buf, sizeof(buf), "          ECHO: %s", 
                gpio_get(ECHO2_PORT, ECHO2_PIN) ? "ALTO (!)" : "BAJO (OK)");
        console_puts(buf);
        
        /* Línea 7: Información adicional */
        console_puts("\n\033[2K\r");
        if (reading1.error == SENSOR_OK && reading2.error == SENSOR_OK) {
            float diff = reading1.distance_cm - reading2.distance_cm;
            snprintf(buf, sizeof(buf), "Diferencia: %.2f cm", diff);
        } else {
            snprintf(buf, sizeof(buf), "Timer: %lu us", timer_get_counter(TIM5));
        }
        console_puts(buf);
        
        /* Delay antes de siguiente medición */
        delay_ms(200);
    }
    
    return 0;
}