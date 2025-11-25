/*
 * Dual HY-SRF05 Ultrasonic Sensors - Versión Optimizada
 * STM32F429I-DISC1
 * 
 * Sensor 1: PB6 (Trigger), PB7 (Echo)
 * Sensor 2: PE2 (Trigger), PE3 (Echo)
 * 
 * MEJORAS:
 * - Manejo robusto de errores y timeouts
 * - Protección contra overflow de timer
 * - Validación de rangos físicos del sensor
 * - Código más eficiente y mantenible
 * - Mejor filtrado de mediciones
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>
#include <stdlib.h>
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
#define TIMEOUT_ECHO_START_US  50000UL  // 50ms para inicio de echo
#define TIMEOUT_ECHO_END_US    50000UL  // 50ms máximo para echo (~8.5m)

/* Límites físicos del sensor HY-SRF05 */
#define MIN_DISTANCE_CM        2.0f
#define MAX_DISTANCE_CM        450.0f
#define MIN_VALID_PULSE_US     116UL    // ~2cm
#define MAX_VALID_PULSE_US     26200UL  // ~450cm

/* Timing del sensor */
#define TRIGGER_PULSE_US       10       // Pulso trigger
#define SENSOR_SETTLING_US     5        // Tiempo de estabilización
#define INTER_SENSOR_DELAY_MS  100      // Delay entre sensores (evita interferencia)
#define INTER_CYCLE_DELAY_MS   100      // Delay entre ciclos

/* Velocidad del sonido: 343 m/s a 20°C */
#define SOUND_SPEED_CM_PER_US  0.0343f
#define US_PER_CM              58.24f   // 1 / (0.0343 / 2)

/* Filtrado de mediciones */
#define MEDIAN_FILTER_SIZE     3
#define MAX_NOISE_CM           5.0f     // Máxima variación aceptable entre lecturas

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
 * VARIABLES GLOBALES
 * ============================================================================ */

static volatile uint32_t timer_frequency = 1000000UL; // 1 MHz

/* ============================================================================
 * FUNCIONES DE TEMPORIZACIÓN
 * ============================================================================ */

/**
 * @brief Delay preciso usando TIM5 (configurado a 1 MHz)
 * @param us Microsegundos a esperar
 */
static inline void delay_us(uint32_t us)
{
    if (us == 0) return;
    
    uint32_t start = timer_get_counter(TIM5);
    uint32_t target = start + us;
    
    /* Manejo de overflow del timer de 32 bits */
    if (target >= start) {
        /* No hay overflow */
        while (timer_get_counter(TIM5) < target);
    } else {
        /* Hay overflow: esperar hasta 0xFFFFFFFF, luego hasta target */
        while (timer_get_counter(TIM5) >= start);
        while (timer_get_counter(TIM5) < target);
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

/**
 * @brief Calcula diferencia de tiempo con manejo de overflow
 * @param start Tiempo inicial
 * @param end Tiempo final
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

/* ============================================================================
 * INICIALIZACIÓN DE HARDWARE
 * ============================================================================ */

/**
 * @brief Configura GPIO y TIM5 para los sensores
 */
static void hcsr05_setup(void)
{
    /* Habilitar relojes */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);
    rcc_periph_clock_enable(RCC_TIM5);
    
    /* Configurar SENSOR 1 */
    gpio_mode_setup(TRIG1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG1_PIN);
    gpio_set_output_options(TRIG1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG1_PIN);
    gpio_clear(TRIG1_PORT, TRIG1_PIN);
    
    gpio_mode_setup(ECHO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO1_PIN);
    
    /* Configurar SENSOR 2 */
    gpio_mode_setup(TRIG2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG2_PIN);
    gpio_set_output_options(TRIG2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG2_PIN);
    gpio_clear(TRIG2_PORT, TRIG2_PIN);
    
    gpio_mode_setup(ECHO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, ECHO2_PIN);
    
    /* Configurar TIM5: temporizador de 32-bit a 1 MHz (1 µs por tick) */
    timer_disable_counter(TIM5);
    
    /* APB1 en STM32F429 @ 180 MHz: APB1=45MHz, TIMxCLK=90MHz */
    uint32_t apb1_freq = rcc_apb1_frequency;
    /* Si APB1 prescaler != 1, entonces TIMxCLK = APB1 * 2 */
    uint32_t cfgr = RCC_CFGR;
    uint32_t ppre1 = (cfgr >> 10) & 0x7; // Bits 10-12 para PPRE1
    if (ppre1 != 0) {  // Si no es división por 1, multiplicar por 2
        apb1_freq *= 2;
    }
    timer_frequency = 1000000UL;
    timer_set_prescaler(TIM5, (apb1_freq / timer_frequency) - 1);
    
    timer_set_period(TIM5, 0xFFFFFFFF);
    timer_disable_preload(TIM5);
    timer_continuous_mode(TIM5);
    timer_set_counter(TIM5, 0);
    timer_enable_counter(TIM5);
    
    /* Pequeño delay para estabilización inicial */
    delay_ms(50);
}

/* ============================================================================
 * FUNCIONES DE MEDICIÓN
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
    
    /* Asegurar que el trigger está bajo - CRÍTICO */
    gpio_clear(trig_port, trig_pin);
    delay_us(5);  // Mínimo 2µs, usamos 5µs para seguridad
    
    /* Asegurar que ECHO está bajo antes de iniciar */
    uint32_t pre_timeout = timer_get_counter(TIM5);
    while (gpio_get(echo_port, echo_pin) != 0) {
        if (timer_diff(pre_timeout, timer_get_counter(TIM5)) > 50000UL) {
            reading->error = SENSOR_TIMEOUT_START;
            return;
        }
    }
    
    /* Enviar pulso de trigger (10 µs) */
    gpio_set(trig_port, trig_pin);
    delay_us(TRIGGER_PULSE_US);
    gpio_clear(trig_port, trig_pin);
    
    /* Delay adicional después del trigger */
    delay_us(5);
    
    /* Esperar flanco de subida del ECHO con timeout */
    timeout_start = timer_get_counter(TIM5);
    while (gpio_get(echo_port, echo_pin) == 0) {
        if (timer_diff(timeout_start, timer_get_counter(TIM5)) > TIMEOUT_ECHO_START_US) {
            reading->error = SENSOR_TIMEOUT_START;
            return;
        }
    }
    
    /* Capturar tiempo de inicio */
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
    
    /* Calcular distancia */
    reading->distance_cm = (float)reading->pulse_us / US_PER_CM;
    
    /* Verificar rango físico del sensor */
    if (reading->distance_cm < MIN_DISTANCE_CM || 
        reading->distance_cm > MAX_DISTANCE_CM) {
        reading->error = SENSOR_OUT_OF_RANGE;
    }
}

/**
 * @brief Compara dos floats para qsort
 */
static int compare_float(const void *a, const void *b)
{
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief Obtiene distancia con filtro de mediana (OPCIONAL - actualmente no usada)
 * @param trig_port Puerto GPIO del trigger
 * @param trig_pin Pin GPIO del trigger
 * @param echo_port Puerto GPIO del echo
 * @param echo_pin Pin GPIO del echo
 * @param samples Número de muestras para el filtro (debe ser impar)
 * @return Distancia filtrada o -1.0 si hay error
 * 
 * Para usar esta función en lugar de hcsr05_read_pulse(), reemplaza las llamadas
 * en main() por:
 *   float dist1 = hcsr05_get_distance_filtered(TRIG1_PORT, TRIG1_PIN, ECHO1_PORT, ECHO1_PIN, 3);
 */
#if 1  // Deshabilitado - habilitar si deseas usar filtro de mediana
static float hcsr05_get_distance_filtered(uint32_t trig_port, uint16_t trig_pin,
                                          uint32_t echo_port, uint16_t echo_pin,
                                          uint8_t samples)
{
    sensor_reading_t reading;
    float readings[MEDIAN_FILTER_SIZE];
    uint8_t valid_count = 0;
    
    /* Limitar número de muestras */
    if (samples > MEDIAN_FILTER_SIZE) {
        samples = MEDIAN_FILTER_SIZE;
    }
    
    /* Tomar múltiples muestras */
    for (uint8_t i = 0; i < samples; i++) {
        hcsr05_read_pulse(trig_port, trig_pin, echo_port, echo_pin, &reading);
        
        if (reading.error == SENSOR_OK) {
            readings[valid_count++] = reading.distance_cm;
        }
        
        if (i < samples - 1) {
            delay_ms(INTER_SENSOR_DELAY_MS);
        }
    }
    
    /* Si no hay lecturas válidas */
    if (valid_count == 0) {
        return -1.0f;
    }
    
    /* Si solo hay una lectura, retornarla */
    if (valid_count == 1) {
        return readings[0];
    }
    
    /* Ordenar y retornar la mediana */
    qsort(readings, valid_count, sizeof(float), compare_float);
    return readings[valid_count / 2];
}
#endif

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
    
    /* Inicializar consola */
    console_setup(115200);
    console_puts("\n==========================================\n");
    console_puts("  Dual HY-SRF05 Sensors - Optimizado v3.0\n");
    console_puts("==========================================\n");
    console_puts("Sensor 1: PB6 (Trig), PB7 (Echo)\n");
    console_puts("Sensor 2: PE2 (Trig), PE3 (Echo)\n");
    snprintf(buf, sizeof(buf), "Rango: %.0f-%.0f cm\n", MIN_DISTANCE_CM, MAX_DISTANCE_CM);
    console_puts(buf);
    snprintf(buf, sizeof(buf), "Timer: %lu Hz\n\n", timer_frequency);
    console_puts(buf);
    
    /* Inicializar hardware */
    hcsr05_setup();
    
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
        
        /* Mover cursor arriba y actualizar pantalla */
        console_puts("\033[7A");
        
        /* Línea 1: Contador */
        console_puts("\033[2K\r");
        snprintf(buf, sizeof(buf), "Medicion #%lu", measurement_count);
        console_puts(buf);
        
        /* Línea 2: Separador */
        console_puts("\n\033[2K\r");
        console_puts("----------------------------------------");
        
        /* Línea 3: Sensor 1 */
        console_puts("\n\033[2K\r");
        if (reading1.error == SENSOR_OK) {
            snprintf(buf, sizeof(buf), "Sensor 1: %.2f cm (%.0f mm) [%lu us]", 
                    reading1.distance_cm, reading1.distance_cm * 10.0f, reading1.pulse_us);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 1: ERROR - %s", get_error_string(reading1.error));
        }
        console_puts(buf);
        
        /* Línea 4: Estado ECHO 1 */
        console_puts("\n\033[2K\r");
        snprintf(buf, sizeof(buf), "          ECHO: %s", 
                gpio_get(ECHO1_PORT, ECHO1_PIN) ? "ALTO (!)" : "BAJO (OK)");
        console_puts(buf);
        
        /* Línea 5: Sensor 2 */
        console_puts("\n\033[2K\r");
        if (reading2.error == SENSOR_OK) {
            snprintf(buf, sizeof(buf), "Sensor 2: %.2f cm (%.0f mm) [%lu us]", 
                    reading2.distance_cm, reading2.distance_cm * 10.0f, reading2.pulse_us);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 2: ERROR - %s", get_error_string(reading2.error));
        }
        console_puts(buf);
        
        /* Línea 6: Estado ECHO 2 */
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
            snprintf(buf, sizeof(buf), "                    ");
        }
        console_puts(buf);
        
        /* Delay antes de siguiente medición */
        delay_ms(200);
    }
    
    return 0;
}