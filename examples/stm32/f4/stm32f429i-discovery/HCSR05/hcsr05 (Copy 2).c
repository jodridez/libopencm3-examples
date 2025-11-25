/*
 * Dual HY-SRF05 Ultrasonic Sensors - Versión Mejorada
 * STM32F429I-DISC1
 * 
 * Sensor 1: PB6 (Trigger), PB7 (Echo)
 * Sensor 2: PE2 (Trigger), PE3 (Echo)
 * Calibración: 41,666,667 iteraciones = 1 segundo
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>
#include "console.h"

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

/* Calibración precisa basada en tus mediciones */
#define ITERATIONS_PER_SECOND 41666667UL
#define ITERATIONS_PER_MS     (ITERATIONS_PER_SECOND / 1000UL)      // 41,667
#define ITERATIONS_PER_US     (ITERATIONS_PER_SECOND / 1000000UL)   // ~42

/* Timeouts calibrados */
#define TIMEOUT_ECHO_START_MS  10   // 10ms para esperar inicio de echo
#define TIMEOUT_ECHO_END_MS    30   // 30ms máximo para echo (> 5m de distancia)

/* Delay calibrado en microsegundos usando loops */
static void delay_us_loop(uint32_t us)
{
    volatile uint32_t count = us * ITERATIONS_PER_US;
    while (count--) {
        __asm__("nop");
    }
}

/* Delay usando TIM5 (más preciso para mediciones largas) */
static void delay_us_timer(uint32_t us)
{
    timer_set_counter(TIM5, 0);
    while (timer_get_counter(TIM5) < us);
}

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
    gpio_mode_setup(ECHO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ECHO1_PIN);
    
    /* Configurar SENSOR 2 */
    gpio_mode_setup(TRIG2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG2_PIN);
    gpio_set_output_options(TRIG2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, TRIG2_PIN);
    gpio_clear(TRIG2_PORT, TRIG2_PIN);
    gpio_mode_setup(ECHO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ECHO2_PIN);
    
    /* Configurar TIM5 como temporizador libre (1 MHz = 1 µs) */
    timer_disable_counter(TIM5);
    timer_set_prescaler(TIM5, (rcc_apb1_frequency / 1000000) - 1);
    timer_set_period(TIM5, 0xFFFFFFFF);
    timer_set_counter(TIM5, 0);
    timer_enable_counter(TIM5);
}

/* Mide el tiempo de pulso del ECHO en microsegundos - VERSIÓN MEJORADA */
static uint32_t hcsr05_read_pulse(uint32_t trig_port, uint16_t trig_pin, 
                                   uint32_t echo_port, uint16_t echo_pin)
{
    uint32_t start, end;
    volatile uint32_t timeout_count;
    
    /* Enviar pulso de trigger de 10 µs */
    gpio_clear(trig_port, trig_pin);
    delay_us_timer(2);  // Asegurar estado bajo
    gpio_set(trig_port, trig_pin);
    delay_us_timer(10); // Pulso de 10µs
    gpio_clear(trig_port, trig_pin);
    
    /* Esperar flanco de subida con timeout calibrado (10ms) */
    timeout_count = TIMEOUT_ECHO_START_MS * ITERATIONS_PER_MS;
    while (gpio_get(echo_port, echo_pin) == 0 && timeout_count--) {
        __asm__("nop");
    }
    
    if (timeout_count == 0) {
        return 0;
    }
    
    /* Capturar tiempo de inicio */
    start = timer_get_counter(TIM5);
    
    /* Esperar flanco de bajada con timeout calibrado (30ms) */
    timeout_count = TIMEOUT_ECHO_END_MS * ITERATIONS_PER_MS;
    while (gpio_get(echo_port, echo_pin) != 0 && timeout_count--) {
        __asm__("nop");
    }
    
    if (timeout_count == 0) {
        return 0;
    }
    
    /* Capturar tiempo final */
    end = timer_get_counter(TIM5);
    
    /* Calcular duración manejando overflow del timer */
    uint32_t duration;
    if (end >= start) {
        duration = end - start;
    } else {
        /* Overflow del contador de 32 bits */
        duration = (0xFFFFFFFF - start) + end + 1;
    }
    
    return duration;
}

/* Calcula distancia en centímetros - VERSIÓN MEJORADA */
static float hcsr05_get_distance_cm(uint32_t trig_port, uint16_t trig_pin,
                                    uint32_t echo_port, uint16_t echo_pin)
{
    uint32_t pulse_us = hcsr05_read_pulse(trig_port, trig_pin, echo_port, echo_pin);
    
    if (pulse_us == 0) {
        return -1.0f;  // Error
    }
    
    /* 
     * Cálculo de distancia:
     * Velocidad del sonido = 343 m/s = 0.0343 cm/µs
     * Distancia = (tiempo * velocidad) / 2
     * Distancia (cm) = (pulse_us * 0.0343) / 2
     * Simplificado: pulse_us / 58.24
     */
    float distance_cm = (float)pulse_us / 58.24f;
    
    return distance_cm;
}

/* Realiza múltiples mediciones y retorna el promedio (filtrado) */
static float hcsr05_get_distance_filtered(uint32_t trig_port, uint16_t trig_pin,
                                          uint32_t echo_port, uint16_t echo_pin,
                                          uint8_t samples)
{
    float sum = 0.0f;
    uint8_t valid_samples = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        float dist = hcsr05_get_distance_cm(trig_port, trig_pin, echo_port, echo_pin);
        
        if (dist > 0) {
            sum += dist;
            valid_samples++;
        }
        
        /* Esperar 60ms entre mediciones (sensor max 40Hz) */
        // for (volatile uint32_t j = 0; j < 60 * ITERATIONS_PER_MS; j++);
    }
    
    if (valid_samples == 0) {
        return -1.0f;
    }
    
    return sum / valid_samples;
}

int main(void)
{
    char buf[128];
    uint32_t measurement_count = 0;
    
    console_setup(115200);
    console_puts("\n==========================================\n");
    console_puts("  Dual HY-SRF05 Ultrasonic Sensors v2.0\n");
    console_puts("==========================================\n");
    console_puts("Sensor 1: PB6 (Trig), PB7 (Echo)\n");
    console_puts("Sensor 2: PE2 (Trig), PE3 (Echo)\n");
    console_puts("Rango: 2-450 cm\n");
    snprintf(buf, sizeof(buf), "Calibracion: %lu iter/seg\n\n", ITERATIONS_PER_SECOND);
    console_puts(buf);
    
    hcsr05_setup();
    
    console_puts("Iniciando mediciones...\n\n");
    
    /* Líneas para actualizar (7 líneas) */
    console_puts("\n\n\n\n\n\n\n");
    
    while (1) {
        measurement_count++;
        
        /* Leer ambos sensores */
        float dist1 = hcsr05_get_distance_cm(TRIG1_PORT, TRIG1_PIN, ECHO1_PORT, ECHO1_PIN);
        
        /* Esperar 30ms entre sensores (evita interferencia) */
        //for (volatile uint32_t i = 0; i < 30 * ITERATIONS_PER_MS; i++);
        
        float dist2 = hcsr05_get_distance_cm(TRIG2_PORT, TRIG2_PIN, ECHO2_PORT, ECHO2_PIN);
        
        /* Esperar 30ms antes del siguiente ciclo (total ~60ms/ciclo = 16 Hz) */
        // for (volatile uint32_t i = 0; i < 30 * ITERATIONS_PER_MS; i++);
        
        /* Mover cursor 7 líneas arriba: ESC[7A */
        console_puts("\033[7A");
        
        /* Línea 1: Contador de mediciones */
        console_puts("\033[2K\r");
        snprintf(buf, sizeof(buf), "Medicion #%lu", measurement_count);
        console_puts(buf);
        
        /* Línea 2: Separador */
        console_puts("\n\033[2K\r");
        console_puts("----------------------------------------");
        
        /* Línea 3: Sensor 1 - Distancia */
        console_puts("\n\033[2K\r");
        if (dist1 < 0) {
            console_puts("Sensor 1: SIN LECTURA              ");
        } else if (dist1 < 2.0f || dist1 > 450.0f) {
            snprintf(buf, sizeof(buf), "Sensor 1: %.2f cm (FUERA RANGO)   ", dist1);
            console_puts(buf);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 1: %.2f cm (%.1f mm)       ", dist1, dist1 * 10.0f);
            console_puts(buf);
        }
        
        /* Línea 4: Sensor 1 - Estado ECHO */
        console_puts("\n\033[2K\r");
        if (gpio_get(ECHO1_PORT, ECHO1_PIN)) {
            console_puts("          ECHO: ALTO (!)           ");
        } else {
            console_puts("          ECHO: BAJO (OK)          ");
        }
        
        /* Línea 5: Sensor 2 - Distancia */
        console_puts("\n\033[2K\r");
        if (dist2 < 0) {
            console_puts("Sensor 2: SIN LECTURA              ");
        } else if (dist2 < 2.0f || dist2 > 450.0f) {
            snprintf(buf, sizeof(buf), "Sensor 2: %.2f cm (FUERA RANGO)   ", dist2);
            console_puts(buf);
        } else {
            snprintf(buf, sizeof(buf), "Sensor 2: %.2f cm (%.1f mm)       ", dist2, dist2 * 10.0f);
            console_puts(buf);
        }
        
        /* Línea 6: Sensor 2 - Estado ECHO */
        console_puts("\n\033[2K\r");
        if (gpio_get(ECHO2_PORT, ECHO2_PIN)) {
            console_puts("          ECHO: ALTO (!)           ");
        } else {
            console_puts("          ECHO: BAJO (OK)          ");
        }
        
        /* Línea 7: Información adicional */
        console_puts("\n\033[2K\r");
        if (dist1 > 0 && dist2 > 0) {
            float diff = dist1 - dist2;
            snprintf(buf, sizeof(buf), "Diferencia: %.2f cm               ", diff);
            console_puts(buf);
        } else {
            console_puts("                                   ");
        }
        
        /* Esperar ~200ms antes de siguiente medición */
        // for (volatile uint32_t i = 0; i < 200 * ITERATIONS_PER_MS; i++);
    }
    
    return 0;
}