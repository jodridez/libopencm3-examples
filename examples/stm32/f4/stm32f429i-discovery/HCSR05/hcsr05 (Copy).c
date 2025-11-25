#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "console.h"   // Usa el puerto USB virtual del ST-LINK

#define TRIG_PORT GPIOB
#define TRIG_PIN  GPIO6
#define ECHO_PORT GPIOB
#define ECHO_PIN  GPIO7

static void delay_us(uint32_t us)
{
    timer_set_counter(TIM5, 0);
    while (timer_get_counter(TIM5) < us);
}

static void hcsr05_setup(void)
{
    /* Habilitar relojes */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_TIM5);

    /* Configurar TRIG como salida push-pull */
    gpio_mode_setup(TRIG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TRIG_PIN);
    gpio_clear(TRIG_PORT, TRIG_PIN);

    /* Configurar ECHO como entrada */
    gpio_mode_setup(ECHO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ECHO_PIN);

    /* Configurar TIM5 como temporizador libre (1 MHz = 1 µs) */
    timer_disable_counter(TIM5);
    timer_set_prescaler(TIM5, (rcc_apb1_frequency / 1000000) - 1);
    timer_set_period(TIM5, 0xFFFFFFFF);
    timer_set_counter(TIM5, 0);
    timer_enable_counter(TIM5);
}


/* Mide el tiempo de pulso del ECHO en microsegundos */
static uint32_t hcsr05_read_pulse(void)
{
    uint32_t start, end;
    int timeout;

    /* Enviar pulso de 10 µs en TRIG */
    gpio_clear(TRIG_PORT, TRIG_PIN);
    delay_us(2);
    gpio_set(TRIG_PORT, TRIG_PIN);
    delay_us(10);
    gpio_clear(TRIG_PORT, TRIG_PIN);

    /* Esperar flanco de subida */
    timeout = 1000000;
    while (gpio_get(ECHO_PORT, ECHO_PIN) == 0 && timeout--) ;
    if (timeout <= 0) {
        console_puts("Timeout esperando flanco de subida.\n");
        return 0;
    }

    start = timer_get_counter(TIM5);

    /* Esperar flanco de bajada */
    timeout = 1000000;
    while (gpio_get(ECHO_PORT, ECHO_PIN) != 0 && timeout--) ;
    if (timeout <= 0) {
        console_puts("Timeout esperando flanco de bajada.\n");
        return 0;
    }

    end = timer_get_counter(TIM5);

    if (end > start)
        return end - start;
    else
        return (0xFFFFFFFF - start + end);
}

/* Calcula distancia en centímetros */
static float hcsr05_get_distance_cm(void)
{
    uint32_t pulse = hcsr05_read_pulse();
    if (pulse == 0)
        return -1.0f;

    /* Distancia (cm) = (tiempo_us * velocidad_sonido / 2) */
    return (pulse * 0.0343f) / 2.0f;
}

int main(void)
{
    console_setup(115200); // salida por /dev/ttyACM0
    console_puts("\n=== HC-SR05 Debug Start ===\n");

    hcsr05_setup();

    while (1) {
        float dist = hcsr05_get_distance_cm();

        if (dist < 0)
            console_puts("Sin lectura (timeout o sin eco)\n");
        else {
            char buf[64];
            snprintf(buf, sizeof(buf), "Distancia: %.2f cm\n", dist);
            console_puts(buf);
        }

        /* Estado actual del pin ECHO para depuración */
        if (gpio_get(ECHO_PORT, ECHO_PIN))
            console_puts("ECHO=1\n");
        else
            console_puts("ECHO=0\n");

        for (volatile uint32_t i = 0; i < 2000000; i++); // ~200 ms
    }

    return 0;
}
