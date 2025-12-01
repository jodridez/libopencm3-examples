#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "clock.h"

/* milliseconds since boot */
static volatile uint32_t system_millis;

/* Called when systick fires */
void sys_tick_handler(void)
{
    system_millis++;
}

/* simple sleep for delay milliseconds */
void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis);
}

/* Getter function for the current time */
uint32_t mtime(void)
{
    return system_millis;
}

/*
 * Robust clock_setup:
 * - Start SysTick right away using HSI (16 MHz) to ensure mtime() runs
 * - Then configure PLL (may block or take time)
 * - After PLL is up, reconfigure SysTick for 168 MHz (1 ms)
 */
void clock_setup(void)
{
    /* --- 1) Start SysTick immediately using HSI (16 MHz) so mtime() works now --- */
    const uint32_t HSI_FREQ_HZ = 16000000UL;
    uint32_t reload_hsi = (HSI_FREQ_HZ / 1000U) - 1U; /* ticks for 1 ms at HSI */
    systick_set_reload(reload_hsi);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB); /* uses core/AHB (currently HSI) */
    systick_counter_enable();
    systick_interrupt_enable();

    /* small pause to let first ticks occur (not required but avoids races) */
    for (volatile int i = 0; i < 1000; ++i) __asm__("nop");

    /* --- 2) Now set system clock to 168 MHz (this may internally wait/lock) --- */
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    /* --- 3) SysTick must be reconfigured for the new AHB frequency (168 MHz) --- */
    const uint32_t AHB_168MHZ = 168000000UL;
    uint32_t reload_ahb = (AHB_168MHZ / 1000U) - 1U; /* ticks for 1 ms at 168 MHz */
    systick_set_reload(reload_ahb);
    /* clocksource already set to AHB; counter and interrupt remain enabled */

    /* optional short delay to allow SysTick to stabilize at new rate */
    for (volatile int i = 0; i < 1000; ++i) __asm__("nop");
}
