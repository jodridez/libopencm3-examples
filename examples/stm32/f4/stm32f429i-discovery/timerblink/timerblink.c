/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Damjan Marion <damjan.marion@gmail.com>
 * Copyright (C) 2011 Mark Panajotovic <marko@electrontube.org>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define LGREENF GPIO13
#define LGREENF_PORT GPIOG
#define LREDF GPIO14
#define LREDF_PORT GPIOG

#define LGREENB GPIO13
#define LGREENB_PORT GPIOB
#define LREDB GPIO5
#define LREDB_PORT GPIOC

#define LCC LREDF_PORT, LREDF
#define LUP LREDB_PORT, LREDB

/*
  Timer 1 clk frequency:
  If TIMPRE == 0: (default)
    if PPRE = DIV1:
      TIM1CLK = PCLK
    else:
      TIM1CLK = 2*PCLK
  else:
    if PPRE = DIV1|DIV2|DIV4:
      TIM1CLK = HCLK
    else:
      TIM1CLK = 4*PCLK
 */


/* Set STM32 to 168 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    // AHB Prescaler = 1 (NODIV) libopencm3/lib/stm32/f4/rcc.c
    // APB1: DIV4
    // APB2: DIV2
    // SYSCLK: 168MHz
    // AHBCLK (HCLK or FCLK): SYSCLK/AHBPre = 168MHz/1
    // APB1: 168/4 = 42MHz
    // APB2: 168/2 = 84MHz
    // rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency
    // are set here (libopencm3/lib/stm32/f4/rcc.c)
    // TIM1 is on APB2
    // TIMPRE: arriba. Default 0
    // TIM1CLK = 2*PCLK = 2*84MHz
    // TIM1CNT = 168MHz/2^16   (TIM1CLK/(PRESCALER+1))
    // TIM1CNT = 2563.48Hz
    // If TIMPRE == 0 and PCLK > DIV1:
    // FCNT = 2*PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 0 and PCLK == DIV1:
    // FCNT = PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 1 and PCLK > DIV4:
    // FCNT = 4*PCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)
    // If TIMPRE == 1 and PCLK <= DIV4:
    // FCNT = HCLK/(TIMPRESC+1) (PCLK = APB2 for TIM1)

	/* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM1);
}

static void gpio_setup(void)
{
	/* Set GPIO13-14 (in GPIO port G) to 'output push-pull'. */
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO13 | GPIO14);

	/* Set GPIO5 (in GPIO port C) to 'output push-pull'. */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO5);

    /* Set GPIOB13 as AF1 */
    gpio_set_af(LGREENB_PORT, GPIO_AF1, LGREENB);

	/* Set GPIO13 (in GPIO port B) to 'alternate function push-pull'. */
	gpio_mode_setup(LGREENB_PORT, GPIO_MODE_AF,
                    GPIO_PUPD_NONE, LGREENB);


}

static void tim_setup(void)
{
  /* Enable TIM1 clock. */
	rcc_periph_clock_enable(RCC_TIM1);

	/* Enable TIM1 interrupt. */
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);

	/* Reset TIM1 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM1);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_CENTER_3, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 */
	timer_set_prescaler(TIM1, 0xFFFF); // 2563Hz clk
    //timer_set_repetition_counter(TIM1, 15);
    timer_disable_preload(TIM1);
    timer_continuous_mode(TIM1);

    /* Count period */
	timer_set_period(TIM1, 2563);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(TIM1, TIM_OC1, 320); // no usar los negativos

    /* Disable outputs. */
    //timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC1N);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1); // no usar los negativos
    //timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FORCE_HIGH);
    //timer_set_oc_polarity_high(TIM1, TIM_OC1N);
    //timer_set_oc_idle_state_unset(TIM1, TIM_OC1N);
    //time_reset_output_idle(TIM1, TIM_CR2_OIS1N);
    //timer_disable_oc_output(TIM1, TIM_OC2);
    //timer_disable_oc_output(TIM1, TIM_OC2N);
    //timer_disable_oc_output(TIM1, TIM_OC3);
    //timer_disable_oc_output(TIM1, TIM_OC3N);

    /* Generate update event to reload all registers before starting*/
    timer_enable_break_main_output(TIM1);
    //timer_set_disabled_off_state_in_idle_mode(TIM1);
    //timer_set_disabled_off_state_in_run_mode(TIM1);
    timer_disable_break(TIM1);

    /* Counter enable. */
	timer_enable_counter(TIM1);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
    //timer_generate_event(TIM1, TIM_EGR_UG);

}

void tim1_cc_isr(void)
{
  timer_clear_flag(TIM1, TIM_SR_CC1IF);
  gpio_toggle(LCC);
}

void tim1_up_tim10_isr(void)
{
  timer_clear_flag(TIM1, TIM_SR_UIF);
  gpio_toggle(LUP);
}


int main(void)
{
	int i;

	clock_setup();
	gpio_setup();
    tim_setup();

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(LGREENF_PORT, LGREENF);

	/* Blink the LEDs (PG13 and PG14) on the board. */
	while (1) {
		/* Toggle LEDs. */
		gpio_toggle(LGREENF_PORT, LGREENF);
		for (i = 0; i < 6000000; i++) { /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}
