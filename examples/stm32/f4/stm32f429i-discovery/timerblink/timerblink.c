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

/* Definiciones para el servo con periodo de 20ms (ARR = 13124, PRESCALER = 0x00FF) */
#define SERVO_0_PERCENT    0      // 0% duty cycle
#define SERVO_10_PERCENT   1312   // 10% duty cycle (13124 * 0.10)
#define SERVO_50_PERCENT   6562   // 50% duty cycle (13124 * 0.50)
#define SERVO_100_PERCENT  13124  // 100% duty cycle

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
    // TIM1CLK = 2*PCLK = 2*84MHz = 168MHz
    // FCNT = 168MHz/(PRESCALER+1) = 168MHz/256 = 656250 Hz
    // T_timer = (ARR+1)/FCNT = 13125/656250 = 0.02s = 20ms

	/* Enable GPIOG clock. */
	rcc_periph_clock_enable(RCC_GPIOG);

	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Enable GPIOC clock. */
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

    /* Set GPIOB13 as AF1 (TIM1_CH1N) */
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

	/* Timer global mode: Edge-aligned, count up */
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Prescaler configurado para servo: 0x00FF (255) */
	timer_set_prescaler(TIM1, 0x00FF);
	
    timer_disable_preload(TIM1);
    timer_continuous_mode(TIM1);

    /* Periodo configurado para 20ms con prescaler 255 */
	timer_set_period(TIM1, 13124);

	/* Valor inicial del servo (10% duty cycle) */
	timer_set_oc_value(TIM1, TIM_OC1, SERVO_10_PERCENT);
    
    /* Habilitar salida complementaria (TIM_OC1N en PB13) */
    timer_enable_oc_output(TIM1, TIM_OC1N);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);

    /* Habilitar salida principal del timer */
    timer_enable_break_main_output(TIM1);
    timer_disable_break(TIM1);

    /* Habilitar contador */
	timer_enable_counter(TIM1);

	/* Habilitar interrupciones */
	timer_enable_irq(TIM1, TIM_DIER_CC1IE);
	timer_enable_irq(TIM1, TIM_DIER_UIE);
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

/* Función de delay simple basada en ciclos de CPU */
static void delay_ms(uint32_t ms)
{
	/* A 168MHz, aproximadamente 168000 ciclos por ms */
	/* Ajustar el multiplicador según sea necesario */
	for (uint32_t i = 0; i < ms; i++) {
		for (uint32_t j = 0; j < 21000; j++) {
			__asm__("nop");
		}
	}
}

/* Función auxiliar para delay en segundos */
static void delay_seconds(uint32_t seconds)
{
	delay_ms(seconds * 1000);
}

int main(void)
{
	clock_setup();
	gpio_setup();
	tim_setup();

	/* Encender LED inicial */
	gpio_set(LGREENF_PORT, LGREENF);

	/* Rutina infinita del servo */
	while (1) {
		/* 0% duty cycle por 2 segundos */
		timer_set_oc_value(TIM1, TIM_OC1, SERVO_0_PERCENT);
		gpio_clear(LGREENF_PORT, LGREENF); // LED apagado
		delay_seconds(2);
		
		/* 10% duty cycle por 1 segundo */
		timer_set_oc_value(TIM1, TIM_OC1, SERVO_10_PERCENT);
		gpio_set(LGREENF_PORT, LGREENF); // LED encendido
		delay_seconds(1);
		
		/* 100% duty cycle por 3 segundos */
		timer_set_oc_value(TIM1, TIM_OC1, SERVO_100_PERCENT);
		gpio_clear(LGREENF_PORT, LGREENF);
		delay_seconds(3);
		
		/* 50% duty cycle por 1 segundo */
		timer_set_oc_value(TIM1, TIM_OC1, SERVO_50_PERCENT);
		gpio_set(LGREENF_PORT, LGREENF);
		delay_seconds(1);
		
		/* 10% duty cycle por 5 segundos */
		timer_set_oc_value(TIM1, TIM_OC1, SERVO_10_PERCENT);
		gpio_toggle(LGREENF_PORT, LGREENF);
		delay_seconds(5);
	}

	return 0;
}