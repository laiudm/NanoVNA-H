/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include "ch.h"
#include "hal.h"
#include "nanovna.h"

#define USE_ADC_LPF
//#define USE_VBAT_MEDIAN_FILTER

#ifdef USE_ADC_LPF
	// VBAT ADC low pass filter(s)
	#define ADC_LPF_SIZE         2			// number of sequencial LPF's
	#define ADC_LPF_SCALE        3			// the higher this scaling value is the stronger the LPF'ing
	uint8_t  adc_lpf_count     = 0;			// pre-charge LPF counter .. this gets the LPF's to their typical levels before LPF starts
	uint32_t vbat_lpf_value[ADC_LPF_SIZE];	// vbat LPF state(s)
#endif

#ifdef USE_VBAT_MEDIAN_FILTER
	// VBAT ADC median filter
	#define VBAT_MEDIAN_FILTER_BUF_SIZE   11
  	uint16_t vbat_median_buf_in[VBAT_MEDIAN_FILTER_BUF_SIZE];
  	uint16_t vbat_median_buf_out[VBAT_MEDIAN_FILTER_BUF_SIZE];
  	uint16_t vbat_median_buf_wr = 0;
#endif

 uint32_t vrefint = 0;

#define ADC_TR(low, high)      (((uint32_t)(high) << 16U) | (uint32_t)(low))
#define ADC_SMPR_SMP_1P5       0U  /**< @brief 14 cycles conversion time   */
#define ADC_SMPR_SMP_239P5     7U  /**< @brief 252 cycles conversion time. */
#define ADC_CFGR1_RES_12BIT    (0U << 3U)

// External Event Select for regular group
#define ADC_TIM1_TRGO          0                                       // 0b000
#define ADC_TIM1_CC4           (ADC_CFGR1_EXTSEL_0)                    // 0b001
#define ADC_TIM2_TRGO          (ADC_CFGR1_EXTSEL_1)                    // 0b010
#define ADC_TIM3_TRGO          (ADC_CFGR1_EXTSEL_1|ADC_CFGR1_EXTSEL_0) // 0b011
#define ADC_TIM15_TRGO         (ADC_CFGR1_EXTSEL_2)                    // 0b100

#define VNA_ADC                ADC1

void adc_init(void)
{
	rccEnableADC1(FALSE);

	#ifdef USE_ADC_LPF
		adc_lpf_count = 0;
  		for (int i = 0; i < ADC_LPF_SIZE; i++)
  			vbat_lpf_value[i] = 0;
	#endif

  /* Ensure flag states */
  VNA_ADC->IER = 0;

  /* Calibration procedure.*/
  ADC->CCR = 0;
  if (VNA_ADC->CR & ADC_CR_ADEN)
    VNA_ADC->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  while (VNA_ADC->CR & ADC_CR_ADEN);
  VNA_ADC->CFGR1 &= ~ADC_CFGR1_DMAEN;
  VNA_ADC->CR |= ADC_CR_ADCAL;
  while (VNA_ADC->CR & ADC_CR_ADCAL);

  if (VNA_ADC->ISR & ADC_ISR_ADRDY)
    VNA_ADC->ISR |= ADC_ISR_ADRDY; /* clear ADRDY */

  /* Enable ADC */
  VNA_ADC->CR |= ADC_CR_ADEN;
  while (!(VNA_ADC->ISR & ADC_ISR_ADRDY));
}

uint16_t adc_single_read(uint32_t chsel)
{
  /* ADC setup */
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = 0;
  VNA_ADC->TR     = ADC_TR(0, 0);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_239P5;
  VNA_ADC->CFGR1  = ADC_CFGR1_RES_12BIT;
  VNA_ADC->CHSELR = chsel;

  VNA_ADC->CR |= ADC_CR_ADSTART;	/// ADC conversion start
  while (VNA_ADC->CR & ADC_CR_ADSTART);

  return VNA_ADC->DR;
}

int16_t adc_vbat_read(void)
{
	// ****************************
	// sample the ADC

	adc_stop();

	ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_VBATEN;

	// VREFINT == ADC_IN17

	uint32_t adc_vref = adc_single_read(ADC_CHSELR_CHSEL17);

	// VBAT == ADC_IN18
	// VBATEN enables resistive divider circuit. It consume vbat power.
	uint32_t adc_vbat = adc_single_read(ADC_CHSELR_CHSEL18);

	ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN);

	touch_start_watchdog();

	// ****************************
	// process the ADC samples

	#ifdef USE_ADC_LPF
		// this is used for LPF pre-charging
		if (adc_lpf_count < 255)
			adc_lpf_count++;
	#endif

	// 13.9 Temperature sensor and internal reference voltage
	// VREFINT_CAL calibrated on 3.3V, need get value in mV
	#define ADC_FULL_SCALE   3300
	#define VREFINT_CAL      (*((uint16_t *)0x1FFFF7BA))
/*
	#ifdef USE_ADC_LPF
		// LPF the reading as the ADC readings are a little noisy
		if (adc_lpf_count < 5)
			vref_lpf_value = adc_vref << ADC_LPF_SCALE;													// pre-charge the LPF
		else
		{
			vref_lpf_value = vref_lpf_value - (vref_lpf_value >> ADC_LPF_SCALE) + adc_vref;	// LPF the vref reading
			adc_vref = vref_lpf_value >> ADC_LPF_SCALE;
		}
	#endif
*/
	// vbat_raw = (3300 * 2 * vbat / 4095) * (VREFINT_CAL / vrefint)
	// uint16_t vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * (float)vbat * 2 / (vrefint * ((1<<12)-1)));
	// For speed divide not on 4095, divide on 4096, get little error, but no matter
	uint16_t bat_mv = ((uint64_t)(ADC_FULL_SCALE * 2 * VREFINT_CAL) * adc_vbat) / (adc_vref << 12);

	if (bat_mv < 100)
		return -1;	// problem or missing/faulty vbat series diode

	// compensate for the vbat diode voltage drop
	bat_mv += config.vbat_offset_mv;

   #ifdef USE_VBAT_MEDIAN_FILTER
   	vbat_median_buf_in[vbat_median_buf_wr] = bat_mv;
   	if (++vbat_median_buf_wr >= VBAT_MEDIAN_FILTER_BUF_SIZE)
   		vbat_median_buf_wr = 0;
   	insert_sort(vbat_median_buf_in, vbat_median_buf_out, VBAT_MEDIAN_FILTER_BUF_SIZE);
   	bat_mv = vbat_median_buf_out[VBAT_MEDIAN_FILTER_BUF_SIZE / 2];
	#endif

	#ifdef USE_ADC_LPF
   	{	// LPF the ADC reading as they are a little noisy
   		uint32_t value = bat_mv;
   		for (int i = 0; i < ADC_LPF_SIZE; i++)
   		{
   			if (adc_lpf_count < 5)
   			{
   				vbat_lpf_value[i] = value << ADC_LPF_SCALE;															// pre-charge the LPF
   			}
   			else
   			{
   				vbat_lpf_value[i] = vbat_lpf_value[i] - (vbat_lpf_value[i] >> ADC_LPF_SCALE) + value;	// LPF the battery reading
   				value = vbat_lpf_value[i] >> ADC_LPF_SCALE;
   			}
   		}
  			bat_mv = value;
   	}
	#endif

	return bat_mv;	// return the reading
}

void adc_start_analog_watchdogd(uint32_t chsel, uint16_t threshold)
{
  // ADC setup, if it is defined a callback for the analog watch dog then it is enabled.
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = ADC_IER_AWDIE;
  VNA_ADC->TR     = ADC_TR(0, threshold);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_1P5;
  VNA_ADC->CHSELR = chsel;

  /* ADC configuration and start.*/
  VNA_ADC->CFGR1  = ADC_CFGR1_RES_12BIT | ADC_CFGR1_AWDEN
                  | ADC_CFGR1_EXTEN_0 // rising edge of external trigger
                  | ADC_TIM3_TRGO; // External trigger is timer TIM3
  /* ADC conversion start.*/
  VNA_ADC->CR |= ADC_CR_ADSTART;
}

void adc_stop(void)
{
  if (VNA_ADC->CR & ADC_CR_ADEN) {
    if (VNA_ADC->CR & ADC_CR_ADSTART) {
      VNA_ADC->CR |= ADC_CR_ADSTP;
      while (VNA_ADC->CR & ADC_CR_ADSTP)
        ;
    }

    /*    VNA_ADC->CR |= ADC_CR_ADDIS;
    while (VNA_ADC->CR & ADC_CR_ADDIS)
    ;*/
  }
}

static void adc_interrupt(void)
{
  uint32_t isr = VNA_ADC->ISR;
  VNA_ADC->ISR = isr;

  if (isr & ADC_ISR_OVR) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/

  }
  if (isr & ADC_ISR_AWD) {
    /* Analog watchdog error.*/
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt();

  OSAL_IRQ_EPILOGUE();
}
