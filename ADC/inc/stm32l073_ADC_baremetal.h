/**
 * @file    stm32l073_ADC_baremetal.h
 * @brief   Bare-metal, polling-only ADC1 driver for STM32L073RZ (CMSIS-only).
 *
 * Features:
 *  - GPIO analog pin setup (no pull) to minimize leakage
 *  - Single-channel, single-conversion in polling mode
 *  - Optional burst read helper
 *  - Power gating for ultra low-power (enable → read → disable)
 *  - Channel selection by channel number or by GPIO pin
 *
 * API intentionally mirrors the STM32F401 version to keep application code identical.
 * Implementation uses only CMSIS headers (no HAL/LL).
 *
 * @author  sdiana.inst
 * @date    2025-09-01
 */


/* --------------------------------------------------------------------------
 * ADC1 external channel map (STM32F401RE & STM32L073RZ, typical packages)
 *
 * Port/Pin   →  Channel
 * PA0..PA7   →  CH0..CH7
 * PB0        →  CH8
 * PB1        →  CH9
 * PC0..PC5   →  CH10..CH15
 *
 * Notes:
 * - Configure pins in Analog mode (MODER=11, no pull) before using the ADC.
 * - Internal channels (VREFINT, TEMPSENSOR, VBAT) exist but are device-specific;
 *   consult the datasheet/reference manual for their enable procedure and IDs.
 * - On some boards (e.g., Nucleo), Arduino headers often map as:
 *   A0=PA0 (CH0), A1=PA1 (CH1), A2=PA4 (CH4), A3=PB0 (CH8), A4=PC1 (CH11), A5=PC0 (CH10).
 * - Verify against your exact MCU/package pinout; alternate functions may conflict.
 * -------------------------------------------------------------------------- */

#ifndef INC_STM32L073_ADC_BAREMETAL_H_
#define INC_STM32L073_ADC_BAREMETAL_H_


/* -------------------------------------------------------------------------- */
/* Includes                                                                   */
/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32l0xx.h"  /* RCC, GPIO, ADC register definitions for L0 family */

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Default timeout for polling loops (in CPU cycles).
 *
 * Used by the convenience wrapper @ref adc_poll_read() so application code
 * does not need to pick a timeout value explicitly.
 */
#ifndef ADC_POLL_TIMEOUT_DEFAULT_CYCLES
#define ADC_POLL_TIMEOUT_DEFAULT_CYCLES  (100000u)
#endif

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief ADC sampling time encoding (SMPR.SMP).
 *
 * On STM32L0 there is a single SMPR register for all channels; values 0..7
 * select times roughly from 1.5 to 239.5 ADC cycles.
 * Names mirror the F4 version for source compatibility; the actual timings differ.
 */
typedef enum
{
    ADC_SMP_3CYC   = 0u,
    ADC_SMP_15CYC  = 1u,
    ADC_SMP_28CYC  = 2u,
    ADC_SMP_56CYC  = 3u,
    ADC_SMP_84CYC  = 4u,
    ADC_SMP_112CYC = 5u,
    ADC_SMP_144CYC = 6u,
    ADC_SMP_480CYC = 7u
} adc_sample_time_t;

/**
 * @brief ADC clock selection “prescaler” (mapped to CFGR2.CKMODE on L0).
 *
 * - DIV2  → synchronous PCLK/2 (CKMODE=01)
 * - DIV4  → synchronous PCLK/4 (CKMODE=10)
 * - others (DIV6/DIV8) → asynchronous HSI16 (CKMODE=00)
 */
typedef enum
{
    ADC_PRESC_DIV2 = 2,
    ADC_PRESC_DIV4 = 4,
    ADC_PRESC_DIV6 = 6,  /* mapped to async HSI16 on L0 */
    ADC_PRESC_DIV8 = 8   /* mapped to async HSI16 on L0 */
} adc_prescaler_t;

/**
 * @brief GPIO port identifier for convenience wrappers.
 */
typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_H = 7
} gpio_port_t;

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Core API (same as F401)                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Configure a GPIO pin as analog (MODER=11, no pull).
 */
void adc_poll_gpio_analog_init(gpio_port_t port, uint8_t pin);

/**
 * @brief  Initialize ADC1 for single-conversion polling on one channel.
 *
 * Performs: clocking, HSI16 enable (for async mode), calibration, enabling,
 * 12-bit, right align, single conversion, software trigger, and selects the
 * initial channel and sampling time.
 */
void adc_poll_init(uint8_t channel,
                   adc_sample_time_t sample_time,
                   adc_prescaler_t   presc);

/**
 * @brief  Perform one blocking conversion (polling) and return the 12-bit result.
 *
 * @param  out             Destination pointer (must be non-NULL).
 * @param  timeout_cycles  Loop-bound timeout while waiting for EOC.
 * @return true on success, false on NULL or timeout.
 */
bool adc_poll_read_once(uint16_t *out, uint32_t timeout_cycles);

/**
 * @brief  Read N samples back-to-back in polling mode, then power down.
 */
size_t adc_poll_read_burst(uint16_t *buf,
                           size_t    n,
                           uint32_t  timeout_cycles_per_sample);

/**
 * @brief  Power down ADC1 and gate its APB2 clock (low-power).
 */
void adc_poll_power_down(void);

/**
 * @brief  Convenience wrapper: one-shot read with library default timeout.
 */
static inline bool adc_poll_read(uint16_t *out)
{
    return adc_poll_read_once(out, ADC_POLL_TIMEOUT_DEFAULT_CYCLES);
}

/* -------------------------------------------------------------------------- */
/* Channel switching helpers                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Reprogram channel selection (CHSELR) and sampling time (SMPR).
 *
 * Call this after @ref adc_poll_init() to switch between external pins
 * without reinitializing the entire ADC.
 */
void adc_poll_select_channel(uint8_t channel, adc_sample_time_t sample_time);

/**
 * @brief  Map a GPIO port/pin to its ADC1 external channel (STM32L073 mapping).
 *
 * External channels on L073 (typical packages):
 *   - PA0..PA7  → CH0..CH7
 *   - PB0       → CH8
 *   - PB1       → CH9
 *   - PC0..PC5  → CH10..CH15
 * Internal channels (e.g., VREFINT, TEMP, VBAT) are not mapped here.
 */
bool adc_poll_map_pin_to_channel(gpio_port_t port, uint8_t pin, uint8_t *out_channel);

/**
 * @brief  Convenience: select the ADC channel by GPIO pin (maps pin and applies SMPR/CHSELR).
 */
bool adc_poll_select_channel_by_pin(gpio_port_t port, uint8_t pin, adc_sample_time_t sample_time);

#ifdef __cplusplus
}
#endif

/* -------------------------------------------------------------------------- */

#endif /* INC_STM32L073_ADC_BAREMETAL_H_ */

