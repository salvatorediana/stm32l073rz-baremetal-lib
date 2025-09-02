/**
 * @file    stm32l073_ADC_baremetal.c
 * @brief   Bare-metal, polling-only ADC1 driver for STM32L073RZ (CMSIS-only).
 *
 * Differences vs. F4:
 *  - ADC registers: CR / CFGR1 / CFGR2 / SMPR / CHSELR / ISR / IER
 *  - Channel selection via CHSELR bitmask (single channel at a time here)
 *  - Requires calibration (ADC_CR_ADCAL) before enabling (ADEN)
 *  - Start with ADSTART; EOC flag in ISR; clear flags by writing 1s to ISR
 */


#include "stm32l073_ADC_baremetal.h"


/* -----------------------------------------------------------------------------
 * Private Macros & Utilities
 * -------------------------------------------------------------------------- */

/** Create a single-bit mask at position n (unsigned long to avoid sign issues). */
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

/**
 * @brief  Tiny busy-wait to allow a few core cycles to elapse.
 * @note   Not time-accurate; used only for very short stabilization gaps.
 */
static inline void tiny_delay(volatile uint32_t cycles)
{
    while (cycles--) { __asm__ volatile ("nop"); }
}

/**
 * @brief  Map a gpio_port_t enum to the corresponding GPIO_TypeDef*.
 */
static GPIO_TypeDef* port_to_gpio(gpio_port_t p)
{
    switch (p) {
        case GPIO_PORT_A: return GPIOA;
        case GPIO_PORT_B: return GPIOB;
        case GPIO_PORT_C: return GPIOC;
        case GPIO_PORT_D: return GPIOD;
        case GPIO_PORT_E: return GPIOE;
        case GPIO_PORT_H: return GPIOH;
        default:          return GPIOA; /* Safe fallback */
    }
}

/* -----------------------------------------------------------------------------
 * Low-level ADC helpers (STM32L0)
 * -------------------------------------------------------------------------- */

/**
 * @brief  Configure sampling time (single SMPR for all channels on L0).
 * @param  smp  Encoded sampling time [0..7].
 */
static inline void adc_set_sample_time(uint8_t smp)
{
    ADC1->SMPR = (uint32_t)(smp & 0x7U);
}

/**
 * @brief  Select exactly one regular channel via CHSELR bitmask.
 * @param  channel  Channel index (0..18 depending on bonding).
 */
static inline void adc_set_single_channel(uint8_t channel)
{
    ADC1->CHSELR = (uint32_t)1U << (channel & 31U);
    /* A read-back is sometimes recommended to ensure the bus write completes
       before starting the conversion on some low-power buses. */
    (void)ADC1->CHSELR;
}

/**
 * @brief  Enable HSI16 (for asynchronous ADC clock) and wait for readiness.
 */
static void rcc_enable_hsi16(void)
{
    if ((RCC->CR & RCC_CR_HSION) == 0U) {
        RCC->CR |= RCC_CR_HSION;
        while ((RCC->CR & RCC_CR_HSIRDY) == 0U) { /* wait HSI16 ready */ }
    }
}

/* -----------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

void adc_poll_gpio_analog_init(gpio_port_t port, uint8_t pin)
{
    /* 1) Enable GPIO peripheral clock (IOPENR bit matches port enum index) */
    RCC->IOPENR |= (1UL << (uint32_t)port);

    /* 2) Enter true analog mode (MODER=11) and ensure no pull (PUPDR=00) */
    GPIO_TypeDef *GPIOx = port_to_gpio(port);
    const uint32_t pos  = (uint32_t)pin * 2U;

    GPIOx->MODER = (GPIOx->MODER & ~(0x3UL << pos)) | (0x3UL << pos);
    GPIOx->PUPDR &= ~(0x3UL << pos);
}

void adc_poll_init(uint8_t channel, adc_sample_time_t sample_time, adc_prescaler_t presc)
{
    /* Sanity: clamp sample time encoding */
    if ((uint8_t)sample_time > 7U) {
        sample_time = (adc_sample_time_t)7U;
    }

    /* 1) Enable ADC clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* 2) ADC clock source / mode:
          - For DIV2/DIV4, select synchronous PCLK/2 or PCLK/4 via CFGR2.CKMODE.
          - Otherwise, use asynchronous HSI16 (CKMODE=00) and ensure HSI16 is ON. */
    if (presc == ADC_PRESC_DIV2) {
        ADC1->CFGR2 = (ADC1->CFGR2 & ~ADC_CFGR2_CKMODE) | (0x1U << ADC_CFGR2_CKMODE_Pos); /* PCLK/2 */
    } else if (presc == ADC_PRESC_DIV4) {
        ADC1->CFGR2 = (ADC1->CFGR2 & ~ADC_CFGR2_CKMODE) | (0x2U << ADC_CFGR2_CKMODE_Pos); /* PCLK/4 */
    } else {
        ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* CKMODE = 00 (async) */
        rcc_enable_hsi16();               /* ensure HSI16 ready for async clock */
    }

    /* 3) Make sure ADC is disabled before calibration */
    if ((ADC1->CR & ADC_CR_ADEN) != 0U) {
        ADC1->CR |= ADC_CR_ADDIS;
        while ((ADC1->CR & ADC_CR_ADEN) != 0U) { /* wait ADDIS complete */ }
    }

    /* 4) Calibrate (single-ended) */
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0U) { /* wait calibration end */ }

    /* 5) Basic configuration: 12-bit, right align, single conversion, SW trigger,
          EOC on each conversion, no DMA (pure polling). */
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;      /* 12-bit */
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;    /* Right align */
    ADC1->CFGR1 &= ~ADC_CFGR1_CONT;     /* Single conversion */
    ADC1->CFGR1 &= ~ADC_CFGR1_WAIT;     /* No auto-wait */
    ADC1->CFGR1 &= ~ADC_CFGR1_AUTOFF;   /* No auto-off */
    ADC1->CFGR1 &= ~ADC_CFGR1_DISCEN;   /* No discontinuous */

    /* 6) Sampling time & initial channel */
    adc_set_sample_time((uint8_t)sample_time);
    adc_set_single_channel(channel);

    /* 7) Enable ADC and wait for ADRDY */
    ADC1->ISR |= ADC_ISR_ADRDY;         /* Clear leftover ADRDY, if any */
    ADC1->CR  |= ADC_CR_ADEN;           /* Enable */
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) { /* wait ready */ }
    ADC1->ISR |= ADC_ISR_ADRDY;         /* Clear ADRDY by writing 1 */
}

bool adc_poll_read_once(uint16_t *out, uint32_t timeout_cycles)
{
    if (out == NULL) {
        return false;
    }

    /* Ensure clock is enabled (idempotent) and ADC is ready */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        /* Re-enable quickly if powered down */
        ADC1->ISR |= ADC_ISR_ADRDY;
        ADC1->CR  |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) { }
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    /* Start conversion */
    ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR; /* clear stale flags */
    ADC1->CR  |= ADC_CR_ADSTART;

    /* Poll EOC with loop-bounded timeout */
    uint32_t t = timeout_cycles;
    while (((ADC1->ISR & ADC_ISR_EOC) == 0U) && (t-- > 0U)) {
        __asm__ volatile ("nop");
    }
    if ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
        return false; /* timed out */
    }

    /* Read result (12-bit right-aligned); reading DR clears EOC */
    *out = (uint16_t)(ADC1->DR & 0xFFFFU);

    /* Optionally clear EOS/OVR if set */
    ADC1->ISR |= ADC_ISR_EOS | ADC_ISR_OVR;

    return true;
}

size_t adc_poll_read_burst(uint16_t *buf, size_t n, uint32_t timeout_cycles_per_sample)
{
    if ((buf == NULL) || (n == 0U)) {
        return 0U;
    }

    /* Keep ADC on for the whole burst */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    if ((ADC1->CR & ADC_CR_ADEN) == 0U) {
        ADC1->ISR |= ADC_ISR_ADRDY;
        ADC1->CR  |= ADC_CR_ADEN;
        while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) { }
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    size_t i = 0U;
    for (; i < n; ++i) {
        ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
        ADC1->CR  |= ADC_CR_ADSTART;

        uint32_t t = timeout_cycles_per_sample;
        while (((ADC1->ISR & ADC_ISR_EOC) == 0U) && (t-- > 0U)) {
            __asm__ volatile ("nop");
        }
        if ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
            break; /* timeout on this sample */
        }

        buf[i] = (uint16_t)(ADC1->DR & 0xFFFFU);
        ADC1->ISR |= ADC_ISR_EOS | ADC_ISR_OVR;
    }

    adc_poll_power_down();
    return i;
}

void adc_poll_power_down(void)
{
    /* Disable ADC if enabled */
    if ((ADC1->CR & ADC_CR_ADEN) != 0U) {
        ADC1->CR |= ADC_CR_ADDIS;
        /* Wait until ADEN bit is cleared by hardware */
        uint32_t guard = 10000U;
        while ((ADC1->CR & ADC_CR_ADEN) != 0U && guard--) {
            __asm__ volatile ("nop");
        }
    }

    /* Gate its APB2 clock */
    RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;

    /* Do NOT disable HSI16 here: it may be used elsewhere in the system.
       If you own HSI16, you can add an optional project-level switch to turn it off. */
}

/* -----------------------------------------------------------------------------
 * Channel switching & pin mapping
 * -------------------------------------------------------------------------- */

void adc_poll_select_channel(uint8_t channel, adc_sample_time_t sample_time)
{
    if ((uint8_t)sample_time > 7U) {
        sample_time = (adc_sample_time_t)7U;
    }

    /* It is the caller's responsibility to not call this mid-conversion. */
    adc_set_sample_time((uint8_t)sample_time);
    adc_set_single_channel(channel);
}

/**
 * External channels on STM32L073 (typical):
 *   PA0..PA7  -> CH0..CH7
 *   PB0       -> CH8
 *   PB1       -> CH9
 *   PC0..PC5  -> CH10..CH15
 * (Internal channels are not mapped by this helper.)
 */
bool adc_poll_map_pin_to_channel(gpio_port_t port, uint8_t pin, uint8_t *out_channel)
{
    if (out_channel == NULL) {
        return false;
    }

    switch (port) {
        case GPIO_PORT_A:
            if (pin <= 7U) { *out_channel = pin; return true; }
            break;

        case GPIO_PORT_B:
            if (pin == 0U) { *out_channel = 8U;  return true; }
            if (pin == 1U) { *out_channel = 9U;  return true; }
            break;

        case GPIO_PORT_C:
            if (pin <= 5U) { *out_channel = (uint8_t)(10U + pin); return true; }
            break;

        default:
            break;
    }
    return false;
}

bool adc_poll_select_channel_by_pin(gpio_port_t port, uint8_t pin, adc_sample_time_t sample_time)
{
    uint8_t ch;
    if (!adc_poll_map_pin_to_channel(port, pin, &ch)) {
        return false;
    }
    adc_poll_select_channel(ch, sample_time);
    return true;
}
