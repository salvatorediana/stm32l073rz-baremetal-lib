/*
 * stm32l073_GPIO_baremetal.h
 * Bare-metal GPIO library for STM32L073 (ULP-friendly)
 */

/**
 * @file    stm32l073_GPIO_baremetal.h
 * @brief   Bare-metal GPIO driver for STM32L073 (STM32L0 family).
 * @details
 *   - Pure CMSIS / register access (no HAL/LL).
 *   - Same API as the STM32F401 version (drop-in compatible).
 *   - On STM32L0, GPIO clocks are gated via RCC->IOPENR.
 * @author Salvatore Diana, email: sdiana517@gmail.com
 */

#ifndef INC_STM32L073RZ_GPIO_BAREMETAL_H_
#define INC_STM32L073RZ_GPIO_BAREMETAL_H_

//#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Minimal descriptor for a GPIO pin. */
typedef struct {
    GPIO_TypeDef *port;  /**< GPIO port base (GPIOA..GPIOx). */
    uint8_t       pin;   /**< Pin index (0..15). */
} bm_gpio_t;

/** Pull configuration. */
typedef enum {
    BM_GPIO_PULL_NONE = 0u,
    BM_GPIO_PULL_UP   = 1u,
    BM_GPIO_PULL_DOWN = 2u,
} bm_gpio_pull_t;

/**
 * Output speed (OSPEEDR encoding 00..11).
 * STM32L0 maps speeds roughly to: ~400 kHz, ~2 MHz, ~10 MHz, up to ~35 MHz.
 */
typedef enum {
    BM_GPIO_SPEED_LOW       = 0u,
    BM_GPIO_SPEED_MEDIUM    = 1u,
    BM_GPIO_SPEED_HIGH      = 2u,
    BM_GPIO_SPEED_VERY_HIGH = 3u,
} bm_gpio_speed_t;

/* --- Port clock control (L0: RCC->IOPENR) --------------------------------- */
void bm_gpio_port_enable(GPIO_TypeDef *port);
void bm_gpio_port_disable(GPIO_TypeDef *port);

/* --- Pin configuration ----------------------------------------------------- */
void bm_gpio_init_output(bm_gpio_t g, bool initial_high, bm_gpio_pull_t pull, bm_gpio_speed_t speed);
void bm_gpio_init_input(bm_gpio_t g, bm_gpio_pull_t pull);
void bm_gpio_deinit_to_analog(bm_gpio_t g);

/* --- Pin I/O --------------------------------------------------------------- */
void bm_gpio_write(bm_gpio_t g, bool high);
void bm_gpio_set(bm_gpio_t g);
void bm_gpio_clear(bm_gpio_t g);
void bm_gpio_toggle(bm_gpio_t g);
bool bm_gpio_read(bm_gpio_t g);

/* --- ULP helper ------------------------------------------------------------ */
void bm_gpio_port_all_analog_except(GPIO_TypeDef *port, uint16_t keep_mask);

#ifdef __cplusplus
}
#endif



#endif /* INC_STM32L073RZ_GPIO_BAREMETAL_H_ */

