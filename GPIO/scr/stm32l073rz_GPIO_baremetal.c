/*
 * stm32l073_GPIO_baremetal.c
 * Bare-metal GPIO library for STM32L073 (ULP-friendly)
 */


#include "stm32l073rz_GPIO_baremetal.h"



/* Local helpers */
static inline uint32_t _pin_mask(uint8_t pin) { return (1u << pin); }
static inline uint32_t _pos2(uint8_t pin)    { return (uint32_t)pin * 2u; }

/* --- Port clock control (L0: RCC->IOPENR) --------------------------------- */
void bm_gpio_port_enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)      { RCC->IOPENR |= RCC_IOPENR_GPIOAEN; }
    else if (port == GPIOB) { RCC->IOPENR |= RCC_IOPENR_GPIOBEN; }
    else if (port == GPIOC) { RCC->IOPENR |= RCC_IOPENR_GPIOCEN; }
#ifdef GPIOD
    else if (port == GPIOD) { RCC->IOPENR |= RCC_IOPENR_GPIODEN; }
#endif
#ifdef GPIOE
    else if (port == GPIOE) { RCC->IOPENR |= RCC_IOPENR_GPIOEEN; }
#endif
#ifdef GPIOF
    else if (port == GPIOF) { RCC->IOPENR |= RCC_IOPENR_GPIOFEN; }
#endif
#ifdef GPIOG
    else if (port == GPIOG) { RCC->IOPENR |= RCC_IOPENR_GPIOGEN; }
#endif
#ifdef GPIOH
    else if (port == GPIOH) { RCC->IOPENR |= RCC_IOPENR_GPIOHEN; }
#endif
    else { return; }
    (void)RCC->IOPENR; __DSB(); // ensure clock visible before GPIO access
}

void bm_gpio_port_disable(GPIO_TypeDef *port)
{
    if (port == GPIOA)      { RCC->IOPENR &= ~RCC_IOPENR_GPIOAEN; }
    else if (port == GPIOB) { RCC->IOPENR &= ~RCC_IOPENR_GPIOBEN; }
    else if (port == GPIOC) { RCC->IOPENR &= ~RCC_IOPENR_GPIOCEN; }
#ifdef GPIOD
    else if (port == GPIOD) { RCC->IOPENR &= ~RCC_IOPENR_GPIODEN; }
#endif
#ifdef GPIOE
    else if (port == GPIOE) { RCC->IOPENR &= ~RCC_IOPENR_GPIOEEN; }
#endif
#ifdef GPIOF
    else if (port == GPIOF) { RCC->IOPENR &= ~RCC_IOPENR_GPIOFEN; }
#endif
#ifdef GPIOG
    else if (port == GPIOG) { RCC->IOPENR &= ~RCC_IOPENR_GPIOGEN; }
#endif
#ifdef GPIOH
    else if (port == GPIOH) { RCC->IOPENR &= ~RCC_IOPENR_GPIOHEN; }
#endif
}

/* --- Pin configuration ----------------------------------------------------- */
void bm_gpio_init_output(bm_gpio_t g, bool initial_high, bm_gpio_pull_t pull, bm_gpio_speed_t speed)
{
    if (g.pin > 15u) return;
    uint32_t m   = _pin_mask(g.pin);
    uint32_t pos = _pos2(g.pin);

    // 1) Initial level atomically (avoid glitch)
    g.port->BSRR = initial_high ? m : (m << 16);

    // 2) Push-pull
    g.port->OTYPER &= ~m;

    // 3) Speed
    g.port->OSPEEDR = (g.port->OSPEEDR & ~(0x3u << pos)) | ((uint32_t)speed << pos);

    // 4) Pull
    g.port->PUPDR   = (g.port->PUPDR   & ~(0x3u << pos)) | ((uint32_t)pull  << pos);

    // 5) Mode = output (01)
    g.port->MODER   = (g.port->MODER   & ~(0x3u << pos)) | (0x1u << pos);
}

void bm_gpio_init_input(bm_gpio_t g, bm_gpio_pull_t pull)
{
    if (g.pin > 15u) return;
    uint32_t pos = _pos2(g.pin);
    g.port->PUPDR = (g.port->PUPDR & ~(0x3u << pos)) | ((uint32_t)pull << pos);
    g.port->MODER = (g.port->MODER & ~(0x3u << pos)) | (0x0u << pos);
}

void bm_gpio_deinit_to_analog(bm_gpio_t g)
{
    if (g.pin > 15u) return;
    uint32_t m   = _pin_mask(g.pin);
    uint32_t pos = _pos2(g.pin);

    // No pull
    g.port->PUPDR &= ~(0x3u << pos);
    // Push-pull (doesn't matter in analog, keep consistent)
    g.port->OTYPER &= ~m;
    // Mode = analog (11)
    g.port->MODER  = (g.port->MODER & ~(0x3u << pos)) | (0x3u << pos);
}

/* --- Pin I/O --------------------------------------------------------------- */
void bm_gpio_write(bm_gpio_t g, bool high)
{
    if (g.pin > 15u) return;
    uint32_t m = _pin_mask(g.pin);
    g.port->BSRR = high ? m : (m << 16);
}

void bm_gpio_set(bm_gpio_t g)
{
    if (g.pin > 15u) return;
    g.port->BSRR = _pin_mask(g.pin);
}

void bm_gpio_clear(bm_gpio_t g)
{
    if (g.pin > 15u) return;
    g.port->BSRR = _pin_mask(g.pin) << 16;
}

void bm_gpio_toggle(bm_gpio_t g)
{
    if (g.pin > 15u) return;
    uint32_t m = _pin_mask(g.pin);
    if (g.port->ODR & m) g.port->BSRR = m << 16; else g.port->BSRR = m;
}

bool bm_gpio_read(bm_gpio_t g)
{
    if (g.pin > 15u) return false;
    return (g.port->IDR & _pin_mask(g.pin)) != 0u;
}

/* --- ULP helper ------------------------------------------------------------ */
void bm_gpio_port_all_analog_except(GPIO_TypeDef *port, uint16_t keep_mask)
{
    for (uint8_t p = 0; p < 16; ++p) {
        if (keep_mask & (1u << p)) continue;
        bm_gpio_deinit_to_analog((bm_gpio_t){ port, p });
    }
}
