# STM32F10x Driver Library

> **Learning Documentation**: This library represents my journey learning CMSIS and embedded systems programming. Instead of relying on HAL abstractions, I built these drivers from scratch using register-level access to truly understand how STM32 peripherals work at the hardware level.

A lightweight, register-based driver library for STM32F1xx microcontrollers. This library provides clean, efficient drivers for common peripherals without the overhead of HAL.

## Features

- **Register-level access** for maximum control and minimal overhead
- **Clean API** with consistent naming conventions
- **Well-documented** code with reference to RM0008 (STM32F1 Reference Manual)
- **Third-person documentation** style for clarity
- **Type-safe** enumerations for configuration

## Supported Hardware

- **STM32F1xx** family (STM32F103, STM32F100, etc.)
- Tested on STM32F103C8T6 (Blue Pill)

## Available Drivers

| Driver | Description | Key Features |
|--------|-------------|--------------|
| **GPIO** | General Purpose I/O | Pin configuration, read/write, toggle, pull-up/down |
| **UART** | Serial Communication | Configurable baud rate, blocking send |
| **SysTick** | System Timer | 1ms tick, delay functions, millis() equivalent |
| **EXTI** | External Interrupts | Pin-based interrupts with callbacks |

## Directory Structure

```
Drivers_F10x/
├── Drv/
│   ├── Inc/          # Header files (.h)
│   │   ├── drv_gpio.h
│   │   ├── drv_uart.h
│   │   ├── drv_systick.h
│   │   └── drv_exti.h
│   └── Src/          # Source files (.c)
│       ├── drv_gpio.c
│       ├── drv_uart.c
│       ├── drv_systick.c
│       └── drv_exti.c
└── README.md
```

## Quick Start

### 1. Include Headers

```c
#include "drv_gpio.h"
#include "drv_uart.h"
#include "drv_systick.h"
#include "drv_exti.h"
```

### 2. GPIO Example

```c
// Initialize LED pin (PC13) as output
drv_gpio_init(GPIOC, 13, GPIO_MODE_OUTPUT_PP_2MHZ);

// Turn LED on
drv_gpio_set(GPIOC, 13);

// Turn LED off
drv_gpio_clear(GPIOC, 13);

// Toggle LED
drv_gpio_toggle(GPIOC, 13);

// Read button state (PA0)
drv_gpio_init(GPIOA, 0, GPIO_MODE_INPUT_PULL_UP);
uint8_t state = drv_gpio_read(GPIOA, 0);
```

### 3. UART Example

```c
// Initialize UART1 at 115200 baud
drv_uart_init(USART1, 115200);

// Send a string
drv_uart_send_string(USART1, "Hello, World!\r\n");

// Send a single character
drv_uart_send_char(USART1, 'A');
```

### 4. SysTick Example

```c
// Initialize SysTick for 1ms ticks
drv_systick_init();

// Non-blocking delay using millis()
uint32_t start = drv_systick_get_ticks();
while ((drv_systick_get_ticks() - start) < 1000) {
    // Do something for 1 second
}

// Blocking delay (not recommended)
drv_systick_delay(500); // 500ms delay
```

### 5. External Interrupt Example

```c
// Callback function
void button_pressed(void) {
    drv_gpio_toggle(GPIOC, 13); // Toggle LED
}

// Initialize button on PA0 with falling edge trigger
drv_exti_init(GPIOA, 0, EXTI_TRIGGER_FALLING, button_pressed);
```

## Integration into Your Project

### Prerequisites

- CMSIS headers (`stm32f1xx.h`)
- Properly configured system clock (`SystemCoreClock`)
- Interrupt handlers in `stm32f1xx_it.c`

### Required Interrupt Handlers

Add these to your `stm32f1xx_it.c`:

```c
// SysTick Handler
void SysTick_Handler(void) {
    drv_systick_tick_handler();
}

// EXTI Handlers (example for pin 0)
void EXTI0_IRQHandler(void) {
    drv_exti_irq_handler(0);
}

// Add other EXTI handlers as needed (EXTI1_IRQHandler, etc.)
```

### Linker Settings

Ensure the `Drv/Src` directory is included in your build system and all `.c` files are compiled.

## GPIO Modes Reference

| Mode | Description | Use Case |
|------|-------------|----------|
| `GPIO_MODE_INPUT_ANALOG` | Analog input | ADC pins |
| `GPIO_MODE_INPUT_FLOATING` | Floating input | UART RX, default |
| `GPIO_MODE_INPUT_PULL_UP` | Input with pull-up | Buttons (active low) |
| `GPIO_MODE_INPUT_PULL_DOWN` | Input with pull-down | Buttons (active high) |
| `GPIO_MODE_OUTPUT_PP_10MHZ` | Push-pull output, 10MHz | General purpose output |
| `GPIO_MODE_OUTPUT_PP_2MHZ` | Push-pull output, 2MHz | Low-speed output |
| `GPIO_MODE_OUTPUT_PP_50MHZ` | Push-pull output, 50MHz | High-speed output |
| `GPIO_MODE_OUTPUT_OD_10MHZ` | Open-drain output, 10MHz | I2C, custom protocols |
| `GPIO_MODE_AF_PP_50MHZ` | Alternate function push-pull | UART TX, SPI |
| `GPIO_MODE_AF_OD_50MHZ` | Alternate function open-drain | I2C with AF |

## UART Pin Mapping

| UART | TX Pin | RX Pin | Clock Domain |
|------|--------|--------|--------------|
| USART1 | PA9 | PA10 | APB2 (72MHz) |
| USART2 | PA2 | PA3 | APB1 (36MHz) |

## Best Practices

1. **Always initialize clocks** before using peripherals (drivers handle this automatically)
2. **Use non-blocking delays** with `drv_systick_get_ticks()` instead of `drv_systick_delay()`
3. **Keep interrupt handlers short** - use callbacks for complex logic
4. **Enable interrupts globally** with `__enable_irq()` after initialization
5. **Check return values** where applicable

## Dependencies

- **CMSIS Core**: For register definitions and NVIC functions
- **System Clock**: `SystemCoreClock` must be correctly configured
- **Compiler**: ARM GCC or Keil MDK

## Reference Documentation

All drivers reference the **STM32F1 Reference Manual (RM0008 Rev-21)**. Register operations include comments with section references for easy lookup.

## License

This library is provided as-is for educational and commercial use.

## Contributing

When contributing:
- Follow the existing code style
- Use third-person perspective in comments
- Include RM0008 references for register operations
- Test on real hardware before submitting

## Author

Pavel Pys

---

**Note**: This is a register-level driver library. For more complex applications or if you need USB, CAN, or advanced DMA features, consider using STM32 HAL or LL libraries.
