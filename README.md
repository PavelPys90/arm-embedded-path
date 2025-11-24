# ARM Embedded Path Portfolio

Welcome to my embedded systems portfolio! This repository documents my journey into the depths of ARM Cortex-M development, specifically focusing on the **STM32F10x** family.

Here you will find everything from bare-metal register manipulations to custom driver implementations and application-level projects.

## Repository Structure

The repository is organized to separate active development from foundational learning:

| Directory | Description |
|-----------|-------------|
| **`10_Projects/`** | Active application-level projects (e.g., `ClimaTracker`). |
| **`99_Libs_Dev/`** | Custom libraries, drivers, and development tools. |
| **`_archive/`** | Foundational studies, CMSIS basics, and early experiments. |
| **`docs/`** | Documentation and datasheets. |

---

## Key Highlights

### [Drivers_F10x](99_Libs_Dev/Drivers_F10x/README.md)
A custom, lightweight **register-level driver library** for STM32F10x.
- **Why?** To truly understand the hardware, I bypassed the standard HAL and built these drivers from scratch using the Reference Manual (RM0008).
- **Features**: GPIO, UART, SysTick, EXTI, ADC, PWM.
- **Philosophy**: Zero overhead, type-safe configuration, and readable code.

### ClimaTracker (Work in Progress)
*Located in `10_Projects/01_ClimaTracker`*
An environmental monitoring application currently under active development.
- **Current State**: **Initial Setup Phase**. The project is currently in the architectural setup stage (System Clock, I2C, GPIO configuration) and is **not yet fully functional**.
- **Roadmap**: Implementation of sensor drivers and data processing logic.

---

## Technical Skills Demonstrated

- **Embedded C**: Writing efficient, hardware-oriented C code.
- **STM32 Architecture**: Deep understanding of the Cortex-M3 core, NVIC, RCC, and peripherals.
- **Bare Metal Programming**: Direct register access without relying on heavy abstraction layers.
- **Driver Development**: API design and implementation for hardware peripherals.
- **Debugging**: Using SWD and logic analyzers to validate timing and protocols.
