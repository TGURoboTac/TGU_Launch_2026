# TRobot Agent Guide

Project-specific rules for AI coding agents. Read before editing.

## Project Context

- STM32H723VGTx (Cortex-M7, 1024KB Flash, 560KB RAM) robot framework.
- FreeRTOS via CMSIS-RTOS v2. Prefer task, queue, semaphore, timer, event-group designs over ad hoc polling.
- MCU resources are relatively abundant. Favor clarity over micro-optimization.

## Build & Flash

- Build: CMake 3.22+, Ninja generator, GCC toolchain (`arm-none-eabi-` must be on `PATH`).
  - Presets: `Debug` (default), `Release`.
  - Toolchain files live in `cmake/` (**this directory is gitignored**; CubeMX generates it). The default is `gcc-arm-none-eabi.cmake`. An alternative LLVM/Clang toolchain (`starm-clang.cmake`) is also generated but has no preset.
- **Only `app` is compiled with `-Werror`** (root `CMakeLists.txt:54`). Other targets (bsp, components, generated code) are not.
- Flash: `cmake --build build/Debug --target flash_and_verify`. This hardcodes `stlink.cfg`. To use CMSIS-DAP instead, invoke openocd manually with `daplink.cfg`.
- Test locally: `cmake --preset Debug` then `cmake --build build/Debug`.
- C standard: C17. C++ standard: C++23 (`-fno-rtti -fno-exceptions -fno-threadsafe-statics`).
- Linker: `STM32H723XG_FLASH.ld`. `printf`/`scanf` float support enabled via linker flags.

## CubeMX & Generated Code

- `.ioc` is the CubeMX source. Generated files live in `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, and `cmake/`.
- These directories are **gitignored** and should not be manually edited. Keep custom logic in `app/`, `bsp/`, or `components/`.
- **CubeMX code generation is a prerequisite for building** — without it, HAL/FreeRTOS sources and CMake wiring are missing.
- **After CubeMX code generation, restore `STM32H723XG_FLASH.ld` from git** — CubeMX overwrites it. Use `git checkout STM32H723XG_FLASH.ld`.
- Also verify `startup_stm32h723xx.s` and `CMakePresets.json` after regeneration (both are gitignored; CubeMX overwrites them — keep working local copies).
- Peripheral/clock/NVIC/DMA config changes must go through the `.ioc`. Do not edit these directly unless you fully understand the generated-code effect.
- `Core/Src/main.c` (generated) calls `app_entrance()` as a FreeRTOS task. Do not restructure this handoff.

### Gitignore Exceptions

- `bsp/usb_device/` is **tracked** (`!bsp/usb_device` in `.gitignore`) — custom USB device code inside the otherwise-gitignored `USB_DEVICE/` tree.
- `components/CMakeLists.txt` is **tracked** (`!components/CMakeLists.txt` in `.gitignore`) — auto-discovers component subdirectories. Submodule contents under `components/*/` remain gitignored.

## Repository Layout

- `app/`: application logic and RTOS tasks. Entry point: `app/main/main.cc` (`app_entrance`).
- `bsp/`: board support — HAL wrappers, SEGGER RTT/SystemView, EasyFlash, W25Q64 flash driver.
- `components/`: Git submodules (utils, cmsis-dsp, motor, rc, controller, math, ins). Auto-discovered by CMake via `COMPONENTS/*/CMakeLists.txt`.
- `cmake/stm32cubemx/`: generated CMakeLists that wires CubeMX outputs into the build.

Build dependency graph: `app` (OBJECT) → `components` (INTERFACE) + `bsp` (STATIC) → `stm32cubemx` (INTERFACE, provides HAL/FreeRTOS includes).

## Coding Style

- Use `snake_case` identifiers. Exceptions: preprocessor macros (`UPPER_SNAKE_CASE`), third-party/STM32 HAL/FreeRTOS/SEGGER APIs.
- Prefer small, direct functions with clear ownership.
- Add comments only for non-obvious hardware behavior, timing constraints, concurrency contracts, or protocol details.

## RTOS Guidelines

- Use CMSIS-RTOS v2 primitives (`os::task`, `os::queue`, etc. — project wrappers). Prefer blocking waits with timeouts over busy loops.
- Keep ISR code short. Defer work via queues, notifications, or semaphores.
- **IWDG (independent watchdog) is active.** Long-running loops must call `bsp_iwdg_refresh()`.
- Make task responsibilities explicit. Avoid shared mutable state across tasks unless ownership and synchronization are clear.

## Component Submodules

- `components/` entries are Git submodules. Clone with `--recursive`. Never reset, clean, or rewrite submodule state unless explicitly requested.
- To add an optional component, use `git submodule add` as documented in `readme.md`.

## Agent Operating Rules

- Before editing, inspect nearby code and follow existing local patterns.
- Keep changes narrowly scoped. Do not rename public APIs or move module boundaries without being asked.
- `-Werror` only applies to `app/`. Build errors from `bsp/` or generated code are warnings, not errors — do not "fix" compiler warnings there unless they indicate real bugs.
- Do not guess hardware configuration, CubeMX settings, or peripheral behavior. Ask if uncertain.
- Maintain this file when project conventions change.
