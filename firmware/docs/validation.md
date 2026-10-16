# Mac validation — 2026-10-16

## Build and regeneration

User supplied CubeIDE Debug console output at 17:51:33–17:51:34, following agent regeneration: `Build Finished. 0 errors, 0 warnings.` Includes `../App/led_app.c`, generated main/HAL/system/startup, and FLASH linker script. Size: text 6232, data 12, bss 1572 bytes. Regeneration previously preserved SHA256 of main.c and both application files.

Flashed ELF SHA256: `fcbb3fb0539ac8de349c6b74d07ac811549346ff7919d4e65e227b5a45ba26a6`.
ELF32 ARM hard-float, entry 0x0800058d; load image starts at 0x08000000 with initialized data load address 0x08001858. No old branch ELF used.

## OpenOCD

OpenOCD 0.12.0 + CMSIS-DAP, SWD 100 kHz. Preflight device ID 0x20036419, Cortex-M4, 2048 KiB flash. This identifies the device family/capacity, not the package (package remains user-confirmed).

Command from firmware/: `openocd -f debug/openocd.cfg -c 'program CornerStone_LED/STM32CubeIDE/Debug/CornerStone_LED.elf verify reset exit'`.
Exit 0; Programming Finished; Verified OK; Resetting Target. No mass erase or protection changes requested.

## Runtime registers (without reset/halt)

Read via OpenOCD `read_memory` after successful reset:

| Register | Value | Interpretation |
|---|---|---|
| RCC_CR | 0x03036083 | HSE ready and PLL ready set |
| RCC_PLLCFGR | 0x08405a0c | HSE source, M=12, N=360, P=2, Q=8 |
| RCC_CFGR | 0x0000940a | PLL SYSCLK, AHB /1, APB1 /4, APB2 /2 |
| PWR_CR | 0x0003c000 | Scale1, OverDrive enable and switching enable |
| PWR_CSR | 0x00034000 | Voltage scale and OverDrive ready bits set |
| FLASH_ACR | 0x00000705 | 5 wait states, prefetch/instruction/data cache enabled |
| GPIOG_ODR sample A | 0x01fa | PG2 low, other LED pins high |
| GPIOG_ODR sample B (+600 ms) | 0x01f6 | PG3 low, other LED pins high |

With the configured 12 MHz HSE baseline this yields 180 MHz SYSCLK. This is register confirmation, not a physical oscillator-frequency measurement. The initial `mdw` commands did not print values in batch output; explicit `echo [read_memory ...]` produced the recorded values.

## Still pending

User physical confirmation of PG1→PG8 cycling at about 500 ms/step for at least two complete cycles and independent restart after removing all power (including potential debugger back-power) and restoring board power. Windows build/hardware not tested.

## 1-second image flashed
- User reported rebuild complete and requested download plus instructions. ELF disassembly shows mov.w r0,#1000 directly before HAL_Delay. SHA256: 47474a9e612f1540561455832a264c25362373fcd84826bd5aeb3add6462168c. No detailed new compiler console supplied; binary inspected directly.
- OpenOCD program verify reset exit succeeded (exit0, Verified OK). New image is now on target. Sanitized log: firmware/docs/openocd-flash-1s.txt.
- Waiting for physical 1s-step/two-cycle (~16s) and full power-cycle confirmation; no new runtime-frequency measurement claimed.

## Delivery checkpoint
- User replied “Good job, proceed” after the 1-second flash. Recorded as approval to proceed with delivery; full removal-of-power restart remains unconfirmed.
- All 1442 files under Drivers match the installed STM32CubeF4 1.28.3 package byte-for-byte before Git line-ending normalization.
- Full staged whitespace check reports inherited vendor/generated whitespace; these files are retained as generated. Application, documentation and OpenOCD files pass the scoped whitespace check after trimming log line endings.
- Build output directories are ignored by anchored paths, avoiding a case-insensitive Debug/debug rule that would accidentally ignore firmware/debug/openocd.cfg.
