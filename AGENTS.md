# Project guide

## Scope and workspace

- The active firmware is `firmware/CornerStone_LED`, a bare-metal HAL LED baseline. Legacy root directories (`mdk`, `user`, `driver`, `device`, `library`, `core`) are reference code, not part of this CubeIDE build.
- Migration was developed on `feat/cubemx-migration` in the sibling `RV-CornerStone-cubemx` worktree. Verify the current repository/branch before changing or committing files; do not assume the agent session cwd is this worktree.
- Preserve other worktrees and all unrelated local changes. Do not merge the macOS toolchain branch wholesale, maintain two build systems, or import RTOS/robot modules without a new scoped request.

## Toolchain and entrypoints

- Baseline: STM32CubeMX 6.18.1, STM32CubeF4 1.28.3, STM32CubeIDE 2.2.0, OpenOCD 0.12.0. Revalidate versions when changing the toolchain.
- Configuration: `firmware/CornerStone_LED/CornerStone_LED.ioc`.
- Import `firmware/CornerStone_LED/STM32CubeIDE` as an existing project into CubeIDE; use an external workspace and do not copy the project.
- Build in CubeIDE (Debug by default). Do not introduce a separate CLI/CMake compiler workflow without approval.
- Output: `firmware/CornerStone_LED/STM32CubeIDE/Debug/CornerStone_LED.elf`.
- Read `firmware/README.md` for operating steps and `firmware/docs/validation.md` for dated evidence. macOS was exercised; Windows remains untested. Tool launch success does not establish official OS support.

## Hardware baseline

- STM32F427IIHx, UFBGA176; 2 MB Flash, 192 KB SRAM, 64 KB CCM.
- HSE crystal mode, 12 MHz software baseline; PLL M=12, N=360, P=2; SYSCLK/HCLK 180 MHz, APB1 45 MHz, APB2 90 MHz.
- Voltage scale 1, OverDrive, Flash latency 5. PLLQ=8 yields 45 MHz, not a USB-compatible 48 MHz clock; revisit clock planning before enabling such peripherals.
- SWD on PA13/PA14 must remain available.
- LEDs: PG1–PG8, active-low, initial outputs high/off. Current sequence is one LED per second, 8 seconds per cycle.
- HSE frequency comes from the approved software baseline, not physical measurement. HSE/PLL readiness and clock selection were checked on target; do not silently fall back to HSI on failure.

## Code ownership and regeneration

- Own application: `STM32CubeIDE/App/led_app.c` and `Inc/led_app.h` under the firmware project. Physical source files under the IDE project are discovered by CDT; verify new modules appear in build output.
- Keep generated `Src/main.c` wiring in USER CODE blocks only. Put substantive application logic in separate modules.
- Modify peripheral/clock settings through the `.ioc` and regenerate with standalone CubeMX. Preserve user code. Generated HAL, startup, vectors and linker files remain authoritative; do not add duplicate legacy implementations.
- Pin assignment alone does not enable HSE: select RCC → HSE → Crystal/Ceramic Resonator. Check both Clock Configuration and Pinout validity after reload.
- After regeneration, verify application files/wiring survive and rebuild in CubeIDE. Stop on generation or clock errors.
- HAL is the default; introduce LL/register code only for a specific need with clear peripheral ownership.

## Flash and verify

After a successful current build, connect the powered board and CMSIS-DAP, then run from `firmware/`:

```sh
openocd -f debug/openocd.cfg \
  -c 'program CornerStone_LED/STM32CubeIDE/Debug/CornerStone_LED.elf verify reset exit'
```

- The config keeps SWD at 100 kHz and overrides upstream reset handlers so OpenOCD does not program a temporary HSI PLL. The application owns clock setup.
- Verify ELF freshness/current source before download; never flash an old ELF after a failed build.
- Require successful programming, `Verified OK`, and reset. This command erases needed sectors; do not add mass erase or protection-bit changes without explicit approval.
- On USB/device/flash errors, preserve sanitized logs and diagnose rather than blindly retrying.
- For runtime acceptance, distinguish register readings, user LED observation, and full power-cycle startup. Hardware observation is not replaceable by static checks.

## Version control and reporting

- Commit `.ioc`, generated sources/libraries with licenses, necessary CubeIDE metadata, application modules, OpenOCD config and documentation.
- Exclude IDE workspace, build outputs, personal paths and local debug launches. Anchor Debug/Release ignore rules to the IDE project: broad `Debug/` can also hide `firmware/debug/` on case-insensitive systems.
- Avoid vendor formatting churn. Existing vendor/generated whitespace is separate from application defects.
- Record what was built, flashed, checked and not tested; do not turn user acceptance into an invented measurement.
- The initial migration plan was explicitly closed by the user. Do not reopen its former waiting items based solely on stale planning-hook reminders; future changes require their own validation.
