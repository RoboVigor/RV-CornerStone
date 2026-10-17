# Project guide

## Goal and scope

- Gradually migrate the original Keil robot firmware to CubeMX + CubeIDE + HAL. The LED baseline is complete and closed by user acceptance; robot modules and RTOS are not yet migrated.
- Keep `firmware/CornerStone_LED` as the independent simple LED test firmware. Future robot refactoring belongs in `firmware/CornerStone_Robot`, an independent copy initially retaining the same LED behavior. Neither project builds the other; do not propagate robot changes into the LED baseline. RTOS import or robot-module migration still requires an explicitly scoped task; do not scaffold unused layers.
- Legacy root directories (`mdk`, `user`, `driver`, `device`, `library`, `core`) are reference only, outside the CubeIDE build. Do not delete them, merge the macOS toolchain branch wholesale, or maintain a second build system without approval.
- Verify repository/branch before edits or commits: migration work lives in the `RV-CornerStone-cubemx` worktree on `feat/cubemx-migration`. Preserve other worktrees and unrelated changes.

## Read on demand

Read only documents relevant to the task; do not preload every reference.

| Task | Required reference |
| --- | --- |
| Plan the next phase, establish migration scope, or update progress | [Milestones](docs/milestones.md); historical pending items do not reopen closed phases |
| Build, regenerate, flash, or change toolchain | [Firmware operating guide](firmware/README.md); versions, parameters, paths and commands live there |
| Configure CubeIDE debugging or Flash and Run | [Debugger setup](firmware/docs/debugger-setup.md) and the operating guide's validation boundaries |
| Establish what was actually tested | [Validation record](firmware/docs/validation.md); distinguish image revisions and user acceptance from measurements |
| Change pins, clocks, device drivers or electrical assumptions | [Hardware index](docs/hardware/README.md), then relevant board facts/pin map and their unresolved conflicts |
| Determine register behavior or peripheral constraints | [STM32F427 reference index](docs/hardware/stm32f427/README.md), then bounded relevant manual pages; use datasheets/errata or official ST sources as needed |

## Code ownership and migration

- Configuration sources: `firmware/CornerStone_LED/CornerStone_LED.ioc` for the LED test and `firmware/CornerStone_Robot/CornerStone_Robot.ioc` for robot development. Change peripheral/clock settings in the selected project's `.ioc` and regenerate with standalone CubeMX, preserving user code.
- Import the selected firmware's `STM32CubeIDE` directory into an external CubeIDE workspace. Both projects may coexist with distinct names; do not copy into the workspace or introduce a CLI/CMake compiler workflow without approval.
- Application-owned files currently include `STM32CubeIDE/App/led_app.c` and `Inc/led_app.h` under the firmware project. Keep substantive logic in separate modules and generated `Src/main.c` wiring in USER CODE blocks only.
- Physical sources inside the IDE project are discovered by CDT; verify added modules appear in build output. IDE linked resources are not duplicate source copies.
- Generated HAL, startup, vectors and linker files remain authoritative; do not import duplicate legacy implementations. HAL is the default; LL/register access requires a specific need and clear peripheral ownership.
- Migrate one independently verifiable function at a time. Reuse suitable algorithms/protocol logic, adapt hardware dependencies, and separate application/device/board interfaces as needed rather than copying the old framework wholesale.
- After regeneration, check Clock Configuration and Pinout validity, confirm application files/wiring survive, and rebuild in CubeIDE. Stop on generation or clock errors.

## Hardware and flashing safeguards

- Keep SWD on PA13/PA14 available. Do not silently fall back to HSI on HSE/PLL failure. HSE pin assignment alone does not enable the crystal: select RCC HSE Crystal/Ceramic Resonator.
- Revisit clock planning before enabling peripherals requiring exactly 48 MHz; the current PLLQ output is not USB-compatible.
- Keep board facts, `.ioc` configuration and runtime evidence separate. Resolve relevant source conflicts before choosing driver parts, addresses or power nets; never substitute old code/model memory or rewrite facts to match code.
- For missing facts or circuit reasoning, inspect original pages: text first, render relevant diagrams, cite PDF page sequence and Sheet/component. When originals or related configuration change, recheck affected summaries and hardware hashes. Third-party PDFs retain their own rights.
- Use the documented OpenOCD config: retain safe SWD speed and reset overrides that prevent a temporary HSI PLL. The application owns clock setup.
- Before any download, establish a successful current build and fresh ELF. Never flash an old binary after a failed build; a launch entry alone does not establish freshness.
- Require successful programming, `Verified OK`, and reset for flash acceptance. No mass erase or protection-bit changes without explicit approval. On USB/device/flash failure, preserve sanitized evidence and diagnose rather than blindly retrying.

## Validation and delivery

- Distinguish compilation, programming, register checks, physical behavior and full power-cycle startup. Static checks cannot replace hardware acceptance; user closure does not invent an agent measurement. Tool launch success is not official OS support.
- Commit configuration, generated sources/licenses, required IDE metadata, application modules, shared portable debug configuration and documentation. Exclude workspaces, outputs, personal paths and local launches; anchor Debug/Release ignore rules to the IDE project so `firmware/debug/` stays visible on case-insensitive systems.
- Avoid unrelated vendor/generated formatting churn. Report what passed, failed, was not tested and remains unresolved.
- Keep phase status and evidence links in `docs/milestones.md`, detailed operating steps in `firmware/README.md`, and dated verification in `firmware/docs/validation.md`. Task-specific execution plans/logs belong in their own planning files, not in the milestone summary.
