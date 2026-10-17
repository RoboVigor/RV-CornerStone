# CornerStone_Robot

Independent starting point for incremental migration of the original Keil robot firmware.

- Configuration: `CornerStone_Robot.ioc` (standalone CubeMX).
- Import `STM32CubeIDE/` as an existing CubeIDE project into an external workspace, without copying.
- Build project `CornerStone_Robot`; expected output: `STM32CubeIDE/Debug/CornerStone_Robot.elf`.
- Shared launch entries: `CornerStone_Robot CMSIS-DAP Flash and Debug` and `CornerStone_Robot Flash and Run`.
- Initial application is still the PG1–PG8 running LED test, 1000 ms per step. No RTOS or robot modules have been imported.
- This project owns independent copies of its sources and vendor libraries; it does not link into `CornerStone_LED`.
- Keep `../CornerStone_LED/` unchanged as the simple test firmware. The two projects produce alternative images for the same board.

See [firmware operating guide](../README.md) for prerequisites, regeneration ownership and flashing safeguards. Debugger setup applies with Robot project/launch/ELF names substituted.

## Validation boundary

Created from the tracked LED baseline without build outputs or personal IDE state. Project naming, XML, linked-source resolution and copy equivalence are checked statically. CubeIDE build, CubeMX reload/regeneration and board execution of this new project have not yet been performed; the LED project's historical acceptance does not constitute Robot acceptance.
