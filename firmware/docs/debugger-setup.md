# STM32CubeIDE 调试配置：Windows / macOS

适用项目：`CornerStone_LED`，使用 CMSIS-DAP + OpenOCD 下载并调试 STM32。

推荐做法：**复制共享配置为本机配置，在 Connection 中填写 OpenOCD 完整路径**。不需要配置 PATH，也不需要 Eclipse 的 `openocd_path` 变量。

> 验证边界：macOS 上此前的完整路径配置已获用户运行确认；Windows 配置是操作参考，尚未实机验证。Windows 所带 GDB 的 pipe 支持、路径引用和下载调试行为需要本机验收。参考 IDE 版本为 STM32CubeIDE 2.2.0，OpenOCD 为 0.12.0。

## 1. 准备

- 按 [README](../README.md#import-and-build-in-cubeide) 导入项目并成功构建 Debug。
- 安装包含 CMSIS-DAP 支持的 OpenOCD，保留其配套 `scripts` 目录，不要只复制可执行文件。
- 连接 CMSIS-DAP 的 USB 数据线、目标板电源及 SWD。关闭占用探针的其他调试器；检查探针是否被虚拟机接管。
- STM32CubeIDE 自带 ARM GDB，不需要另外安装主机 GDB。

找到 OpenOCD 可执行文件：

| 平台 | 示例路径（按实际安装修改） |
|---|---|
| macOS Apple Silicon / Homebrew | `/opt/homebrew/bin/openocd` |
| macOS Intel / Homebrew | `/usr/local/bin/openocd` |
| Windows | `C:/Tools/OpenOCD/bin/openocd.exe` |

macOS 可在终端用 `command -v openocd` 查路径；Windows 可在安装目录查找 `openocd.exe`。下文 Windows 示例使用正斜杠，减少反斜杠在不同命令解析层的歧义。

## 2. 创建本机调试入口

1. 刷新 CubeIDE 中的 `CornerStone_LED` 项目。
2. 打开 **Run → Debug Configurations…**。
3. 展开 **GDB Hardware Debugging**，选择 `CornerStone_LED CMSIS-DAP Flash and Debug`。
4. 复制该配置，命名为 `CornerStone_LED CMSIS-DAP Local`。
5. 在 **Common → Save as** 选择 **Local file**，保存在工作区，不把个人绝对路径保存到仓库的共享 `.launch` 文件。

找不到共享入口时：使用 **File → Import → Run/Debug → Launch Configurations**，选择 `firmware/CornerStone_LED/STM32CubeIDE`。不要选 **STM32 C/C++ Application → CornerStone_LED Debug**，那可能是 ST-LINK 配置。

## 3. Main：项目与固件

复制后通常无需修改：

| 项目 | 值 |
|---|---|
| Project | `CornerStone_LED` |
| C/C++ Application | `Debug/CornerStone_LED.elf` |
| Build Configuration | `Debug` |
| Build before launch | Enable auto build |

共享配置的工作目录为 `${workspace_loc:/CornerStone_LED}`，即 `firmware/CornerStone_LED/STM32CubeIDE`。保留它；下文的相对路径依赖这个目录。

在 Preferences 的 **Run/Debug → Launching** 中，将 **Continue launch if project contains errors** 设为 **Never**。编译失败后不要下载旧 ELF。

## 4. Debugger：连接 CMSIS-DAP

- GDB Command：保留 `${gnu_tools_for_stm32_compiler_path}/arm-none-eabi-gdb`；Windows 若无法解析可执行文件，可通过 Browse 选择 CubeIDE 工具链中的 `arm-none-eabi-gdb.exe`。
- **Use remote target：勾选**。
- JTAG Device：选择 **OpenOCD (via pipe)**（具体显示名称可能略有差异）。
- Protocol：`remote`。
- Connection：按平台填写以下内容，替换实际安装路径。

### macOS

```text
| "/opt/homebrew/bin/openocd" -f "../../debug/openocd.cfg" -c "gdb_port pipe; tcl_port disabled; telnet_port disabled"
```

### Windows（待实机验证）

```text
| "C:/Tools/OpenOCD/bin/openocd.exe" -f "../../debug/openocd.cfg" -c "gdb_port pipe; tcl_port disabled; telnet_port disabled"
```

路径含空格时保留双引号。Connection 在当前 CDT 插件中不会展开 Eclipse `${...}` 变量，因此这里直接填写完整路径。

**不要取消 Use remote target**：它还控制 IDE 的 remote 会话类型。使用 USB/SWD 调试外部 MCU 同样属于 remote，并不一定是网络连接。

## 5. Startup：下载与停在 main

Initialization Commands：

```text
set remotetimeout 30
monitor reset halt
```

- 保留独立的 **Reset and Delay**、**Halt** 复选框未勾选，避免重复复位。
- 勾选 **Load image**、**Load symbols**，均选择 **Use project binary**。
- 不设置额外 image/symbol offset，也不手动设置 PC。
- 勾选 **Set breakpoint at**，值为 `main`。
- 勾选 **Resume**。

Run Commands（在下载后校验并复位）：

```text
monitor verify_image "Debug/CornerStone_LED.elf"
monitor reset halt
```

这里使用相对路径，同样依赖上面的项目工作目录。不要在 Startup 中再写 `target remote`，Connection 已经负责连接。

## 6. 使用与排错

点击 **Apply → Debug**，预期流程：

```text
构建 → GDB 启动 OpenOCD → 连接探针/MCU → 下载 → 校验 → 复位 → 停在 main
```

停在 `main` 是正常行为，此时初始化还没执行。按 **F8 / Resume** 继续运行；Mac 键盘可能需要 Fn+F8。当前 SWD 为 100 kHz，下载较慢，不要因此随意修改速率。

| 现象 | 检查方向 |
|---|---|
| 只有 Build finished | 仅代表构建完成；打开 Debug 视图确认 GDB 会话状态 |
| 找不到 openocd / 无法创建进程 | 检查 Connection 中可执行文件路径、引号和运行权限 |
| `unable to find a matching CMSIS-DAP device` | USB 数据线、探针连接、虚拟机占用或其他调试器占用；Windows 还需检查设备驱动识别 |
| SWD / target 通信错误 | 目标板电源、SWD 接线与共地 |
| 找不到 `.cfg` | 检查项目工作目录、仓库目录结构和 OpenOCD 配套 scripts 安装 |
| 找不到 ELF | 确认 Debug 构建成功，输出路径正确 |
| 没有独立 OpenOCD Console | pipe 方案的正常结构；查看可用的 GDB Console 和 Debug 视图 |

打开 Debug 视图：**Window → Show View → Other… → Debug → Debug**。退出码 0 本身不代表下载成功。IDE 异常可进一步查看 **General → Error Log**。

若需要单独检查设备连接，先 Terminate IDE 调试会话，在仓库根目录执行：

macOS 终端：

```sh
"/opt/homebrew/bin/openocd" -f firmware/debug/openocd.cfg -c "init; shutdown"
```

Windows PowerShell：

```powershell
& "C:/Tools/OpenOCD/bin/openocd.exe" -f firmware/debug/openocd.cfg -c "init; shutdown"
```

这会连接设备，但不执行下载命令。失败时保留第一条错误，不要盲目反复烧录或更改保护位。

调试结束用 **Terminate**；若要脱离 IDE 独立运行，请给目标板重新上电，不要假设终止一个暂停中的会话会自动恢复 MCU。

## 7. 只下载并运行：Flash and Run

仓库另外提供 `CornerStone_LED Flash and Run.launch`。它不启动 GDB、不设置断点，执行：

```text
下载 Debug ELF → 校验 → reset 运行 → 退出 OpenOCD
```

### 本机设置（推荐，与 Debug 的本机配置方式一致）

1. 刷新项目，打开 **Run → External Tools → External Tools Configurations…**。
2. 在 **Program** 下选择 `CornerStone_LED Flash and Run`。未出现时，按前述 Launch Configurations 导入步骤导入。
3. 复制为 `CornerStone_LED Flash and Run Local`，在 **Common → Save as → Local file** 保存到工作区。
4. 在 **Main → Location** 用 **Browse File System** 选择 OpenOCD 可执行文件；路径不加外围引号：
   - macOS 示例：`/opt/homebrew/bin/openocd`
   - Windows 示例：`C:/Tools/OpenOCD/bin/openocd.exe`
5. 保留 Working Directory：`${workspace_loc:/CornerStone_LED}`。
6. 保留 Arguments：

```text
-f "../../debug/openocd.cfg" -c "program Debug/CornerStone_LED.elf verify reset exit"
```

与 Debug 的 Connection 不同，这里的 Location 是单独的文件路径字段：**不填写 `|`，不在 Location 后附加参数，也不要只填 `openocd`**。当前 External Tools 会检查该文件是否存在，不会自动从 PATH 搜索。

共享模板的 Location 是 `${openocd_path}`，因此也可以不复制配置，而在 **Preferences → Run/Debug → String Substitution** 添加 `openocd_path`，值为本机完整路径（无外围引号）。这是可选方式；如果已复制并填入实际路径，就无需该变量。个人路径不要保存回共享 `.launch`。

### 每次下载

1. 保存源文件，选中 `CornerStone_LED`，确保 **Build Configurations → Set Active → Debug**。
2. **Project → Build Project**，等待成功结束，确认无编译错误且 `Debug/CornerStone_LED.elf` 是当前构建输出。失败或不确定时不要继续。
3. Terminate 仍在使用 CMSIS-DAP 的 Debug/OpenOCD 会话，确保板子上电。
4. 在 External Tools 中运行 `CornerStone_LED Flash and Run Local`，或者已设置变量的共享入口。
5. 在该工具的 Console 中确认写入成功、`Verified OK`、复位完成和进程正常退出。固件随后自主运行，不会停在 `main`。

**构建安全边界：**模板的 Build 步骤明确禁用，只下载已有 Debug ELF。当前 Program launch delegate 不提供可靠的编译错误拦截；不要将勾选 Build 或全局“遇错误不继续”当作防止下载旧固件的保证。手动成功构建是必要前置条件，本工具不会证明 ELF 新鲜度，也不会自动选择 Release。

可在 **Common → Display in favorites menu → External Tools** 中加入收藏（共享入口已设置），之后从 External Tools 工具栏下拉菜单启动。它不是普通绿色 Run 按钮的默认程序。

验收边界：共享配置和命令可静态检查；首次 IDE 点击下载、复位后运行，以及 Windows/macOS 本机路径配置需在实际环境验收。该入口使用原有 100 kHz OpenOCD 配置，不增加 mass erase 或保护位操作。
