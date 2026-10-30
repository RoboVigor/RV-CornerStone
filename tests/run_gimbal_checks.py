#!/usr/bin/env python3
"""Reproducible offline checks. Never opens a port or executes target firmware.

Uses temporary binaries only. --arm-syntax checks the *new adapter boundary*
against real vendor headers, NOT the legacy ARMCC project or a linked image.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONPATH=str(ROOT / "tools/gimbal_tuning"))


def run(args):
    print("+ " + " ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=ROOT, env=ENV, check=True)


def project_check():
    root = ET.parse(ROOT / "mdk/infantry.uvprojx").getroot()
    assert root.find(".//uAC6").text == "0"
    assert root.find(".//Device").text == "STM32F427II"
    assert root.find(".//uC99").text == "1"
    paths = [(ROOT / "mdk" / p.text.replace("\\", "/")).resolve() for p in root.findall(".//FilePath")]
    for relative in ("driver/src/Driver_GimbalTuning.c", "driver/inc/Driver_GimbalTuning.h",
                     "user/infantry/gimbal_tuning.c", "user/infantry/gimbal_tuning.h"):
        path = ROOT / relative
        assert path.exists() and paths.count(path) == 1, relative
    print("PASS Keil XML entries/device/C99 (static only; not a firmware build)", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sanitize", action="store_true", help="ASan/UBSan for native C tests")
    parser.add_argument("--arm-syntax", action="store_true", help="optional ARM GCC vendor-header adapter boundary check")
    parser.add_argument("--no-python", action="store_true", help="only C/project checks")
    args = parser.parse_args()
    project_check()
    cc = shutil.which("cc")
    with tempfile.TemporaryDirectory(prefix="gimbal-checks-") as temp:
        if cc:
            common = [cc, "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-g", "-Idriver/inc"]
            if args.sanitize:
                common += ["-fsanitize=address,undefined"]
            core = Path(temp) / "core-test"
            # The actual pre-existing PID_Print uses %d for a promoted float.
            # Leave unrelated code unchanged; suppress its warning in this comparison test.
            run(common + ["-Wno-format", "-Itests/stubs", "driver/src/Driver_GimbalTuning.c",
                          "driver/src/Driver_PID.c", "tests/gimbal_core_test.c", "-lm", "-o", core])
            run([core])
            adapter = Path(temp) / "adapter-test"
            run(common + ["-Wno-pointer-to-int-cast", "-Iuser/infantry", "driver/src/Driver_GimbalTuning.c",
                          "tests/gimbal_adapter_test.c", "-lm", "-o", adapter])
            run([adapter])
        else:
            print("NOT RUN native C tests: cc unavailable", flush=True)
        if args.arm_syntax:
            arm = shutil.which("arm-none-eabi-gcc")
            if not arm:
                print("NOT RUN ARM adapter boundary syntax: arm-none-eabi-gcc unavailable", flush=True)
            else:
                command = [arm, "-std=c99", "-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
                           "-DSTM32F427_437xx", "-DUSE_STDPERIPH_DRIVER", "-D__TARGET_FPU_CM4",
                           "-Icore", "-Iuser/common", "-Idevice/inc", "-Idriver/inc", "-Iuser/infantry",
                           "-Wall", "-Wextra", "-Werror", "-fsyntax-only"]
                run(command + ["driver/src/Driver_GimbalTuning.c"])
                for mode in ("1", "0"):
                    run(command + ["-DGIMBAL_TUNING_ENABLED=" + mode, "-include", "tests/stubs/gimbal_platform.h",
                                   "user/infantry/gimbal_tuning.c"])
                print("PASS new adapter syntax against real vendor/CMSIS declarations (legacy handle graph substituted)", flush=True)
        if not args.no_python:
            run([sys.executable, "-m", "unittest", "discover", "-s", "tools/gimbal_tuning/tests", "-v"])
            run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"])
    print("Offline checks finished. Keil link, USART/CAN timing, VOFA UI and hardware NOT RUN.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
