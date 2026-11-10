"""离线编译生产固件和 C 测试。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
SDK = Path("C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.3")
ARM = Path("F:/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/"
           "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin")
UNITY = Path("C:/Program Files/Unity/Hub/Editor/6000.6.2f1/Editor/Data/PlaybackEngines/"
             "WebGLSupport/BuildTools/Emscripten")
CORE = [ROOT / "src" / name for name in ("control.c", "drive.c", "protocol.c")]
COMMON = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(ROOT / "include")]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command, log, env=None):
    result = subprocess.run(list(map(str, command)), cwd=REPO, env=env, capture_output=True,
                            text=True, encoding="utf-8", errors="replace", timeout=180)
    output = result.stdout + result.stderr
    log.write_text(output, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"{command[0]} exited {result.returncode}\n{output}")
    return output


def host_tests(args, out):
    board = (ROOT / "src/board_g474.c").read_text(encoding="utf-8")
    uart_start = board.index("void HAL_UART_RxCpltCallback(")
    uart_end = board.index("void HAL_FDCAN_MspInit(")
    (out / "board_uart.inc").write_text(board[uart_start:uart_end], encoding="utf-8")
    native = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    env = os.environ.copy()
    if native:
        compiler = Path(native)
        flags = []
    else:
        compiler = args.emscripten / "emscripten/emcc.bat"
        if not compiler.is_file():
            raise FileNotFoundError("No native C compiler or configured Emscripten")
        config = out / "emscripten_config.py"
        config.write_text(
            f"LLVM_ROOT={str(args.emscripten / 'llvm')!r}\n"
            f"BINARYEN_ROOT={str(args.emscripten / 'binaryen')!r}\n"
            f"NODE_JS={[str(args.emscripten / 'node/node.exe')]!r}\nFROZEN_CACHE=True\n",
            encoding="utf-8")
        env["EM_CONFIG"] = str(config)
        flags = ["-sENVIRONMENT=node", "-sEXIT_RUNTIME=1", "-sSINGLE_FILE=1"]
    results = {}
    for test in sorted((ROOT / "tests").glob("test_*.c")):
        suffix = ".js" if not native else ".exe" if os.name == "nt" else ""
        binary = out / (test.stem + suffix)
        run([compiler, *COMMON, "-I" + str(out), *CORE, test, "-lm", *flags, "-o", binary],
            out / (test.stem + "_compile.log"), env)
        command = [binary] if native else [args.emscripten / "node/node.exe", binary]
        output = run(command, out / (test.stem + ".log"), env)
        results[test.stem] = [line for line in output.splitlines() if line.startswith("PASS ")]
    if not results:
        raise RuntimeError("No production C tests found")
    return {"compiler": str(compiler), "suites": results}


def firmware(args, out):
    def tool(name):
        executable = name + (".exe" if os.name == "nt" else "")
        path = args.compiler_dir / executable if args.compiler_dir else shutil.which(executable)
        if not path:
            raise FileNotFoundError(executable)
        return Path(path)

    gcc = tool("arm-none-eabi-gcc")
    cmsis = args.sdk / "Drivers/CMSIS"
    device = cmsis / "Device/ST/STM32G4xx"
    hal = args.sdk / "Drivers/STM32G4xx_HAL_Driver"
    modules = ("hal", "hal_cortex", "hal_rcc", "hal_rcc_ex", "hal_gpio", "hal_dma", "hal_dma_ex",
               "hal_flash", "hal_flash_ex", "hal_pwr", "hal_pwr_ex", "hal_uart", "hal_uart_ex",
               "hal_fdcan", "hal_rng")
    external = [device / "Source/Templates/system_stm32g4xx.c",
                device / "Source/Templates/gcc/startup_stm32g474xx.s"]
    external.extend(hal / "Src" / ("stm32g4xx_" + module + ".c") for module in modules)
    flags = [*COMMON, "-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard", "-Os", "-g3", "-flto",
             "-ffunction-sections", "-fdata-sections", "-DSTM32G474xx", "-DUSE_HAL_DRIVER",
             "-DUSER_VECT_TAB_ADDRESS", "-I" + str(hal / "Inc"), "-I" + str(cmsis / "Include"),
             "-I" + str(device / "Include")]
    objects = []
    for source in [ROOT / "src/board_g474.c", *CORE, *external]:
        obj = out / (source.name + ".o")
        run([gcc, *flags, "-c", source, "-o", obj], out / (source.name + ".log"))
        objects.append(obj)
    elf = out / "mcu-v3.elf"
    binary = out / "mcu-v3.bin"
    mapping = out / "mcu-v3.map"
    run([gcc, *flags, "-nostartfiles", "--specs=nano.specs", "--specs=nosys.specs", *objects,
         "-T" + str(ROOT / "flash.ld"), "-Wl,--gc-sections", "-Wl,-Map=" + str(mapping),
         "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group", "-o", elf], out / "link.log")
    run([tool("arm-none-eabi-objcopy"), "-O", "binary", elf, binary], out / "objcopy.log")
    size = run([tool("arm-none-eabi-size"), elf], out / "size.log")
    if binary.stat().st_size > 32768:
        raise RuntimeError("Firmware exceeds the 32 KiB application")
    return {"bytes": binary.stat().st_size, "flash_limit": 32768, "size": size.strip(),
            "bin_sha256": digest(binary), "elf_sha256": digest(elf), "map_sha256": digest(mapping),
            "compiler_sha256": digest(gcc), "flags": flags,
            "sdk_source_sha256": {str(source): digest(source) for source in external}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-tests", action="store_true")
    parser.add_argument("--firmware", action="store_true")
    parser.add_argument("--name", default="mcu-v3-offline")
    parser.add_argument("--sdk", type=Path, default=SDK)
    parser.add_argument("--compiler-dir", type=Path, default=ARM if os.name == "nt" else None)
    parser.add_argument("--emscripten", type=Path, default=UNITY)
    args = parser.parse_args()
    if not (args.host_tests or args.firmware):
        parser.error("choose --host-tests and/or --firmware")
    if (not args.name or args.name in (".", "..") or
        any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-." for c in args.name)):
        parser.error("--name must be a single safe directory name")
    out = REPO / "build" / args.name
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = {"passed": False, "hardware_accessed": False, "flashed": False,
              "source_sha256": {str(path.relative_to(REPO)).replace("\\", "/"): digest(path)
                                for path in sorted(ROOT.rglob("*"))
                                if path.suffix in (".c", ".h", ".py", ".ld")}}
    try:
        if args.host_tests:
            report["tests"] = host_tests(args, out)
        if args.firmware:
            report["firmware"] = firmware(args, out)
        report["passed"] = True
    finally:
        report["elapsed_s"] = time.monotonic() - started
        (out / "receipt.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({k: v for k, v in report.items() if k != "source_sha256"}, indent=2))


if __name__ == "__main__":
    main()
