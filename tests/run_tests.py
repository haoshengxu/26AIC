"""Compile the production controller sources and run them as ARM instructions.

Requires Keil ARMCC 5 and Python unicorn==2.1.4. No board connection is used.
Usage: python tests/run_tests.py [--keil D:/KEIL]
"""
import argparse
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(tempfile.gettempdir()) / "ackermann-verification-deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR, UC_HOOK_MEM_INVALID
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keil", default="D:/KEIL")
    parser.add_argument("--suite", choices=("control", "oled", "vofa", "tracking", "vision", "route"), default="control")
    args = parser.parse_args()
    name = "test_" + args.suite
    expected_cases = {"control":14,"oled":3,"vofa":10,"tracking":8,"vision":30,"route":18}[args.suite]
    compiler = Path(args.keil) / "ARM/ARMCC/bin"
    build = ROOT / "tests/build"
    build.mkdir(exist_ok=True)
    commands = [
        [str(compiler / "armcc.exe"), "--cpu=Cortex-M4", "--fpu=SoftVFP",
         "--c99", "-O1", "-g", "-Itests/shims", "-IBALANCE", "-IHARDWARE",
         "-ICore/Inc", "-ISYSTEM/sys", "-ISYSTEM/delay", "-c", f"tests/{name}.c",
         "-o", f"tests/build/{name}.o"],
        [str(compiler / "armlink.exe"), "--cpu=Cortex-M4", "--fpu=SoftVFP",
         "--ro_base=0x10000", "--rw_base=0x20000000", "--entry=test_entry",
         f"tests/build/{name}.o", "-o", f"tests/build/{name}.axf"],
    ]
    logs = []
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True)
        output = (result.stdout + result.stderr).decode("utf-8", errors="replace")
        logs.append(output)
        if result.returncode:
            print(output)
            raise SystemExit(result.returncode)
    elf = (build / f"{name}.axf").read_bytes()
    assert elf[:6] == b"\x7fELF\x01\x01", "Expected little-endian ELF32"
    entry, phoff = struct.unpack_from("<II", elf, 24)
    phentsize, phnum = struct.unpack_from("<HH", elf, 42)
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x10000, 0x100000)
    emu.mem_map(0x20000000, 0x40000)
    for index in range(phnum):
        kind, offset, address, _, size, _, _, _ = struct.unpack_from("<8I", elf, phoff + index * phentsize)
        if kind == 1 and size:
            emu.mem_write(address, elf[offset:offset + size])
    emu.reg_write(UC_ARM_REG_SP, 0x2003FFF0)
    emu.hook_add(UC_HOOK_INTR, lambda uc, number, data: uc.emu_stop())
    def memory_error(uc, access, address, size, value, data):
        print(f"Invalid memory access: address={address:#x} PC={uc.reg_read(UC_ARM_REG_PC):#x}")
        return False
    emu.hook_add(UC_HOOK_MEM_INVALID, memory_error)
    emu.emu_start(entry | 1, 0, timeout=30_000_000, count=100_000_000)
    count, failure, case_id, signature = struct.unpack("<4I", emu.mem_read(0x2001FFF0, 16))
    passed = signature == 0xACCE5501 and failure == 0 and case_id == expected_cases
    text = f"{'PASS' if passed else 'FAIL'}: {args.suite}; {count} assertions; case {case_id}/{expected_cases}; failure line {failure}; signature {signature:#x}\n"
    if not passed and failure:
        text += (ROOT / f"tests/{name}.c").read_text().splitlines()[failure - 1] + "\n"
    print(text, end="")
    report = "test_results.txt" if args.suite == "control" else args.suite + "_test_results.txt"
    (ROOT / "tests" / report).write_text("".join(logs) + text, encoding="utf-8")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
