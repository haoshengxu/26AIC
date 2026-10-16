"""Check the linked vectors and execute RTOS integration code from the real AXF.

Requires a fresh Keil build and unicorn 2.1.4 (same as run_tests.py).
OS/peripheral callees are intercepted: this checks application wiring, not
Cortex-M exception context switching, timing, or real task stack usage.
"""
from pathlib import Path
import struct
import sys
import tempfile

sys.path.insert(0, str(Path(tempfile.gettempdir()) / 'ackermann-verification-deps'))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3

ROOT = Path(__file__).resolve().parents[1]
elf = (ROOT / 'MDK-ARM/ACKERMANN_YAW/ACKERMANN_YAW.axf').read_bytes()
assert elf[:6] == b'\x7fELF\x01\x01'
phoff, shoff = struct.unpack_from('<II', elf, 28)
phsize, phnum, shsize, shnum = struct.unpack_from('<4H', elf, 42)
sections = [struct.unpack_from('<10I', elf, shoff + i * shsize) for i in range(shnum)]
symbols = {}
for section in sections:
    if section[1] != 2:
        continue
    strings_section = sections[section[6]]
    strings = elf[strings_section[4]:strings_section[4] + strings_section[5]]
    for offset in range(section[4], section[4] + section[5], section[9]):
        name, value, size, info, other, index = struct.unpack_from('<IIIBBH', elf, offset)
        name = strings[name:strings.find(b'\0', name)].decode('utf-8')
        if name and index:
            symbols[name] = (value, size)


def address(name):
    return symbols[name][0] & ~1


def machine():
    emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    emu.mem_map(0x08000000, 0x80000)
    emu.mem_map(0x20000000, 0x20000)
    emu.mem_map(0xE000E000, 0x2000)
    for i in range(phnum):
        kind, off, virtual, physical, size, memory, flags, alignment = struct.unpack_from('<8I', elf, phoff + i * phsize)
        if kind == 1 and size:
            emu.mem_write(virtual, elf[off:off + size])
    emu.reg_write(UC_ARM_REG_SP, 0x2001F000)
    emu.reg_write(UC_ARM_REG_LR, 0x0807F001)
    return emu


def run(name, hooks, args=(), memory=()):
    emu = machine()
    for pointer, data in memory:
        emu.mem_write(pointer, data)
    for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
        emu.reg_write(register, value)
    lookup = {address(key): value for key, value in hooks.items()}
    stopped = []

    def intercept(uc, pc, size, data):
        if pc == 0x0807F000:
            stopped.append(True)
            uc.emu_stop()
        elif pc in lookup:
            result = lookup[pc](uc)
            if result == 'stop':
                stopped.append(True)
                uc.emu_stop()
            else:
                if result is not None:
                    uc.reg_write(UC_ARM_REG_R0, result)
                uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))

    emu.hook_add(UC_HOOK_CODE, intercept)
    emu.emu_start(address(name) | 1, 0, timeout=2_000_000, count=100_000)
    assert stopped, name + ' did not reach expected return/block/fault'
    return emu


reports = []


def check(label, result):
    assert result, label
    reports.append('PASS: ' + label)


# A successful link alone does not guarantee the startup vectors use the port.
emu = machine()
vectors = struct.unpack('<16I', emu.mem_read(0x08000000, 64))
for index, name in ((11, 'SVC_Handler'), (14, 'PendSV_Handler'), (15, 'SysTick_Handler')):
    check(name + ' linked into vector table', vectors[index] == (address(name) | 1))
check('SVC/PendSV are substantive port handlers', symbols['SVC_Handler'][1] > 20 and symbols['PendSV_Handler'][1] > 50)
check('Application stacks each 3 KiB', symbols['vofaTaskBuffer'][1] == 3072 and symbols['displayTaskBuffer'][1] == 3072)
check('CMSIS dynamic heap available for future objects', 'pvPortMalloc' in symbols)

for state, ticks in ((1, 0), (2, 1), (0, 1)):
    calls = []
    run('SysTick_Handler', {
        'xTaskGetSchedulerState': lambda uc: state,
        'HAL_IncTick': lambda uc: calls.append('HAL'),
        'xPortSysTickHandler': lambda uc: calls.append('RTOS'),
    })
    check(f'SysTick scheduler state {state}: HAL untouched, RTOS gated', calls == ['RTOS'] * ticks)


def cstring(uc, pointer):
    data = bytes(uc.mem_read(pointer, 32))
    return data.split(b'\0', 1)[0].decode()


created = []
events = []


def create_task(uc):
    attr = struct.unpack('<8I', uc.mem_read(uc.reg_read(UC_ARM_REG_R2), 32))
    created.append((cstring(uc, attr[0]), attr[5], attr[6], attr[4], attr[2]))
    return attr[2]


run('MX_FREERTOS_Init', {'osThreadNew': create_task})
check('CubeMX creates both configured static tasks', [(x[0], x[1], x[2]) for x in created] == [('vofaTask', 3072, 32), ('displayTask', 3072, 24)])
check('Separate task stacks and control blocks', created[0][3] != created[1][3] and created[0][4] != created[1][4])
failure = []
run('MX_FREERTOS_Init', {'osThreadNew': lambda uc: 0, 'Error_Handler': lambda uc: (failure.append(1), 'stop')[1]})
check('Task creation failure reaches motor stop path', failure == [1])
for instance, hal_ticks in ((0x40001000, 1), (0x40001400, 0)):
    calls = []
    run('HAL_TIM_PeriodElapsedCallback', {
        'HAL_IncTick': lambda uc: calls.append('HAL'),
        'Balance_TimerCallback': lambda uc: calls.append('control-dispatch'),
    }, (0x2001E000,), [(0x2001E000, struct.pack('<I', instance))])
    check(f'Timer callback {instance:#x} routes HAL tick correctly', calls == ['HAL'] * hal_ticks + ['control-dispatch'])

for thread, worker, delay, watermark in (
    ('StartVofaTask', 'Vofa_Task', 2, 'g_vofa_stack_free_words'),
    ('StartDisplayTask', 'show_task', 10, 'g_display_stack_free_words'),
):
    calls = []
    def block(uc):
        calls.append(uc.reg_read(UC_ARM_REG_R0))
        return 'stop'
    emu = run(thread, {
        worker: lambda uc: calls.append('work'),
        'uxTaskGetStackHighWaterMark': lambda uc: 123,
        'osDelay': block,
    })
    check(thread + ' performs work then blocks', calls == ['work', delay])
    check(thread + ' records stack headroom', struct.unpack('<I', emu.mem_read(address(watermark), 4))[0] == 123)

faults = []
emu = run('AppRTOS_AssertFailed', {'Error_Handler': lambda uc: (faults.append(1), 'stop')[1]}, (0x1234, 57))
check('RTOS assertion reaches stop path and records line', faults == [1] and struct.unpack('<I', emu.mem_read(address('g_rtos_assert_line'), 4))[0] == 57)
faults.clear()
run('vApplicationStackOverflowHook', {'Error_Handler': lambda uc: (faults.append(1), 'stop')[1]}, (0, 0x1234))
check('Stack overflow reaches stop path', faults == [1])
for state, expected in ((1, ('HAL', 5)), (2, ('RTOS', 6))):
    calls = []
    run('delay_ms', {
        'xTaskGetSchedulerState': lambda uc: state,
        'HAL_Delay': lambda uc: calls.append(('HAL', uc.reg_read(UC_ARM_REG_R0))),
        'vTaskDelay': lambda uc: calls.append(('RTOS', uc.reg_read(UC_ARM_REG_R0))),
    }, (5,))
    check(f'Millisecond delay uses correct clock before/after scheduler ({state})', calls == [expected])
emu = run('vApplicationGetIdleTaskMemory', {}, (0x2001E000, 0x2001E004, 0x2001E008))
tcb, stack, words = struct.unpack('<3I', emu.mem_read(0x2001E000, 12))
check('Idle task has valid static TCB and 1 KiB stack', 0x20000000 <= tcb < 0x20020000 and 0x20000000 <= stack < 0x20020000 and words == 256)
text = '\n'.join(reports) + '\n'
(ROOT / 'tests/rtos_test_results.txt').write_text(text, encoding='utf-8')
print(text, end='')
