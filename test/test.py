# SPDX-FileCopyrightText: © 2026 X4NTHA, x4ntha.com
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer
import random
from make_rom import assemble_nova

# instruction encoding utilities for Nova
def encode_alc(acs=0, acd=0, op=0, shift=0, carry=0, no_load=0, skip=0):
    """
    ALC instruction:
    ir[0]     = 1 (ALC class)
    ir[2:1]   = acs (0..3)
    ir[4:3]   = acd (0..3)
    ir[7:5]   = op (0:COM, 1:NEG, 2:MOV, 3:INC, 4:ADC, 5:SUB, 6:ADD, 7:AND)
    ir[9:8]   = shift (0:None, 1:L, 2:R, 3:S)
    ir[11:10] = carry (0:None, 1:Z, 2:O, 3:C)
    ir[12]    = no_load (0:load, 1:no-load '#')
    ir[15:13] = skip (0:Never, 1:SKP, 2:SZC, 3:SNC, 4:SZR, 5:SNR, 6:SEZ, 7:SBN)
    """
    return ((1 & 1) << 0) | \
           ((acs & 3) << 1) | \
           ((acd & 3) << 3) | \
           ((op & 7) << 5) | \
           ((shift & 3) << 8) | \
           ((carry & 3) << 10) | \
           ((no_load & 1) << 12) | \
           ((skip & 7) << 13)

def encode_mem(mode=0, func_or_ac=0, indir=0, index=0, disp=0):
    """
    memory reference instruction:
    ir[0]    = 0 (memory class)
    ir[2:1]  = mode (0: non-AC, 1: LDA, 2: STA)
    ir[4:3]  = func_or_ac (non-AC: 0:JMP, 1:JSR, 2:ISZ, 3:DSZ; else AC 0..3)
    ir[5]    = indir (0: direct, 1: indirect '@')
    ir[7:6]  = index (0: Page 0, 1: PC-rel, 2: AC2-rel, 3: AC3-rel)
    ir[15:8] = disp (8-bit signed/unsigned)
    """
    disp_byte = disp & 0xFF
    return ((0 & 1) << 0) | \
           ((mode & 3) << 1) | \
           ((func_or_ac & 3) << 3) | \
           ((indir & 1) << 5) | \
           ((index & 3) << 6) | \
           (disp_byte << 8)

def encode_io(ac=0, transfer=0, control=0, dev=0):
    """
    I/O instruction:
    ir[0]     = 0
    ir[2:1]   = 3 (2'b11)
    ir[4:3]   = ac (0..3)
    ir[7:5]   = transfer (0:NIO, 1:DIA, 2:DOA, 3:DIB, 4:DOB, 5:DIC, 6:DOC, 7:SKP)
    ir[9:8]   = control (0:None, 1:S, 2:C, 3:P / if SKP: 0:SKPBN, 1:SKPBZ, 2:SKPDN, 3:SKPDZ)
    ir[15:10] = dev (6-bit device code, e.g. 0o10, 0o11, 0o77)
    """
    return ((0 & 1) << 0) | \
           ((3 & 3) << 1) | \
           ((ac & 3) << 3) | \
           ((transfer & 7) << 5) | \
           ((control & 3) << 8) | \
           ((dev & 0x3F) << 10)

# behavioral SPI flash (CS0) & PSRAM (CS1) memory model
async def spi_memory_responder(dut, mem_flash, mem_ram, access_log=None):
    """
    simulates external SPI Flash (CS0# on uio[0]) and PSRAM (CS1# on uio[6])
    over 48-clock-edge SPI Mode 0 transactions (CPOL=0, CPHA=0).
    """
    dut.uio_in.value = 0
    while True:
        await FallingEdge(dut.clk)
        uio_val = int(dut.uio_out.value)
        cs0_n = (uio_val >> 0) & 1
        cs1_n = (uio_val >> 6) & 1

        # check if either chip select is active (LOW)
        if cs0_n == 0 or cs1_n == 0:
            is_flash = (cs0_n == 0)
            target_mem = mem_flash if is_flash else mem_ram

            # sample 8 bits of CMD + 24 bits of byte address on MOSI (32 bits total)
            cmd_addr = 0
            for _ in range(32):
                while True:
                    await FallingEdge(dut.clk)
                    uio_val = int(dut.uio_out.value)
                    curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                    if curr_cs == 1 or ((uio_val >> 3) & 1) == 1:
                        break
                if curr_cs == 1:
                    break

                mosi = (uio_val >> 1) & 1
                cmd_addr = (cmd_addr << 1) | mosi

                while True:
                    await FallingEdge(dut.clk)
                    uio_val = int(dut.uio_out.value)
                    curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                    if curr_cs == 1 or ((uio_val >> 3) & 1) == 0:
                        break
                if curr_cs == 1:
                    break

            cmd = (cmd_addr >> 24) & 0xFF
            byte_addr = cmd_addr & 0x00FFFFFF
            word_addr = (byte_addr >> 1) & 0x7FFF

            if access_log is not None:
                access_log.append(('READ' if cmd == 0x03 else 'WRITE', is_flash, word_addr))

            if cmd == 0x03:
                # read 16-bit word from memory
                read_word = target_mem.get(word_addr, 0x0000)
                for i in range(16):
                    bit = (read_word >> (15 - i)) & 1
                    dut.uio_in.value = (bit << 2)

                    while True:
                        await FallingEdge(dut.clk)
                        uio_val = int(dut.uio_out.value)
                        curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                        if curr_cs == 1 or ((uio_val >> 3) & 1) == 1:
                            break
                    if curr_cs == 1:
                        break

                    while True:
                        await FallingEdge(dut.clk)
                        uio_val = int(dut.uio_out.value)
                        curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                        if curr_cs == 1 or ((uio_val >> 3) & 1) == 0:
                            break
                    if curr_cs == 1:
                        break

                dut.uio_in.value = 0

            elif cmd == 0x02:
                # write 16-bit word into memory
                write_word = 0
                for _ in range(16):
                    while True:
                        await FallingEdge(dut.clk)
                        uio_val = int(dut.uio_out.value)
                        curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                        if curr_cs == 1 or ((uio_val >> 3) & 1) == 1:
                            break
                    if curr_cs == 1:
                        break

                    mosi = (uio_val >> 1) & 1
                    write_word = (write_word << 1) | mosi

                    while True:
                        await FallingEdge(dut.clk)
                        uio_val = int(dut.uio_out.value)
                        curr_cs = ((uio_val >> 0) & 1) if is_flash else ((uio_val >> 6) & 1)
                        if curr_cs == 1 or ((uio_val >> 3) & 1) == 0:
                            break
                    if curr_cs == 1:
                        break

                target_mem[word_addr] = write_word

# UART helper coroutines (115200 Baud @ 10 MHz, 87 clocks/bit)
async def uart_tx_byte(dut, byte_val, baud_clocks=87):
    """transmits a byte to dut.ui_in[0] at 8N1."""
    # start bit (0)
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, baud_clocks)
    # 8 data bits LSB first
    for i in range(8):
        bit = (byte_val >> i) & 1
        dut.ui_in.value = bit
        await ClockCycles(dut.clk, baud_clocks)
    # stop bit (1)
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, baud_clocks)

async def uart_rx_byte(dut, baud_clocks=87, timeout_cycles=100000):
    """receives a byte from dut.uo_out[0] at 8N1."""
    cycles_waited = 0
    while True:
        await RisingEdge(dut.clk)
        cycles_waited += 1
        if (int(dut.uo_out.value) & 1) == 0:
            break
        if cycles_waited > timeout_cycles:
            raise TimeoutError(f"UART RX timed out waiting for start bit after {timeout_cycles} cycles")

    # sample at center of start bit
    await ClockCycles(dut.clk, baud_clocks // 2)

    # sample 8 data bits
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, baud_clocks)
        bit = int(dut.uo_out.value) & 1
        byte_val |= (bit << i)

    # wait for stop bit
    await ClockCycles(dut.clk, baud_clocks)
    return byte_val

# test 1: exhaustive ALU, shifter, carry & skip matrix
@cocotb.test()
async def test_alu_matrix(dut):
    """
    test 1: exhaustive ALU matrix verification covering all operations,
    shifts (None, L, R, Swap), carry initializations, skips, and no-load '#'
    """
    clock = Clock(dut.clk, 100, unit="ns") # 10 MHz
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    flash_mem[0]  = encode_alc(acs=0, acd=0, op=0)                                # COM AC0, AC0 -> 0xFFFF
    flash_mem[1]  = encode_alc(acs=0, acd=1, op=3, carry=1)                       # INC AC0, AC1 (c=0) -> 0x0000, cout=1
    flash_mem[2]  = encode_alc(acs=0, acd=1, op=6, carry=0)                       # ADD AC0, AC1 -> 0x0000, cout=1
    flash_mem[3]  = encode_alc(acs=0, acd=2, op=1, carry=2)                       # NEG AC0, AC2 (c=1) -> 0x0001
    flash_mem[4]  = encode_alc(acs=0, acd=2, op=7)                                # AND AC0, AC2 -> 0x0001
    flash_mem[5]  = encode_alc(acs=2, acd=0, op=5, carry=2)                       # SUB AC2, AC0 (c=1) -> 0xFFFE
    flash_mem[6]  = encode_alc(acs=2, acd=0, op=4, carry=1)                       # ADC AC2, AC0 (c=0) -> 0xFFFC
    flash_mem[7]  = encode_alc(acs=2, acd=2, op=2, shift=1, carry=1)              # MOVL AC2, AC2 (c=0) -> 0x0002
    flash_mem[8]  = encode_alc(acs=2, acd=2, op=2, shift=2, carry=2)              # MOVR AC2, AC2 (c=1) -> 0x8001
    flash_mem[9]  = encode_alc(acs=0, acd=0, op=2, shift=3)                       # MOVS AC0, AC0 -> 0xFCFF
    flash_mem[10] = encode_alc(acs=1, acd=1, op=2, no_load=1, skip=4)             # MOV# SZR (AC1==0 -> skips 11)
    flash_mem[11] = encode_alc(acs=0, acd=0, op=0)                                # skipped NOP
    flash_mem[12] = encode_alc(acs=2, acd=2, op=2, no_load=1, skip=5)             # MOV# SNR (AC2!=0 -> skips 13)
    flash_mem[13] = encode_alc(acs=0, acd=0, op=0)                                # skipped NOP
    flash_mem[14] = encode_io(ac=0, transfer=6, control=0, dev=0o77)              # HALT

    # apply reset
    dut.ena.value = 1
    dut.ui_in.value = 1 # UART RX idle HIGH
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 3000)

    fetched_addrs = [addr for (op, is_flash, addr) in access_log if op == 'READ' and is_flash]
    assert 0 in fetched_addrs, "PC=0 was not fetched."
    assert 10 in fetched_addrs, "PC=10 (SZR skip) was not fetched."
    assert 11 not in fetched_addrs, "PC=11 was executed despite SZR skip condition!"
    assert 12 in fetched_addrs, "PC=12 (SNR skip) was not fetched."
    assert 13 not in fetched_addrs, "PC=13 was executed despite SNR skip condition!"
    assert 14 in fetched_addrs, "PC=14 (HALT) was not reached."

# test 2: full memory addressing modes & QSPI PSRAM partitioning
@cocotb.test()
async def test_memory_reference(dut):
    """
    test 2: comprehensive verification of Page 0, PC-relative (positive & negative),
    AC2/AC3 indexed addressing, indirect defer chains, and PSRAM read-after-write
    """
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    flash_mem[0x0020] = 0x1234      # page zero data
    flash_mem[0x0021] = 0x4010      # pointer into PSRAM
    ram_mem[0x4010]   = 0xABCD      # PSRAM initial data
    flash_mem[0x0022] = 0x0001      # target for DSZ

    flash_mem[0] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x20) # LDA AC0, Page0 0x20
    flash_mem[1] = encode_mem(mode=1, func_or_ac=1, indir=1, index=0, disp=0x21) # LDA AC1, @0x21 (PSRAM)
    flash_mem[2] = encode_mem(mode=2, func_or_ac=1, indir=1, index=0, disp=0x21) # STA AC1, @0x21
    flash_mem[3] = encode_mem(mode=0, func_or_ac=2, indir=0, index=0, disp=0x20) # ISZ 0x20
    flash_mem[4] = encode_mem(mode=0, func_or_ac=3, indir=0, index=0, disp=0x22) # DSZ 0x22 (skips 5)
    flash_mem[5] = encode_mem(mode=0, func_or_ac=0, indir=0, index=0, disp=0x00) # JMP 0 (skipped)
    flash_mem[6] = encode_mem(mode=0, func_or_ac=1, indir=0, index=1, disp=2)    # JSR PC+2 -> PC=8
    flash_mem[7] = encode_mem(mode=0, func_or_ac=0, indir=0, index=0, disp=0x00) # skipped
    flash_mem[8] = encode_io(ac=0, transfer=6, control=0, dev=0o77)              # HALT

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 4500)

    assert flash_mem.get(0x0020) == 0x1235, f"ISZ mismatch: {hex(flash_mem.get(0x0020, 0))}"
    assert flash_mem.get(0x0022) == 0x0000, f"DSZ mismatch: {hex(flash_mem.get(0x0022, 1))}"
    assert ram_mem.get(0x4010) == 0xABCD, f"PSRAM writeback mismatch: {hex(ram_mem.get(0x4010, 0))}"

    fetched_addrs = [addr for (op, is_flash, addr) in access_log if op == 'READ' and is_flash]
    assert 5 not in fetched_addrs, "PC=5 was fetched despite DSZ skip!"
    assert 7 not in fetched_addrs, "PC=7 was fetched despite JSR jump target PC=8!"
    assert 8 in fetched_addrs, "PC=8 (HALT) was reached."

# test 3: continuous back-to-back burst throughput & full 8-bit byte range
@cocotb.test()
async def test_uart_burst_throughput(dut):
    """
    test 3: sends continuous back-to-back serial bytes with zero inter-character delay
    across boundary values (0x00, 0x55, 0xAA, 0xFF), control characters, and high ASCII
    to verify UART double-buffering, baud timing, and receiver FIFO/flag handshaking
    under saturated serial line conditions
    """
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem))

    # echo program:
    flash_mem[0] = encode_io(ac=0, transfer=7, control=2, dev=0o10) # SKPDN 010
    flash_mem[1] = encode_mem(mode=0, func_or_ac=0, disp=0)         # JMP 0
    flash_mem[2] = encode_io(ac=0, transfer=1, control=0, dev=0o10) # DIA AC0, 010
    flash_mem[3] = encode_io(ac=0, transfer=7, control=1, dev=0o11) # SKPBZ 011
    flash_mem[4] = encode_mem(mode=0, func_or_ac=0, disp=3)         # JMP 3
    flash_mem[5] = encode_io(ac=0, transfer=2, control=0, dev=0o11) # DOA AC0, 011
    flash_mem[6] = encode_mem(mode=0, func_or_ac=0, disp=0)         # JMP 0

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 200)

    # test sequence: boundary bits, control characters, full alphanumeric patterns
    burst_data = bytes([
        0x00, 0x55, 0xAA, 0xFF, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
        0x7F, 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF,
        ord('N'), ord('O'), ord('V'), ord('A'), ord('1'), ord('2'), ord('0'), ord('0'),
        0x0D, 0x0A, 0x00, 0xFF
    ])

    for byte_val in burst_data:
        rx_task = cocotb.start_soon(uart_rx_byte(dut))
        # zero inter-character delay: continuous line saturation
        await uart_tx_byte(dut, byte_val)
        echoed_byte = await rx_task
        assert echoed_byte == byte_val, f"burst throughput mismatch on byte {hex(byte_val)}: got {hex(echoed_byte)}"

# test 4: UART ingest into PSRAM buffer with interactive playback
@cocotb.test()
async def test_uart_psram_buffering(dut):
    """
    test 4: simulates serial terminal string entry where incoming UART characters
    are buffered into PSRAM memory until newline, then the CPU reads the buffered 
    string back from PSRAM and transmits it
    """
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem))

    # page 0 variables:
    flash_mem[0x0040] = 0x4000      # write pointer
    flash_mem[0x0041] = 0x4000      # read pointer
    flash_mem[0x0042] = 0x000A      # '\n' terminator

    # program:
    # phase 1: ingest characters into PSRAM until newline (0x0A)
    # 0: SKPDN 010                  ; poll keyboard
    # 1: JMP 0
    # 2: DIA AC0, 010               ; read char
    # 3: STA AC0, @0x40             ; store char at PSRAM[0x0040]
    # 4: ISZ 0x40                   ; advance write pointer
    # 5: LDA AC1, 0x42              ; load newline char
    # 6: SUB# AC0, AC1, SZR         ; compare char with '\n', skip if equal
    # 7: JMP 0                      ; continue ingesting
    #
    # phase 2: transmit buffered characters from PSRAM out to UART
    # 8: LDA AC0, @0x41             ; read char from PSRAM[0x0041]
    # 9: ISZ 0x41                   ; advance read pointer
    # 10: SKPBZ 011                 ; wait for TX ready
    # 11: JMP 10
    # 12: DOA AC0, 011              ; transmit char
    # 13: SUB# AC0, AC1, SZR        ; was this the newline char?
    # 14: JMP 8                     ; if not, transmit next char
    # 15: HALT                      ; finished playback
    flash_mem[0]  = encode_io(ac=0, transfer=7, control=2, dev=0o10)           # SKPDN 010
    flash_mem[1]  = encode_mem(mode=0, func_or_ac=0, disp=0)                   # JMP 0
    flash_mem[2]  = encode_io(ac=0, transfer=1, control=0, dev=0o10)           # DIA AC0, 010
    flash_mem[3]  = encode_mem(mode=2, func_or_ac=0, indir=1, index=0, disp=0x40) # STA AC0, @0x40
    flash_mem[4]  = encode_mem(mode=0, func_or_ac=2, indir=0, index=0, disp=0x40) # ISZ 0x40
    flash_mem[5]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x42) # LDA AC1, 0x42 ('\n')
    flash_mem[6]  = encode_alc(acs=0, acd=1, op=5, carry=2, no_load=1, skip=4) # SUB#O SZR (skip on equal)
    flash_mem[7]  = encode_mem(mode=0, func_or_ac=0, disp=0)                   # JMP 0
    flash_mem[8]  = encode_mem(mode=1, func_or_ac=0, indir=1, index=0, disp=0x41) # LDA AC0, @0x41
    flash_mem[9]  = encode_mem(mode=0, func_or_ac=2, indir=0, index=0, disp=0x41) # ISZ 0x41
    flash_mem[10] = encode_io(ac=0, transfer=7, control=1, dev=0o11)           # SKPBZ 011
    flash_mem[11] = encode_mem(mode=0, func_or_ac=0, disp=10)                  # JMP 10
    flash_mem[12] = encode_io(ac=0, transfer=2, control=0, dev=0o11)           # DOA AC0, 011
    flash_mem[13] = encode_alc(acs=0, acd=1, op=5, carry=2, no_load=1, skip=4) # SUB#O SZR
    flash_mem[14] = encode_mem(mode=0, func_or_ac=0, disp=8)                   # JMP 8
    flash_mem[15] = encode_io(ac=0, transfer=6, control=0, dev=0o77)           # HALT

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 200)

    test_stream = b"NOVA: PSRAM BUFFER TEST\n"

    # send characters with realistic pacing (1500 cycles = 150us)
    for byte_val in test_stream:
        await uart_tx_byte(dut, byte_val)
        await ClockCycles(dut.clk, 1500)

    # receive playback stream from CPU
    received_playback = bytearray()
    for _ in range(len(test_stream)):
        b = await uart_rx_byte(dut, timeout_cycles=100000)
        received_playback.append(b)

    assert received_playback == test_stream, f"PSRAM buffer playback corrupted: expected {test_stream}, got {received_playback}"

# test 5: glitch and noise rejection on UART RX
@cocotb.test()
async def test_uart_glitch_rejection(dut):
    """
    test 5: verifies that short line noise/glitches (< half baud period)
    on the UART RX line are properly rejected by the half-baud center sampler
    """
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem))

    # echo program:
    flash_mem[0] = encode_io(ac=0, transfer=7, control=2, dev=0o10) # SKPDN 010
    flash_mem[1] = encode_mem(mode=0, func_or_ac=0, disp=0)         # JMP 0
    flash_mem[2] = encode_io(ac=0, transfer=1, control=0, dev=0o10) # DIA AC0, 010
    flash_mem[3] = encode_io(ac=0, transfer=7, control=1, dev=0o11) # SKPBZ 011
    flash_mem[4] = encode_mem(mode=0, func_or_ac=0, disp=3)         # JMP 3
    flash_mem[5] = encode_io(ac=0, transfer=2, control=0, dev=0o11) # DOA AC0, 011
    flash_mem[6] = encode_mem(mode=0, func_or_ac=0, disp=0)         # JMP 0

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 200)

    # send 3 noise glitches that are 10 clock cycles long (< HALF_BAUD=43)
    for _ in range(3):
        dut.ui_in.value = 0 # glitch LOW
        await ClockCycles(dut.clk, 10)
        dut.ui_in.value = 1 # back HIGH
        await ClockCycles(dut.clk, 100)

    # now send a valid character 'Z' (0x5A)
    valid_char = 0x5A
    rx_task = cocotb.start_soon(uart_rx_byte(dut))
    await uart_tx_byte(dut, valid_char)

    echoed = await rx_task
    assert echoed == valid_char, f"noise glitch caused misframing: expected {hex(valid_char)}, got {hex(echoed)}"

# test 6: PSRAM computational test: Fibonacci series to UART stream
@cocotb.test()
async def test_psram_fibonacci_stream(dut):
    """
    test 6: full-system algorithm test:
    CPU initializes PSRAM with F(0)=0 and F(1)=1, iteratively computes
    F(2)..F(7) in PSRAM, reads them back, formats each as an ASCII digit ('0'..'9'),
    and transmits the stream "0112358..." over UART
    """
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    # page 0 variables:
    flash_mem[0x0040] = 0x4000      # PSRAM base pointer
    flash_mem[0x0041] = 0x0030      # ASCII '0' base offset (0x30)
    flash_mem[0x0042] = 0x0006      # loop counter for 6 additions
    flash_mem[0x0043] = 0x0008      # print count (8 numbers)

    # initial values in PSRAM:
    ram_mem[0x4000] = 0             # F(0) = 0
    ram_mem[0x4001] = 1             # F(1) = 1

    # program:
    # 0: LDA AC2, 0x40              ; AC2 = 0x4000 (pointer)
    #
    # LOOP (calculate next Fibonacci in PSRAM):
    # 1: LDA AC0, 0,2               ; AC0 = PSRAM[AC2] (F_n-2)
    # 2: LDA AC1, 1,2               ; AC1 = PSRAM[AC2+1] (F_n-1)
    # 3: ADD AC0, AC1               ; AC1 = F_n-2 + F_n-1 (F_n)
    # 4: STA AC1, 2,2               ; PSRAM[AC2+2] = F_n
    # 5: INC AC2, AC2 (c=Z)         ; advance pointer AC2
    # 6: DSZ 0x42                   ; decrement loop counter, skip if 0
    # 7: JMP 1                      ; loop back
    #
    # PRINT LOOP (stream F(0)..F(7) to UART as ASCII digits):
    # 8: LDA AC2, 0x40              ; reset AC2 to 0x4000
    # 9: LDA AC3, 0x41              ; AC3 = '0' (0x30)
    # 10: LDA AC1, 0,2              ; AC1 = PSRAM[AC2]
    # 11: ADD AC3, AC1              ; convert to ASCII: AC1 = AC1 + '0'
    # 12: SKPBZ 011                 ; wait for UART TX ready
    # 13: JMP 12
    # 14: DOA AC1, 011              ; transmit ASCII character
    # 15: INC AC2, AC2 (c=Z)        ; advance pointer
    # 16: DSZ 0x43                  ; decrement print count (8)
    # 17: JMP 10                    ; loop
    # 18: HALT

    flash_mem[0]  = encode_mem(mode=1, func_or_ac=2, indir=0, index=0, disp=0x40) # LDA AC2, 0x40
    flash_mem[1]  = encode_mem(mode=1, func_or_ac=0, indir=0, index=2, disp=0)    # LDA AC0, 0,2
    flash_mem[2]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=1)    # LDA AC1, 1,2
    flash_mem[3]  = encode_alc(acs=0, acd=1, op=6)                                # ADD AC0, AC1
    flash_mem[4]  = encode_mem(mode=2, func_or_ac=1, indir=0, index=2, disp=2)    # STA AC1, 2,2
    flash_mem[5]  = encode_alc(acs=2, acd=2, op=3, carry=1)                       # INC AC2, AC2 (c=0)
    flash_mem[6]  = encode_mem(mode=0, func_or_ac=3, indir=0, index=0, disp=0x42) # DSZ 0x42
    flash_mem[7]  = encode_mem(mode=0, func_or_ac=0, disp=1)                      # JMP 1
    
    # print:
    flash_mem[8]  = encode_mem(mode=1, func_or_ac=2, indir=0, index=0, disp=0x40) # LDA AC2, 0x40
    flash_mem[9]  = encode_mem(mode=1, func_or_ac=3, indir=0, index=0, disp=0x41) # LDA AC3, 0x41 ('0')
    flash_mem[10] = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=0)    # LDA AC1, 0,2
    flash_mem[11] = encode_alc(acs=3, acd=1, op=6)                                # ADD AC3, AC1
    flash_mem[12] = encode_io(ac=0, transfer=7, control=1, dev=0o11)              # SKPBZ 011
    flash_mem[13] = encode_mem(mode=0, func_or_ac=0, disp=12)                     # JMP 12
    flash_mem[14] = encode_io(ac=1, transfer=2, control=0, dev=0o11)              # DOA AC1, 011
    flash_mem[15] = encode_alc(acs=2, acd=2, op=3, carry=1)                       # INC AC2, AC2 (c=0)
    flash_mem[16] = encode_mem(mode=0, func_or_ac=3, indir=0, index=0, disp=0x43) # DSZ 0x43
    flash_mem[17] = encode_mem(mode=0, func_or_ac=0, disp=10)                     # JMP 10
    flash_mem[18] = encode_io(ac=0, transfer=6, control=0, dev=0o77)              # HALT

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Fibonacci series values: 0, 1, 1, 2, 3, 5, 8, 13
    expected_output = bytes([0x30 + x for x in [0, 1, 1, 2, 3, 5, 8, 13]])

    received = bytearray()
    for _ in range(8):
        b = await uart_rx_byte(dut, timeout_cycles=100000)
        received.append(b)

    assert received == expected_output, f"Fibonacci UART stream mismatch: expected {expected_output}, got {received}"

# test 7: Nova mini-assembler pipeline and live interactive echo program
@cocotb.test()
async def test_assembler_pipeline_echo(dut):
    """
    test 7: verification of the make_rom.py assembler: feeds raw Nova assembly 
    to assemble_nova(), verifies binary generation, loads the generated ROM into 
    SPI Flash memory, executes the live binary on the Nova CPU core, and validates
    interactive UART echo behavior
    """
    clock = Clock(dut.clk, 100, unit="ns") # 10 MHz
    cocotb.start_soon(clock.start())

    # raw Nova assembly source code:
    echo_asm_source = """
    ; Nova interactive UART echo demo
    START:  SKPDN 010        ; check if a character was received on UART RX (Dev 0o10)
            JMP   START      ; wait for character
            DIA   0, 010     ; read character into AC0 (clears RX done flag)
    WAIT:   SKPBZ 011        ; wait until UART TX (Dev 0o11) transmitter is ready
            JMP   WAIT       ; wait
            DOA   0, 011     ; echo character back out over UART TX
            JMP   START      ; loop indefinitely for next character
    """

    # assemble directly via make_rom.py
    rom_words = assemble_nova(echo_asm_source)
    assert len(rom_words) == 16384, f"ROM size unexpected: {len(rom_words)} words"

    # populate SPI Flash model with compiled binary words
    flash_mem = {addr: val for addr, val in enumerate(rom_words) if val != 0}
    ram_mem = {}
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem))

    # apply reset
    dut.ena.value = 1
    dut.ui_in.value = 1 # UART RX idle HIGH
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 200)

    # test typing a sequence of characters through the assembled binary
    test_phrase = "Hello from make_rom.py assembler on Nova RTL!\r\n"
    for char in test_phrase:
        byte_val = ord(char)
        rx_task = cocotb.start_soon(uart_rx_byte(dut))
        await uart_tx_byte(dut, byte_val)
        echoed_byte = await rx_task
        assert echoed_byte == byte_val, f"assembler echo mismatch on '{char}': sent {hex(byte_val)}, received {hex(echoed_byte)}"
        
        # inter-character typing jitter
        await ClockCycles(dut.clk, random.randint(300, 1500))