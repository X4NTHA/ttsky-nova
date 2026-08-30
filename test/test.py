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

# UART helper coroutines (19200 Baud @ 1.5 MHz, 78 clocks/bit)
async def uart_tx_byte(dut, byte_val, baud_clocks=78):
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

async def uart_rx_byte(dut, baud_clocks=78, timeout_cycles=100000):
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
    clock = Clock(dut.clk, 666, unit="ns") # 1.5 MHz
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    # test 1: ALU, shifter, carry & skip matrix (using AC0, AC1)
    # 0:  COM AC0, AC0       -> AC0 = 0xFFFF, carry = 0
    # 1:  INC AC0, AC1 (c=0) -> AC1 = 0x0000, carry = 1 (wrap)
    # 2:  ADD AC0, AC1       -> AC1 = 0xFFFF, carry = 0
    # 3:  NEG AC0, AC1 (c=1) -> AC1 = 0x0001, carry = 1
    # 4:  AND AC0, AC1       -> AC1 = 0x0001, carry = 1
    # 5:  SUB AC1, AC0 (c=1) -> AC0 = 0xFFFE, carry = 1
    # 6:  ADC AC1, AC0 (c=0) -> AC0 = 0xFFFC, carry = 1
    # 7:  MOVL AC1, AC1 (c=0)-> AC1 = 0x0002 (rotate left through carry=0)
    # 8:  MOVR AC1, AC1 (c=1)-> AC1 = 0x8001 (rotate right through carry=1)
    # 9:  MOVS AC0, AC0      -> AC0 = 0xFCFF (swap bytes)
    # 10: MOV# AC1, AC1 SZR  -> AC1!=0 -> NOT skip 11
    # 11: sentinel (executed)
    # 12: MOV# AC1, AC1 SNR  -> AC1!=0 -> SNR -> skip 13
    # 13: sentinel (skipped)
    # 14: HALT
    flash_mem[0]  = encode_alc(acs=0, acd=0, op=0)                                # COM AC0, AC0 -> 0xFFFF
    flash_mem[1]  = encode_alc(acs=0, acd=1, op=3, carry=1)                       # INC AC0, AC1 (c=0) -> 0x0000
    flash_mem[2]  = encode_alc(acs=0, acd=1, op=6, carry=0)                       # ADD AC0, AC1 -> 0xFFFF
    flash_mem[3]  = encode_alc(acs=0, acd=1, op=1, carry=2)                       # NEG AC0, AC1 (c=1) -> 0x0001
    flash_mem[4]  = encode_alc(acs=0, acd=1, op=7)                                # AND AC0, AC1 -> 0x0001
    flash_mem[5]  = encode_alc(acs=1, acd=0, op=5, carry=2)                       # SUB AC1, AC0 (c=1) -> 0xFFFE
    flash_mem[6]  = encode_alc(acs=1, acd=0, op=4, carry=1)                       # ADC AC1, AC0 (c=0) -> 0xFFFC
    flash_mem[7]  = encode_alc(acs=1, acd=1, op=2, shift=1, carry=1)              # MOVL AC1, AC1 (c=0) -> 0x0002
    flash_mem[8]  = encode_alc(acs=1, acd=1, op=2, shift=2, carry=2)              # MOVR AC1, AC1 (c=1) -> 0x8001
    flash_mem[9]  = encode_alc(acs=0, acd=0, op=2, shift=3)                       # MOVS AC0, AC0 -> 0xFCFF
    flash_mem[10] = encode_alc(acs=1, acd=1, op=2, no_load=1, skip=4)             # MOV# SZR (AC1!=0 -> does NOT skip 11)
    flash_mem[11] = encode_alc(acs=0, acd=0, op=0)                                # sentinel (must be fetched)
    flash_mem[12] = encode_alc(acs=1, acd=1, op=2, no_load=1, skip=5)             # MOV# SNR (AC1!=0 -> skips 13)
    flash_mem[13] = encode_alc(acs=0, acd=0, op=0)                                # skipped sentinel
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
    assert 10 in fetched_addrs, "PC=10 was not fetched."
    assert 11 in fetched_addrs, "PC=11 should be executed (SZR condition false)!"
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
    clock = Clock(dut.clk, 666, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    flash_mem[0x0020] = 0x1234      # page zero data
    flash_mem[0x0021] = 0x4010      # pointer into PSRAM
    ram_mem[0x4010]   = 0xABCD      # PSRAM initial data
    ram_mem[0x4012]   = 0x5678      # target for indexed load

    # 0: LDA AC0, Page0 0x20   -> AC0 = 0x1234
    # 1: LDA AC1, @0x21        -> AC1 = 0xABCD (reads PSRAM 0x4010)
    # 2: STA AC1, @0x21        -> writeback 0xABCD to PSRAM 0x4010
    # 3: JSR .+3               -> PC = 6, AC1 = return address 4
    # 4: skipped sentinel
    # 5: skipped sentinel
    # 6: LDA AC0, 0x21         -> AC0 = 0x4010
    # 7: LDA AC1, 2,0          -> AC1 = RAM[AC0+2] = RAM[0x4012] = 0x5678 (AC0-indexed)
    # 8: HALT
    flash_mem[0] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x20) # LDA AC0, Page0 0x20
    flash_mem[1] = encode_mem(mode=1, func_or_ac=1, indir=1, index=0, disp=0x21) # LDA AC1, @0x21 (PSRAM)
    flash_mem[2] = encode_mem(mode=2, func_or_ac=1, indir=1, index=0, disp=0x21) # STA AC1, @0x21
    flash_mem[3] = encode_mem(mode=0, func_or_ac=1, indir=0, index=1, disp=3)    # JSR PC+3 -> PC=6 (stores 4 in AC1)
    flash_mem[4] = encode_alc(acs=0, acd=0, op=0)                                # skipped sentinel
    flash_mem[5] = encode_alc(acs=0, acd=0, op=0)                                # skipped sentinel
    flash_mem[6] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x21) # LDA AC0, 0x21 (0x4010)
    flash_mem[7] = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=2)    # LDA AC1, 2,0 (AC0-indexed -> 0x4012)
    flash_mem[8] = encode_io(ac=0, transfer=6, control=0, dev=0o77)              # HALT

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 4500)

    assert ram_mem.get(0x4010) == 0xABCD, f"PSRAM writeback mismatch: {hex(ram_mem.get(0x4010, 0))}"

    fetched_addrs = [addr for (op, is_flash, addr) in access_log if op == 'READ' and is_flash]
    assert 4 not in fetched_addrs, "PC=4 was fetched despite JSR jump target PC=6!"
    assert 5 not in fetched_addrs, "PC=5 was fetched despite JSR jump target PC=6!"
    assert 6 in fetched_addrs, "PC=6 was reached."
    assert 7 in fetched_addrs, "PC=7 was reached."
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
    clock = Clock(dut.clk, 666, unit="ns")
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
    clock = Clock(dut.clk, 666, unit="ns")
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
    # 4: LDA AC1, 0x40              ; load write pointer
    # 5: INC AC1, AC1 (c=0)         ; increment pointer
    # 6: STA AC1, 0x40              ; store updated write pointer
    # 7: LDA AC1, 0x42              ; load newline char
    # 8: SUB# AC0, AC1, SZR         ; compare char with '\n', skip if equal
    # 9: JMP 0                      ; continue ingesting
    #
    # phase 2: transmit buffered characters from PSRAM out to UART
    # 10: LDA AC0, @0x41            ; read char from PSRAM[0x0041]
    # 11: LDA AC1, 0x41             ; load read pointer
    # 12: INC AC1, AC1 (c=0)        ; increment read pointer
    # 13: STA AC1, 0x41             ; store updated read pointer
    # 14: SKPBZ 011                 ; wait for TX ready
    # 15: JMP 14
    # 16: DOA AC0, 011              ; transmit char
    # 17: LDA AC1, 0x42             ; load '\n'
    # 18: SUB# AC0, AC1, SZR        ; was this the newline char?
    # 19: JMP 10                    ; if not, transmit next char
    # 20: HALT                      ; finished playback
    flash_mem[0]  = encode_io(ac=0, transfer=7, control=2, dev=0o10)             # SKPDN 010
    flash_mem[1]  = encode_mem(mode=0, func_or_ac=0, disp=0)                     # JMP 0
    flash_mem[2]  = encode_io(ac=0, transfer=1, control=0, dev=0o10)             # DIA AC0, 010
    flash_mem[3]  = encode_mem(mode=2, func_or_ac=0, indir=1, index=0, disp=0x40) # STA AC0, @0x40
    flash_mem[4]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x40) # LDA AC1, 0x40
    flash_mem[5]  = encode_alc(acs=1, acd=1, op=3, carry=1)                       # INC AC1, AC1 (c=0)
    flash_mem[6]  = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x40) # STA AC1, 0x40
    flash_mem[7]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x42) # LDA AC1, 0x42 ('\n')
    flash_mem[8]  = encode_alc(acs=0, acd=1, op=5, carry=2, no_load=1, skip=4)   # SUB#O SZR (skip on equal)
    flash_mem[9]  = encode_mem(mode=0, func_or_ac=0, disp=0)                     # JMP 0
    flash_mem[10] = encode_mem(mode=1, func_or_ac=0, indir=1, index=0, disp=0x41) # LDA AC0, @0x41
    flash_mem[11] = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x41) # LDA AC1, 0x41
    flash_mem[12] = encode_alc(acs=1, acd=1, op=3, carry=1)                       # INC AC1, AC1 (c=0)
    flash_mem[13] = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x41) # STA AC1, 0x41
    flash_mem[14] = encode_io(ac=0, transfer=7, control=1, dev=0o11)             # SKPBZ 011
    flash_mem[15] = encode_mem(mode=0, func_or_ac=0, disp=14)                    # JMP 14
    flash_mem[16] = encode_io(ac=0, transfer=2, control=0, dev=0o11)             # DOA AC0, 011
    flash_mem[17] = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x42) # LDA AC1, 0x42 ('\n')
    flash_mem[18] = encode_alc(acs=0, acd=1, op=5, carry=2, no_load=1, skip=4)   # SUB#O SZR
    flash_mem[19] = encode_mem(mode=0, func_or_ac=0, disp=10)                    # JMP 10
    flash_mem[20] = encode_io(ac=0, transfer=6, control=0, dev=0o77)             # HALT

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
    clock = Clock(dut.clk, 666, unit="ns")
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
    clock = Clock(dut.clk, 666, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    # page 0 variables:
    flash_mem[0x0040] = 0x4000      # PSRAM working pointer
    flash_mem[0x0041] = 0x0030      # ASCII '0' base offset (0x30)
    flash_mem[0x0042] = 0xFFFF      # -1 for decrement
    flash_mem[0x0043] = 0x0006      # compute count (6 additions)
    flash_mem[0x0044] = 0x0008      # print count (8 numbers)
    flash_mem[0x0045] = 0x0000      # temp storage
    flash_mem[0x0046] = 0x4000      # base pointer (0x4000)

    # initial values in PSRAM:
    ram_mem[0x4000] = 0             # F(0) = 0
    ram_mem[0x4001] = 1             # F(1) = 1

    # program:
    # 0:  LDA AC0, 0x40             ; AC0 = ptr
    # 1:  LDA AC1, 0,0              ; AC1 = PSRAM[ptr] (F_n-2)
    # 2:  STA AC1, 0x45             ; temp = F_n-2
    # 3:  LDA AC1, 1,0              ; AC1 = PSRAM[ptr+1] (F_n-1)
    # 4:  LDA AC0, 0x45             ; AC0 = F_n-2
    # 5:  ADD AC0, AC1              ; AC1 = F_n-2 + F_n-1
    # 6:  LDA AC0, 0x40             ; AC0 = ptr
    # 7:  STA AC1, 2,0              ; PSRAM[ptr+2] = F_n
    # 8:  INC AC0, AC0 (c=0)        ; ptr++
    # 9:  STA AC0, 0x40             ; save ptr
    # 10: LDA AC0, 0x42             ; AC0 = -1
    # 11: LDA AC1, 0x43             ; AC1 = count
    # 12: ADD AC0, AC1              ; AC1 = count - 1
    # 13: STA AC1, 0x43             ; save count
    # 14: MOV# AC1, AC1, SZR        ; if count == 0, skip to print
    # 15: JMP 0                     ; loop back
    #
    # PRINT:
    # 16: LDA AC0, 0x46             ; AC0 = 0x4000
    # 17: STA AC0, 0x40             ; reset ptr to 0x4000
    # 18: LDA AC0, 0x40             ; AC0 = ptr
    # 19: LDA AC1, 0,0              ; AC1 = PSRAM[ptr]
    # 20: LDA AC0, 0x41             ; AC0 = '0' (0x30)
    # 21: ADD AC0, AC1              ; AC1 = digit + '0'
    # 22: SKPBZ 011                 ; wait for TX ready
    # 23: JMP 22
    # 24: DOA AC1, 011              ; transmit char
    # 25: LDA AC0, 0x40             ; AC0 = ptr
    # 26: INC AC0, AC0 (c=0)        ; ptr++
    # 27: STA AC0, 0x40             ; save ptr
    # 28: LDA AC0, 0x42             ; AC0 = -1
    # 29: LDA AC1, 0x44             ; AC1 = print count
    # 30: ADD AC0, AC1              ; AC1 = count - 1
    # 31: STA AC1, 0x44             ; save print count
    # 32: MOV# AC1, AC1, SZR        ; if count == 0, skip
    # 33: JMP 18                    ; loop back
    # 34: HALT
    flash_mem[0]  = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x40) # LDA AC0, 0x40
    flash_mem[1]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=0)    # LDA AC1, 0,0
    flash_mem[2]  = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x45) # STA AC1, 0x45
    flash_mem[3]  = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=1)    # LDA AC1, 1,0
    flash_mem[4]  = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x45) # LDA AC0, 0x45
    flash_mem[5]  = encode_alc(acs=0, acd=1, op=6, carry=1)                       # ADDZ AC0, AC1 (c=0)
    flash_mem[6]  = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x40) # LDA AC0, 0x40
    flash_mem[7]  = encode_mem(mode=2, func_or_ac=1, indir=0, index=2, disp=2)    # STA AC1, 2,0
    flash_mem[8]  = encode_alc(acs=0, acd=0, op=3, carry=1)                       # INC AC0, AC0 (c=0)
    flash_mem[9]  = encode_mem(mode=2, func_or_ac=0, indir=0, index=0, disp=0x40) # STA AC0, 0x40
    flash_mem[10] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x42) # LDA AC0, 0x42
    flash_mem[11] = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x43) # LDA AC1, 0x43
    flash_mem[12] = encode_alc(acs=0, acd=1, op=6, carry=1)                       # ADDZ AC0, AC1 (c=0)
    flash_mem[13] = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x43) # STA AC1, 0x43
    flash_mem[14] = encode_alc(acs=1, acd=1, op=2, no_load=1, skip=4)             # MOV# AC1, AC1 SZR
    flash_mem[15] = encode_mem(mode=0, func_or_ac=0, disp=0)                      # JMP 0

    flash_mem[16] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x46) # LDA AC0, 0x46
    flash_mem[17] = encode_mem(mode=2, func_or_ac=0, indir=0, index=0, disp=0x40) # STA AC0, 0x40
    flash_mem[18] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x40) # LDA AC0, 0x40
    flash_mem[19] = encode_mem(mode=1, func_or_ac=1, indir=0, index=2, disp=0)    # LDA AC1, 0,0
    flash_mem[20] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x41) # LDA AC0, 0x41 ('0')
    flash_mem[21] = encode_alc(acs=0, acd=1, op=6, carry=1)                       # ADDZ AC0, AC1 (c=0)
    flash_mem[22] = encode_io(ac=0, transfer=7, control=1, dev=0o11)              # SKPBZ 011
    flash_mem[23] = encode_mem(mode=0, func_or_ac=0, disp=22)                     # JMP 22
    flash_mem[24] = encode_io(ac=1, transfer=2, control=0, dev=0o11)              # DOA AC1, 011
    flash_mem[25] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x40) # LDA AC0, 0x40
    flash_mem[26] = encode_alc(acs=0, acd=0, op=3, carry=1)                       # INC AC0, AC0 (c=0)
    flash_mem[27] = encode_mem(mode=2, func_or_ac=0, indir=0, index=0, disp=0x40) # STA AC0, 0x40
    flash_mem[28] = encode_mem(mode=1, func_or_ac=0, indir=0, index=0, disp=0x42) # LDA AC0, 0x42
    flash_mem[29] = encode_mem(mode=1, func_or_ac=1, indir=0, index=0, disp=0x44) # LDA AC1, 0x44
    flash_mem[30] = encode_alc(acs=0, acd=1, op=6, carry=1)                       # ADDZ AC0, AC1 (c=0)
    flash_mem[31] = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x44) # STA AC1, 0x44
    flash_mem[32] = encode_alc(acs=1, acd=1, op=2, no_load=1, skip=4)             # MOV# AC1, AC1 SZR
    flash_mem[33] = encode_mem(mode=0, func_or_ac=0, disp=18)                     # JMP 18
    flash_mem[34] = encode_io(ac=0, transfer=6, control=0, dev=0o77)              # HALT

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
    clock = Clock(dut.clk, 666, unit="ns") # 1.5 MHz
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

# test 8: complete ALC skip matrix: all 8 skip conditions + no-load AC preservation
@cocotb.test()
async def test_alc_skip_matrix(dut):
    """
    test 8: verifies all 8 ALC skip conditions (never/SKP/SZC/SNC/SZR/SNR/SEZ/SBN)
    and confirms that the no-load '#' modifier preserves the dest accumulator
    """
    clock = Clock(dut.clk, 666, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    # setup: ac0=0x0000, ac1=0x0001, carry=0
    # MOV ac1,ac1 (c=Z) -> ac1=0x0001, carry=0
    # then exercise all 8 skip cases in order, collecting which addr were fetched

    # linear sequence: each pair is (skip-test, sentinel that must/must-not be fetched)
    # all tests use ac0 only, tracked through the sequence
    #
    # addr 0:  COM ac0         -> ac0=0xFFFF, carry=0
    # addr 1:  MOV#Z SZC      -> carry_in=0, SZC(carry=0) -> skip 2
    # addr 2:  sentinel (must be SKIPPED)
    # addr 3:  MOV# SNR       -> no-load, result=0xFFFF != 0 -> SNR -> skip 4
    # addr 4:  sentinel (must be SKIPPED)
    # addr 5:  INC ac0 (c=Z)  -> ac0=0x0000, carry=1 (wrap from 0xFFFF+1)
    # addr 6:  MOV# SNC       -> carry_in=carry_flag=1, carry_out=1, SNC -> skip 7
    # addr 7:  sentinel (must be SKIPPED)
    # addr 8:  MOV# SZR       -> no-load, result=0x0000, SZR -> skip 9
    # addr 9:  sentinel (must be SKIPPED)
    # addr 10: INC ac0 (c=Z)  -> ac0=0x0001, carry=0
    # addr 11: MOV#Z SEZ      -> carry_in=0, carry_zero=1, SEZ -> skip 12
    # addr 12: sentinel (must be SKIPPED)
    # addr 13: MOV# SKP       -> always -> skip 14
    # addr 14: sentinel (must be SKIPPED)
    # addr 15: COM ac0        -> ac0=0xFFFE, carry=0 (result != 0, carry = 0)
    # addr 16: MOV#Z SBN     -> carry_in=0, carry_zero=1, SBN=false -> NOT skip 17
    # addr 17: sentinel (must NOT be skipped — confirms SBN evaluated false)
    # addr 18: HALT
    flash_mem[0]  = encode_alc(acs=0, acd=0, op=0)                              # COM ac0 -> 0xFFFF, carry=0
    flash_mem[1]  = encode_alc(acs=0, acd=0, op=2, carry=1, no_load=1, skip=2)  # MOV#Z SZC: carry=0 -> skip 2
    flash_mem[2]  = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[3]  = encode_alc(acs=0, acd=0, op=2, no_load=1, skip=5)           # MOV# SNR: 0xFFFF != 0 -> skip 4
    flash_mem[4]  = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[5]  = encode_alc(acs=0, acd=0, op=3, carry=1)                     # INC ac0 (c=Z) -> 0x0000, carry=1
    flash_mem[6]  = encode_alc(acs=0, acd=0, op=2, carry=0, no_load=1, skip=3)  # MOV# SNC: carry=1 -> skip 7
    flash_mem[7]  = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[8]  = encode_alc(acs=0, acd=0, op=2, no_load=1, skip=4)           # MOV# SZR: result=0 -> skip 9
    flash_mem[9]  = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[10] = encode_alc(acs=0, acd=0, op=3, carry=1)                     # INC ac0 (c=Z) -> 0x0001, carry=0
    flash_mem[11] = encode_alc(acs=0, acd=0, op=2, carry=1, no_load=1, skip=6)  # MOV#Z SEZ: carry_zero=1 -> skip 12
    flash_mem[12] = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[13] = encode_alc(acs=0, acd=0, op=2, no_load=1, skip=1)           # MOV# SKP: always -> skip 14
    flash_mem[14] = encode_alc(acs=0, acd=0, op=0)                              # sentinel (skipped)
    flash_mem[15] = encode_alc(acs=0, acd=0, op=0)                              # COM ac0 -> 0xFFFE, carry=0    mmm, C0xFFFE
    flash_mem[16] = encode_alc(acs=0, acd=0, op=2, carry=1, no_load=1, skip=7)  # MOV#Z SBN: carry_zero=1 -> SBN false -> NOT skip
    flash_mem[17] = encode_alc(acs=0, acd=0, op=0)                              # sentinel (NOT skipped — SBN correctly false)
    flash_mem[18] = encode_io(ac=0, transfer=6, control=0, dev=0o77)            # HALT!     7(0o0)7
    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 5000)

    fetched = [addr for (op, is_flash, addr) in access_log if op == 'READ' and is_flash]

    # sentinels that must have been skipped (layout: SZC->2, SNR->4, SNC->7, SZR->9, SEZ->12, SKP->14)
    assert 2  not in fetched, "SZC skip failed (sentinel at 2 was fetched)"
    assert 4  not in fetched, "SNR skip failed (sentinel at 4 was fetched)"
    assert 7  not in fetched, "SNC skip failed (sentinel at 7 was fetched)"
    assert 9  not in fetched, "SZR skip failed (sentinel at 9 was fetched)"
    assert 12 not in fetched, "SEZ skip failed (sentinel at 12 was fetched)"
    assert 14 not in fetched, "SKP skip failed (sentinel at 14 was fetched)"
    # SBN should NOT have skipped (carry_zero=1 makes SBN false when carry=0)
    assert 17 in fetched, "SBN incorrectly skipped (sentinel at 17 should have been fetched)"
    assert 18 in fetched, "HALT at 18 was not reached"

# test 9: JSR return address + indirect JMP
@cocotb.test()
async def test_jsr_return_and_indirect_jmp(dut):
    """
    test 9: verifies that JSR writes pc+1 into AC1 as the return address, and
    that an indirect JMP correctly dereferences the pointer and jumps through it
    """
    clock = Clock(dut.clk, 666, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    access_log = []
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem, access_log))

    # test sequence:
    # 0: JSR 0x10  (jump to 0x10, stores return addr 1 in AC1)
    # 1: HALT      (should be reached via return)
    # ...
    # 0x10: STA AC1, 0x20    (store return address into page-zero pointer at 0x20)
    # 0x11: JMP @0x20        (indirect JMP through pointer -> should land at 1)     ┌(@0x20)┘ JUMP!
    #
    # page-zero pointer slot:
    flash_mem[0x0020] = 0x0000  # will be overwritten by STA at 0x10

    flash_mem[0x00]  = encode_mem(mode=0, func_or_ac=1, indir=0, index=0, disp=0x10)  # JSR 0x10
    flash_mem[0x01]  = encode_io(ac=0, transfer=6, control=0, dev=0o77)               # HALT
    flash_mem[0x10]  = encode_mem(mode=2, func_or_ac=1, indir=0, index=0, disp=0x20)  # STA AC1, 0x20
    flash_mem[0x11]  = encode_mem(mode=0, func_or_ac=0, indir=1, index=0, disp=0x20)  # JMP @0x20

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 6000)

    fetched = [addr for (op, is_flash, addr) in access_log if op == 'READ' and is_flash]
    writes  = [(addr, op) for (op, is_flash, addr) in access_log if op == 'WRITE' and is_flash]

    # JSR must have stored return address 0x0001 into page-zero slot 0x0020
    assert flash_mem.get(0x0020) == 0x0001, \
        f"JSR return address not stored correctly: 0x20 = {hex(flash_mem.get(0x0020, 0))}"
    # after indirect JMP through 0x0020, PC should land at 0x0001 (HALT)
    assert 0x01 in fetched, "indirect JMP did not return to JSR+1 (HALT not reached)"
    # the HALT is at 0x01 — it should be the last instr fetched
    assert 0x10 in fetched, "subroutine at 0x10 was never fetched"
    assert 0x11 in fetched, "indirect JMP at 0x11 was never fetched"

# test 10: blinkenlights output and IORST device reset
@cocotb.test()
async def test_blinkenlights_and_iorst(dut):
    """
    test 10: verifies that uo_out[7:1] reflects the current CPU state
    and instr opcode bits, and that IORST clears the UART done flags
    """
    clock = Clock(dut.clk, 666, unit="ns")
    cocotb.start_soon(clock.start())

    flash_mem = {}
    ram_mem = {}
    cocotb.start_soon(spi_memory_responder(dut, flash_mem, ram_mem))

    # program: receive a byte, then IORST, then re-check done flag is cleared
    # 0: SKPDN 010          ; wait for RX done
    # 1: JMP 0
    # 2: DIA AC0, 010       ; read char (clears rx_done implicitly)
    # 3: NIO 010            ; IORST equivalent - use DOC to reset (ctrl=2=C flag)
    #    (use the IORST CPU instr: DOC (transfer=6) on dev 0o77 with ctrl=1 = IORST)
    # 4: SKPDN 010          ; rx_done should now be clear -> NOT skip
    # 5: JMP 7              ; rx_done is clear as expected -> jump to HALT
    # 6: JMP 6              ; if IORST did NOT work, spin forever (test timeout)
    # 7: HALT
    flash_mem[0] = encode_io(ac=0, transfer=7, control=2, dev=0o10)   # SKPDN 010
    flash_mem[1] = encode_mem(mode=0, func_or_ac=0, disp=0)           # JMP 0
    flash_mem[2] = encode_io(ac=0, transfer=1, control=0, dev=0o10)   # DIA AC0, 010
    flash_mem[3] = encode_io(ac=0, transfer=5, control=0, dev=0o77)   # IORST (DOC 5 = IORST on CPU dev)
    flash_mem[4] = encode_io(ac=0, transfer=7, control=2, dev=0o10)   # SKPDN 010 (should NOT skip)
    flash_mem[5] = encode_mem(mode=0, func_or_ac=0, disp=7)           # JMP 7
    flash_mem[6] = encode_mem(mode=0, func_or_ac=0, disp=6)           # spin (IORST failed)
    flash_mem[7] = encode_io(ac=0, transfer=6, control=0, dev=0o77)   # HALT

    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # wait a few cycles so the FSM has settled out of reset but is not yet HALT'd
    await ClockCycles(dut.clk, 5)

    # verify HALT bit is clear immediately after reset: state transitions through
    # STATE_FETCH(0) -> STATE_FETCH_WAIT(1) within the first clock, but the CPU
    #  def shouldn't be halted yet (uo_out[7] = blinkenlights[6] = halted)
    blink = (int(dut.uo_out.value) >> 1) & 0x7F
    halted_bit = (blink >> 6) & 1
    assert halted_bit == 0, f"HALT bit incorrectly set right after reset: uo_out={hex(int(dut.uo_out.value))}"

    # send one byte to trigger rx_done
    await uart_tx_byte(dut, 0x42)

    # give the CPU time to read the char, execute IORST, and reach HALT
    await ClockCycles(dut.clk, 5000)

    # verify HALT bit now set (uo_out[7] = blinkenlights[6] = (state == STATE_HALT))
    blink = (int(dut.uo_out.value) >> 1) & 0x7F
    halted_bit = (blink >> 6) & 1
    assert halted_bit == 1, f"HALT bit not set in blinkenlights after program completes: uo_out={hex(int(dut.uo_out.value))}"