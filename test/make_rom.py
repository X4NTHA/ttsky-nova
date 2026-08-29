#!/usr/bin/env python3
"""
Data General Nova mini-assembler for Tiny Tapeout
translates standard Nova assembly into a 32 KB big-endian binary (rom.bin)
Copyright (c) 2026 X4NTHA, x4ntha.com
SPDX-License-Identifier: Apache-2.0
"""
import re, sys

# default demo assembly program
ASM_SOURCE = """
; Nova interactive UART echo demo
START:  SKPDN 010        ; check if a character was received on UART RX (Dev 0o10)
        JMP   START      ; wait for character
        DIA   0, 010     ; read character into AC0 (clears RX done flag)
WAIT:   SKPBZ 011        ; wait until UART TX (Dev 0o11) transmitter is ready
        JMP   WAIT       ; wait
        DOA   0, 011     ; echo character back out over UART TX
        JMP   START      ; loop indefinitely for next character
"""

def assemble_nova(source_text):
    """
    assembles Nova assembly source text into a list of 16-bit words (16384 words / 32 KB)
    """
    lines = source_text.strip().split('\n')
    rom = [0] * 16384 # 16K words (32 KB)
    labels = {}
    cleaned = []
    pc = 0
    
    # pass 1: strip comments, extract labels, and map program counter
    for raw in lines:
        line = re.sub(r'[;#].*$', '', raw).strip()
        if not line: continue
        while ':' in line:
            lbl, line = line.split(':', 1)
            labels[lbl.strip()] = pc
            line = line.strip()
        if line:
            cleaned.append((pc, line))
            pc += 1
            
    # instruction lookup tables
    alc_ops = {'COM':0, 'NEG':1, 'MOV':2, 'INC':3, 'ADC':4, 'SUB':5, 'ADD':6, 'AND':7}
    alc_shifts = {'L':1, 'R':2, 'S':3}
    alc_carries = {'Z':1, 'O':2, 'C':3}
    alc_skips = {'SKP':1, 'SZC':2, 'SNC':3, 'SZR':4, 'SNR':5, 'SEZ':6, 'SBN':7}
    io_trans = {'NIO':0, 'DIA':1, 'DOA':2, 'DIB':3, 'DOB':4, 'DIC':5, 'DOC':6}
    io_skips = {'SKPBN':0, 'SKPBZ':1, 'SKPDN':2, 'SKPDZ':3}
    mrc_ops = {'JMP':(0,0), 'JSR':(0,1), 'ISZ':(0,2), 'DSZ':(0,3), 'LDA':(1,None), 'STA':(2,None)}
    
    # helper to resolve labels, dot (current PC), offsets, and numeric literals
    def resolve_val(s, curr_pc):
        s = s.strip()
        if s == '.': return curr_pc
        if s in labels: return labels[s]
        m = re.match(r'^([\w\.]+)\s*([\+\-])\s*(\d+)$', s)
        if m:
            sym, sign, val = m.groups()
            base = curr_pc if sym == '.' else labels.get(sym, int(sym, 0))
            return (base + int(val, 0)) if sign == '+' else (base - int(val, 0))
        return int(s, 0)

    # pass 2: encode instructions into 16-bit binary words
    for curr_pc, line in cleaned:
        # direct word data directive (.WORD <val>)
        if line.upper().startswith('.WORD'):
            val_str = line.split(None, 1)[1]
            rom[curr_pc] = resolve_val(val_str, curr_pc) & 0xFFFF
            continue
            
        tokens = [t.strip() for t in re.split(r'[\s,]+', line) if t.strip()]
        mnemonic = tokens[0].upper()
        args = tokens[1:]
        
        # special CPU control aliases
        if mnemonic == 'HALT':
            rom[curr_pc] = 0x633F # DOC 0, 077
            continue
        if mnemonic == 'IORST':
            rom[curr_pc] = 0x65BF # DICC 0, 077
            continue
            
        # I/O skip instructions (SKPBN, SKPBZ, SKPDN, SKPDZ)
        if mnemonic in io_skips:
            ctrl = io_skips[mnemonic]
            dev = int(args[0], 8) if args[0].startswith('0') else int(args[0])
            rom[curr_pc] = (0 << 0) | (3 << 1) | (0 << 3) | (7 << 5) | (ctrl << 8) | ((dev & 0x3F) << 10)
            continue
            
        # I/O data transfer instructions (DIA, DOA, NIO, etc.)
        if mnemonic in io_trans:
            trans = io_trans[mnemonic]
            ac = int(args[0].replace('AC', ''))
            dev = int(args[1], 8) if args[1].startswith('0') else int(args[1])
            rom[curr_pc] = (0 << 0) | (3 << 1) | ((ac & 3) << 3) | ((trans & 7) << 5) | (0 << 8) | ((dev & 0x3F) << 10)
            continue
            
        # memory reference class instructions (LDA, STA, JMP, JSR, ISZ, DSZ)
        base_mrc = None
        for k in mrc_ops:
            if mnemonic.startswith(k):
                base_mrc = k
                break
        if base_mrc:
            indir = 1 if ('@' in mnemonic or any('@' in a for a in args)) else 0
            clean_args = [a.replace('@', '').replace('AC', '') for a in args]
            mode, func = mrc_ops[base_mrc]
            if mode == 0:
                func_or_ac = func
                target = clean_args[0]
                index = int(clean_args[1]) if len(clean_args) > 1 else 0
            else:
                func_or_ac = int(clean_args[0])
                target = clean_args[1]
                index = int(clean_args[2]) if len(clean_args) > 2 else 0
                
            addr_val = resolve_val(target, curr_pc)
            disp = (addr_val - (curr_pc + 1)) if index == 1 else addr_val
            rom[curr_pc] = (0 << 0) | ((mode & 3) << 1) | ((func_or_ac & 3) << 3) | ((indir & 1) << 5) | ((index & 3) << 6) | ((disp & 0xFF) << 8)
            continue
            
        # arithmetic & logic class instructions (ALC)
        m = re.match(r'^(COM|NEG|MOV|INC|ADC|SUB|ADD|AND)([LRS]?)([ZOC]?)(#?)$', mnemonic)
        if m:
            op_str, sh_str, c_str, nl_str = m.groups()
            op = alc_ops[op_str]
            shift = alc_shifts.get(sh_str, 0)
            carry = alc_carries.get(c_str, 0)
            no_load = 1 if nl_str == '#' else 0
            clean_args = [a.replace('AC', '') for a in args]
            acs = int(clean_args[0])
            acd = int(clean_args[1])
            skip = alc_skips.get(clean_args[2].upper(), 0) if len(clean_args) > 2 else 0
            rom[curr_pc] = (1 << 0) | ((acs & 3) << 1) | ((acd & 3) << 3) | ((op & 7) << 5) | ((shift & 3) << 8) | ((carry & 3) << 10) | ((no_load & 1) << 12) | ((skip & 7) << 13)
            continue
            
        raise ValueError(f"Unknown instruction at line {curr_pc}: {line}")
        
    return rom

if __name__ == '__main__':
    source = ASM_SOURCE
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            source = f.read()
            
    # compile assembly to big-endian binary image
    rom = assemble_nova(source)
    with open("rom.bin", "wb") as f:
        for word in rom:
            f.write(bytes([(word >> 8) & 0xFF, word & 0xFF]))
    print("Compiled rom.bin (32 KB) successfully!")
