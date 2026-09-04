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
    
    def parse_string_bytes(s):
        # extracts string content from quotes and processes escape seq
        m = re.search(r'"([^"\\]*(?:\\.[^"\\]*)*)"', s)
        if not m:
            return []
        raw_content = m.group(1)
        # unescape standard C escapes
        content = raw_content.encode('utf-8').decode('unicode_escape')
        # append null terminator
        byte_list = [ord(c) for c in content] + [0]
        # pack into 16-bit big-endian words
        words = []
        for i in range(0, len(byte_list), 2):
            b_hi = byte_list[i]
            b_lo = byte_list[i+1] if (i+1 < len(byte_list)) else 0
            words.append((b_hi << 8) | b_lo)
        return words

    def parse_asciz_words(s):
        # extracts string content from quotes and returns 1 char per 16-bit word + 0
        m = re.search(r'"([^"\\]*(?:\\.[^"\\]*)*)"', s)
        if not m: return []
        content = m.group(1).encode('utf-8').decode('unicode_escape')
        return [ord(c) for c in content] + [0]

    # pass 1: strip comments, extract labels, and map program counter
    for raw in lines:
        line = re.sub(r';.*$', '', raw).strip()
        if not line: continue
        while True:
            m_lbl = re.match(r'^([A-Za-z0-9_]+):\s*(.*)$', line)
            if m_lbl:
                labels[m_lbl.group(1)] = pc
                line = m_lbl.group(2).strip()
            else:
                break
        if line:
            cleaned.append((pc, line))
            if line.upper().startswith('.ASCIZ'):
                asciz_words = parse_asciz_words(line)
                pc += max(1, len(asciz_words))
            elif line.upper().startswith('.STRING') or line.upper().startswith('.TXT'):
                str_words = parse_string_bytes(line)
                pc += max(1, len(str_words))
            else:
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
        # direct 1-character-per-word ASCIZ directive (.ASCIZ "...")
        if line.upper().startswith('.ASCIZ'):
            words = parse_asciz_words(line)
            for idx, w in enumerate(words):
                rom[curr_pc + idx] = w & 0xFFFF
            continue

        # direct packed string directive (.STRING "..." or .TXT "...")
        if line.upper().startswith('.STRING') or line.upper().startswith('.TXT'):
            words = parse_string_bytes(line)
            for idx, w in enumerate(words):
                rom[curr_pc + idx] = w & 0xFFFF
            continue

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
            
        # I/O data transfer instructions (DIA, DOA, NIO, DIAS, DIAC, DOAS, DOAC, etc.)
        base_io = mnemonic[:3]
        suffix_io = mnemonic[3:] if len(mnemonic) > 3 else ''
        if base_io in io_trans and suffix_io in ('', 'S', 'C', 'P'):
            trans = io_trans[base_io]
            ctrl = {'':0, 'S':1, 'C':2, 'P':3}[suffix_io]
            ac = int(args[0].replace('AC', '')) if base_io != 'NIO' else 0
            dev = int(args[1], 8) if (len(args) > 1 and args[1].startswith('0')) else (int(args[1]) if len(args) > 1 else int(args[0], 8))
            rom[curr_pc] = (0 << 0) | (3 << 1) | ((ac & 3) << 3) | ((trans & 7) << 5) | ((ctrl & 3) << 8) | ((dev & 0x3F) << 10)
            continue
            
        # memory reference class instructions (LDA, STA, JMP, JSR, ISZ, DSZ)
        base_mrc = None
        for k in mrc_ops:
            if mnemonic.startswith(k):
                base_mrc = k
                break
        if base_mrc:
            indir = 1 if ('@' in mnemonic or any('@' in a for a in args)) else 0
            clean_args = [re.sub(r'^AC(?=[0-3]|$)', '', a.replace('@', ''), flags=re.I) for a in args]
            mode, func = mrc_ops[base_mrc]
            if mode == 0:
                func_or_ac = func
                target = clean_args[0]
            else:
                func_or_ac = int(clean_args[0])
                target = clean_args[1]

            explicit_index = len(clean_args) > (1 if mode == 0 else 2)
            if explicit_index:
                index = int(clean_args[1 if mode == 0 else 2])
            else:
                if target.startswith('.'):
                    index = 1
                else:
                    addr_val_temp = resolve_val(target, curr_pc)
                    if addr_val_temp < 256:
                        index = 0
                    elif -128 <= (addr_val_temp - curr_pc) <= 127:
                        index = 1
                    else:
                        raise ValueError(f"Target '{target}' (addr 0x{addr_val_temp:04X}) out of 8-bit displacement range at PC 0x{curr_pc:04X}. Use PC-relative or Page Zero indirect '@PTR'.")
                
            addr_val = resolve_val(target, curr_pc)
            disp = (addr_val - curr_pc) if index == 1 else addr_val
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
            clean_args = [re.sub(r'^AC(?=[0-3]|$)', '', a, flags=re.I) for a in args]
            acs = int(clean_args[0])
            acd = int(clean_args[1])
            skip = alc_skips.get(clean_args[2].upper(), 0) if len(clean_args) > 2 else 0
            rom[curr_pc] = (1 << 0) | ((acs & 3) << 1) | ((acd & 3) << 3) | ((op & 7) << 5) | ((shift & 3) << 8) | ((carry & 3) << 10) | ((no_load & 1) << 12) | ((skip & 7) << 13)
            continue
            
        raise ValueError(f"Unknown instruction at line {curr_pc}: {line}")
        
    return rom

if __name__ == '__main__':
    source = ASM_SOURCE
    out_path = "rom.bin"
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            source = f.read()
    if len(sys.argv) > 2:
        out_path = sys.argv[2]
            
    # compile assembly to big-endian binary image
    rom = assemble_nova(source)
    with open(out_path, "wb") as f:
        for word in rom:
            f.write(bytes([(word >> 8) & 0xFF, word & 0xFF]))
    print(f"Compiled {out_path} (32 KB) successfully!")
