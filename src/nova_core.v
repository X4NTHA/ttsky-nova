/*
 * Nova Core
 * Copyright (c) 2026 X4NTHA, x4ntha.com
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module nova_core (
    input  wire        clk,
    input  wire        rst_n,

    // UART
    input  wire        uart_rx,
    output wire        uart_tx,

    // blinkenlights (uo_out[7:1])
    output wire [6:0]  blinkenlights,

    // SPI memory interface
    input  wire        spi_miso,
    output wire        spi_mosi,
    output wire        spi_sck,
    output wire        spi_cs0_n,
    output wire        spi_cs1_n
);

    // architectural registers (2 accumulators: ac0, ac1) (RIP ac2, ac3)
    reg [15:0] ac0, ac1;
    reg [14:0] pc;
    reg [15:0] ir;
    reg        carry_flag;
    reg        carry_intermediate;
    reg [14:0] ea;

    // destination accumulator value selected by ir[3] (0=ac0, 1=ac1)
    wire [15:0] ac_dst_val = ir[3] ? ac1 : ac0;

    // core control FSM: major cycles and instruction execution
    localparam STATE_FETCH         = 4'd0;
    localparam STATE_FETCH_WAIT    = 4'd1;
    localparam STATE_DECODE        = 4'd2;
    localparam STATE_ALU_0         = 4'd3;
    localparam STATE_ALU_1         = 4'd4;
    localparam STATE_ALU_2         = 4'd5;
    localparam STATE_ALU_3         = 4'd6;
    localparam STATE_INDIR_WAIT    = 4'd7;
    localparam STATE_MEM_WAIT      = 4'd8;
    localparam STATE_HALT          = 4'd10;

    reg [3:0]  state;
    reg        ea_valid;

    // pre-decoded I/O device fields (shared comparators)
    wire is_io_group = (ir[15:11] == 5'b00100);
    wire is_kbd = is_io_group & ~ir[10];
    wire is_prt = is_io_group &  ir[10];
    wire is_cpu = (ir[15:10] == 6'o77);

    // effective address calculation for memory reference class (MRC)
    // index: 00 = Page 0, 01 = PC-relative, 10 = AC0-indexed, 11 = AC1-indexed
    reg [14:0] rel_base;
    always @(*) begin
        case (ir[7:6])
            2'b00:   rel_base = 15'd0;
            2'b01:   rel_base = pc;
            2'b10:   rel_base = ac0[14:0];
            default: rel_base = ac1[14:0];
        endcase
    end

    wire [14:0] base_addr = rel_base;
    wire [6:0]  offset_hi = (ir[7:6] == 2'b00) ? 7'b0 : {7{ir[15]}};
    wire [14:0] calculated_ea = base_addr + {offset_hi, ir[15:8]};
    wire [14:0] pc_plus_1     = pc + 15'd1;
    wire [14:0] pc_plus_2     = pc + 15'd2;

    // memory interface (nova_meow)
    wire [14:0] active_ea = ea_valid ? ea : calculated_ea;
    wire [14:0] mem_addr = (state == STATE_FETCH || state == STATE_FETCH_WAIT) ? pc : active_ea;
    wire [15:0] mem_data_in;
    wire [15:0] mem_data_out = ac_dst_val;
    wire        meow_busy;

    // decode-cycle helper: true only during STATE_DECODE for a non-ALC instruction (MRC or I/O class)
    wire in_decode_ref = (state == STATE_DECODE) && !ir[0];

    // combinational strobes for memory requests
    wire mem_read_req = (state == STATE_FETCH) ||
                        (in_decode_ref && ir[2:1] != 2'b11 &&
                         (ir[5] && !ea_valid || ir[2:1] == 2'b01));

    wire mem_write_req = in_decode_ref && (ir[2:1] == 2'b10) && (!ir[5] || ea_valid);

    nova_meow meow_inst (
        .clk(clk),
        .rst_n(rst_n),
        .addr(mem_addr),
        .data_in(mem_data_out),
        .data_out(mem_data_in),
        .read_req(mem_read_req),
        .write_req(mem_write_req),
        .busy(meow_busy),
        .spi_miso(spi_miso),
        .spi_mosi(spi_mosi),
        .spi_sck(spi_sck),
        .spi_cs0_n(spi_cs0_n),
        .spi_cs1_n(spi_cs1_n)
    );

    // ALU interface: nibble-serial 4-bit datapath
    reg  [3:0] alu_in_a;
    reg  [3:0] alu_in_b;
    wire [3:0] alu_result;
    wire       alu_carry_out;

    always @(*) begin
        alu_in_a = ir[1] ? ac1[3:0] : ac0[3:0];
        alu_in_b = ac_dst_val[3:0];
    end

    nova_alu alu_inst (
        .a_nib(alu_in_a),
        .b_nib(alu_in_b),
        .carry_in(carry_intermediate),
        .opcode(ir[7:5]),
        .is_first_cycle(state == STATE_ALU_0),
        .result(alu_result),
        .carry_out(alu_carry_out)
    );

    // hardware UART (19200 baud @ 1.5 MHz: 1,500,000 / 19200 = 78.125 -> div 78)
    localparam BAUD_DIV  = 7'd78;
    localparam HALF_BAUD = 7'd39;

    // UART RX: 3-stage digital synchronizer & majority noise filter
    reg [2:0] rx_sync_reg;
    wire      rx_pin = (rx_sync_reg[2] & rx_sync_reg[1]) |
                       (rx_sync_reg[2] & rx_sync_reg[0]) |
                       (rx_sync_reg[1] & rx_sync_reg[0]);

    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0] rx_state;
    reg [2:0] rx_bit_idx;
    reg [6:0] rx_baud_cnt;
    reg [7:0] rx_shift_reg;
    reg       rx_done_flag;
    wire      rx_busy = (rx_state != RX_IDLE);

    // clear rx_done when CPU reads DIA from Keyboard, issues IORST, or c-clear
    wire is_io_decode = in_decode_ref && (ir[2:1] == 2'b11);
    wire is_iorst     = is_cpu && (ir[7:5] == 3'b101);

    // shared subterms: the IORST-clears-everything base case, and the
    // "pulse" edge-clear condition
    wire io_base_clear  = is_io_decode && is_iorst;
    wire io_pulse_clear = (ir[7:5] != 3'b111) && (ir[9:8] == 2'b10);

    wire io_clear_rx_done = io_base_clear || (is_io_decode && is_kbd && (ir[7:5] == 3'b001 || io_pulse_clear));
    wire io_clear_tx_done = io_base_clear || (is_io_decode && is_prt && io_pulse_clear);

    // line must be stable HIGH for at least 4 cycles before falling edge
    reg [2:0]  rx_idle_high_cnt;

    // UART RX state machine, 8N1 receiver w/ glitch+crosstalk rejection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync_reg      <= 3'b111;
            rx_state         <= RX_IDLE;
            rx_bit_idx       <= 3'd0;
            rx_baud_cnt      <= 7'd0;
            rx_shift_reg     <= 8'h00;
            rx_done_flag     <= 1'b0;
            rx_idle_high_cnt <= 3'd0;
        end else begin
            rx_sync_reg <= {rx_sync_reg[1:0], uart_rx};

            if (io_clear_rx_done) begin
                rx_done_flag <= 1'b0;
            end

            // RX never clamped by tx_busy so terminal streaming and burst transmission operate without dropped chars
            case (rx_state)
                RX_IDLE: begin
                    rx_baud_cnt <= 7'd0;
                    rx_bit_idx  <= 3'd0;
                    if (rx_pin == 1'b1) begin
                        if (rx_idle_high_cnt < 3'd4)
                            rx_idle_high_cnt <= rx_idle_high_cnt + 3'd1;
                    end else begin
                        // falling edge: only accept if line was solidly high (>= 4 clocks)
                        if (rx_idle_high_cnt >= 3'd4) begin
                            rx_state         <= RX_START;
                            rx_idle_high_cnt <= 3'd0;
                        end
                    end
                end

                RX_START: begin
                    if (rx_baud_cnt == HALF_BAUD) begin
                        if (rx_pin == 1'b0) begin
                            rx_baud_cnt <= 7'd0;
                            rx_bit_idx  <= 3'd0;
                            rx_state    <= RX_DATA;
                        end else begin
                            // false start bit / line glitch: abort back to IDLE
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end

                RX_DATA: begin
                    if (rx_baud_cnt == BAUD_DIV - 7'd1) begin
                        rx_baud_cnt  <= 7'd0;
                        rx_shift_reg <= {rx_pin, rx_shift_reg[7:1]};
                        if (rx_bit_idx == 3'd7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit_idx <= rx_bit_idx + 3'd1;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end

                RX_STOP: begin
                    if (rx_baud_cnt == BAUD_DIV - 7'd1) begin
                        rx_baud_cnt  <= 7'd0;
                        rx_done_flag <= 1'b1;
                        rx_state     <= RX_IDLE;
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end
            endcase
        end
    end

    // UART TX: 8N1 serial frame transmitter
    localparam TX_IDLE  = 2'd0;
    localparam TX_START = 2'd1;
    localparam TX_DATA  = 2'd2;
    localparam TX_STOP  = 2'd3;

    reg [1:0] tx_state;
    reg [2:0] tx_bit_idx;
    reg [6:0] tx_baud_cnt;
    reg [7:0] tx_shift_reg;
    reg       tx_pin_out;
    reg       tx_done_flag;

    assign uart_tx = tx_pin_out;
    wire tx_busy   = (tx_state != TX_IDLE);

    wire tx_start_req = is_io_decode && is_prt && (ir[7:5] == 3'b010);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state     <= TX_IDLE;
            tx_bit_idx   <= 3'd0;
            tx_baud_cnt  <= 7'd0;
            tx_shift_reg <= 8'h00;
            tx_pin_out   <= 1'b1;
            tx_done_flag <= 1'b0;
        end else begin
            if (io_clear_tx_done) begin
                tx_done_flag <= 1'b0;
            end

            case (tx_state)
                TX_IDLE: begin
                    tx_pin_out  <= 1'b1;
                    tx_baud_cnt <= 7'd0;
                    if (tx_start_req) begin
                        tx_shift_reg <= ac_dst_val[7:0];
                        tx_pin_out   <= 1'b0; // start bit
                        tx_state     <= TX_START;
                        tx_done_flag <= 1'b0;
                    end
                end

                TX_START: begin
                    if (tx_baud_cnt == BAUD_DIV - 7'd1) begin
                        tx_baud_cnt  <= 7'd0;
                        tx_pin_out   <= tx_shift_reg[0];
                        tx_shift_reg <= {1'b1, tx_shift_reg[7:1]};
                        tx_bit_idx   <= 3'd0;
                        tx_state     <= TX_DATA;
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 7'd1;
                    end
                end

                TX_DATA: begin
                    if (tx_baud_cnt == BAUD_DIV - 7'd1) begin
                        tx_baud_cnt <= 7'd0;
                        if (tx_bit_idx == 3'd7) begin
                            tx_pin_out <= 1'b1; // stop bit
                            tx_state   <= TX_STOP;
                        end else begin
                            tx_pin_out   <= tx_shift_reg[0];
                            tx_shift_reg <= {1'b1, tx_shift_reg[7:1]};
                            tx_bit_idx   <= tx_bit_idx + 3'd1;
                        end
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 7'd1;
                    end
                end

                TX_STOP: begin
                    if (tx_baud_cnt == BAUD_DIV - 7'd1) begin
                        tx_baud_cnt  <= 7'd0;
                        tx_done_flag <= 1'b1;
                        tx_state     <= TX_IDLE;
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 7'd1;
                    end
                end
            endcase
        end
    end

    // ALC shifter logic (applied on cycle 3 once full 16-bit word is reassembled)
    wire [15:0] raw_res_word = {alu_result, ac_dst_val[15:4]};
    wire        raw_cout     = alu_carry_out;
    reg [15:0] shifted_res;
    always @(*) begin
        case (ir[9:8])
            2'b00: shifted_res = raw_res_word;                                // none
            2'b01: shifted_res = {raw_res_word[14:0], raw_cout};              // rotate left through carry
            2'b10: shifted_res = {raw_cout, raw_res_word[15:1]};              // rotate right through carry
            2'b11: shifted_res = {raw_res_word[7:0], raw_res_word[15:8]};    // swap upper and lower 8-bit bytes
        endcase
    end

    wire shifted_cout = (ir[9:8] == 2'b01) ? raw_res_word[15] :
                        (ir[9:8] == 2'b10) ? raw_res_word[0] : raw_cout;

    // ALC destination accumulator writeback value
    wire [15:0] next_dest_ac = (state == STATE_ALU_3) ?
                               (ir[12] ? {ac_dst_val[3:0], ac_dst_val[15:4]} : shifted_res) :
                               {alu_result, ac_dst_val[15:4]};

    // ALC skip condition evaluation (4-way base condition folded with ir[13] polarity)
    wire carry_zero  = !shifted_cout;
    wire result_zero = (shifted_res == 16'h0000);

    reg base_skip;
    always @(*) begin
        case (ir[15:14])
            2'b00: base_skip = 1'b0;                         // 000=never, 001=SKP
            2'b01: base_skip = carry_zero;                   // 010=SZC,   011=SNC
            2'b10: base_skip = result_zero;                  // 100=SZR,   101=SNR
            2'b11: base_skip = carry_zero | result_zero;     // 110=SEZ,   111=SBN
        endcase
    end
    wire alc_skip = base_skip ^ ir[13];

    // I/O device status and skip (2-way device condition folded with ir[8] polarity)
    wire io_dev_busy = is_kbd ? rx_busy : (is_prt ? tx_busy : 1'b0);
    wire io_dev_done = is_kbd ? rx_done_flag : (is_prt ? tx_done_flag : 1'b0);
    wire io_skip     = (ir[9] ? io_dev_done : io_dev_busy) ^ ir[8];

    // FSM main sequential logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= STATE_FETCH;
            pc         <= 15'd0;
            ac0        <= 16'd0;
            ac1        <= 16'd0;
            carry_flag <= 1'b0;
            ea_valid   <= 1'b0;
        end else begin
            case (state)
                // instruction fetch: request 16-bit instruction word from SPI
                STATE_FETCH: begin
                    ea_valid <= 1'b0;
                    state    <= STATE_FETCH_WAIT;
                end

                STATE_FETCH_WAIT: begin
                    if (!meow_busy) begin
                        ir    <= mem_data_in;
                        state <= STATE_DECODE;
                    end
                end

                // instruction decode & branching
                STATE_DECODE: begin
                    if (ir[0]) begin
                        // arithmetic / logic class (ALC): enter 4-cycle nibble sequence
                        state <= STATE_ALU_0;

                        // setup initial carry-in based on c-field ir[11:10]
                        case (ir[11:10])
                            2'b00: carry_intermediate <= carry_flag;
                            2'b01: carry_intermediate <= 1'b0;
                            2'b10: carry_intermediate <= 1'b1;
                            2'b11: carry_intermediate <= ~carry_flag;
                        endcase
                    end else if (ir[2:1] == 2'b11) begin
                        // input / output class (I/O) using pre-decoded device signals
                        pc    <= (ir[7:5] == 3'b111 && io_skip) ? pc_plus_2 : pc_plus_1;
                        state <= (is_cpu && ir[7:5] == 3'b110) ? STATE_HALT : STATE_FETCH;
                        if (ir[7:5] == 3'b001 && is_kbd) begin
                            // DIA: read byte from keeb shift reg
                            if (ir[3])
                                ac1 <= {8'h00, rx_shift_reg};
                            else
                                ac0 <= {8'h00, rx_shift_reg};
                        end
                    end else begin
                        // memory reference class (MRC): JMP, JSR, LDA, STA
                        if (ir[5] && !ea_valid) begin
                            // indirect deferral: fetch pointer word from memory
                            state <= STATE_INDIR_WAIT;
                        end else if (ir[2:1] == 2'b01 || ir[2:1] == 2'b10) begin
                            // LDA or STA: writeback gated by ir[2:1] in STATE_MEM_WAIT
                            state    <= STATE_MEM_WAIT;
                        end else begin
                            // JMP / JSR (ir[2:1] == 2'b00)
                            pc       <= active_ea;
                            if (ir[3]) ac1 <= {1'b0, pc_plus_1}; // JSR: return addr in ac1
                            ea_valid <= 1'b0;
                            state    <= STATE_FETCH;
                        end
                    end
                end

                // ALC 4-cycle execution sequence: nibble serial arithmetic
                STATE_ALU_0, STATE_ALU_1, STATE_ALU_2, STATE_ALU_3: begin
                    carry_intermediate <= alu_carry_out;
                    if (state == STATE_ALU_3) begin
                        carry_flag <= shifted_cout;
                        pc         <= alc_skip ? pc_plus_2 : pc_plus_1;
                        state      <= STATE_FETCH;
                    end else begin
                        state      <= state + 4'd1;
                    end

                    // dest AC receives ALU writeback, other AC rotates if source
                    if (ir[3]) begin
                        ac1 <= next_dest_ac;
                        if (!ir[1]) ac0 <= {ac0[3:0], ac0[15:4]};
                    end else begin
                        ac0 <= next_dest_ac;
                        if (ir[1]) ac1 <= {ac1[3:0], ac1[15:4]};
                    end
                end

                // indirect addressing deferral: dereference pointer, then re-enter decode
                STATE_INDIR_WAIT: begin
                    if (!meow_busy) begin
                        ea       <= mem_data_in[14:0];
                        ea_valid <= 1'b1;
                        state    <= STATE_DECODE;
                    end
                end

                // memory read/write (LDA/STA)
                STATE_MEM_WAIT: begin
                    if (!meow_busy) begin
                        if (ir[2:1] == 2'b01) begin // LDA only
                            if (ir[3])
                                ac1 <= mem_data_in;
                            else
                                ac0 <= mem_data_in;
                        end
                        pc       <= pc_plus_1;
                        ea_valid <= 1'b0;
                        state    <= STATE_FETCH;
                    end
                end

                // soft halt state: freeze until hard reset (rst_n = 0)
                STATE_HALT: begin
                    state <= STATE_HALT;
                end

                default: state <= STATE_FETCH;
            endcase
        end
    end

    // blinkenlights diagnostic outputs (uo_out[7:1])
    assign blinkenlights[2:0] = state[2:0];
    assign blinkenlights[5:3] = ir[3:1];
    assign blinkenlights[6]   = (state == STATE_HALT);

    wire _unused_ir_bits = &{ir[4], ir[2]};

endmodule
