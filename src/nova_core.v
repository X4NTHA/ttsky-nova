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
    localparam STATE_FETCH         = 3'd0;
    localparam STATE_FETCH_WAIT    = 3'd1;
    localparam STATE_DECODE        = 3'd2;
    localparam STATE_EXEC_ALU      = 3'd3;
    localparam STATE_INDIR_WAIT    = 3'd4;
    localparam STATE_MEM_RD_WAIT   = 3'd5;
    localparam STATE_MEM_WR_WAIT   = 3'd6;
    localparam STATE_HALT          = 3'd7;

    reg [2:0]  state;
    reg [1:0]  exec_cycle;
    reg        ea_valid;

    // pre-decoded I/O device fields (shared comparators)
    wire is_kbd = (ir[15:10] == 6'o10);
    wire is_prt = (ir[15:10] == 6'o11);
    wire is_cpu = (ir[15:10] == 6'o77);

    // effective address calculation for memory reference class (MRC)
    // index: 00 = Page 0, 01 = PC-relative, 10 = AC0-indexed, 11 = AC1-indexed
    wire [14:0] rel_base = (ir[7:6] == 2'b01) ? pc :
                           (ir[7:6] == 2'b10) ? ac0[14:0] : ac1[14:0];

    wire [14:0] base_addr = (ir[7:6] == 2'b00) ? 15'd0 : rel_base;
    wire [6:0]  offset_hi = (ir[7:6] == 2'b00) ? 7'b0 : {7{ir[15]}};
    wire [14:0] calculated_ea = base_addr + {offset_hi, ir[15:8]};

    // memory interface (nova_meow)
    wire [14:0] active_ea = ea_valid ? ea : calculated_ea;
    wire [14:0] mem_addr = (state == STATE_FETCH || state == STATE_FETCH_WAIT) ? pc : active_ea;
    wire [15:0] mem_data_in;
    wire [15:0] mem_data_out = ac_dst_val;
    wire        meow_busy;

    // combinational strobes for memory requests
    wire mem_read_req = (state == STATE_FETCH) ||
                        (state == STATE_DECODE && !ir[0] && ir[2:1] != 2'b11 &&
                         (ir[5] && !ea_valid || ir[2:1] == 2'b01));

    wire mem_write_req = (state == STATE_DECODE && !ir[0] && ir[2:1] == 2'b10 && (!ir[5] || ea_valid));

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
        alu_in_b = ir[3] ? ac1[3:0] : ac0[3:0];
    end

    nova_alu alu_inst (
        .a_nib(alu_in_a),
        .b_nib(alu_in_b),
        .carry_in(carry_intermediate),
        .opcode(ir[7:5]),
        .is_first_cycle(exec_cycle == 2'b00),
        .result(alu_result),
        .carry_out(alu_carry_out)
    );

    // hardware UART (115200 baud @ 10 MHz: 10,000,000 / 115200 = 86.8 -> div 87)
    localparam BAUD_DIV = 7'd87;
    localparam HALF_BAUD = 7'd43;

    // UART RX: 2-stage synchronizer, mid-bit sampling, start-glitch rejection
    reg [1:0] rx_sync;
    wire      rx_pin = rx_sync[1];

    reg [6:0] rx_baud_cnt;
    reg [3:0] rx_bit_cnt;
    reg [7:0] rx_shift_reg;
    reg       rx_done_flag;
    wire      rx_busy = (rx_bit_cnt != 4'd0);

    // clear rx_done when CPU reads DIA from Keyboard, issues IORST, or c-clear
    wire is_io_decode = (state == STATE_DECODE && !ir[0] && ir[2:1] == 2'b11);
    wire is_iorst     = is_cpu && (ir[7:5] == 3'b101);

    wire io_clear_rx_done = is_io_decode && (is_iorst || (is_kbd && (ir[7:5] == 3'b001 || ir[9:8] == 2'b10)));
    wire io_clear_tx_done = is_io_decode && (is_iorst || (is_prt && ir[9:8] == 2'b10));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync      <= 2'b11;
            rx_baud_cnt  <= 7'd0;
            rx_bit_cnt   <= 4'd0;
            rx_shift_reg <= 8'd0;
            rx_done_flag <= 1'b0;
        end else begin
            // 2-stage input synchronizer
            rx_sync <= {rx_sync[0], uart_rx};

            // clear rx_done_flag if CPU reads DIA (Dev 0o10) or issues CLR pulse
            if (io_clear_rx_done) begin
                rx_done_flag <= 1'b0;
            end

            if (rx_bit_cnt == 4'd0) begin
                rx_baud_cnt <= 7'd0;
                if (rx_pin == 1'b0) begin
                    rx_bit_cnt <= 4'd1;
                end
            end else begin
                // baud counter
                if (rx_baud_cnt == BAUD_DIV - 7'd1) begin
                    rx_baud_cnt <= 7'd0;
                    rx_bit_cnt  <= (rx_bit_cnt == 4'd10) ? 4'd0 : (rx_bit_cnt + 4'd1);
                end else begin
                    rx_baud_cnt <= rx_baud_cnt + 7'd1;
                end

                // mid-bit sampling
                if (rx_baud_cnt == HALF_BAUD) begin
                    if (rx_bit_cnt == 4'd1) begin
                        // validate start bit (reject sub-half-baud glitches)
                        if (rx_pin == 1'b1) begin
                            rx_bit_cnt <= 4'd0;
                        end
                    end else if (rx_bit_cnt == 4'd10) begin
                        // stop bit verif and latching
                        rx_done_flag <= 1'b1;
                    end else begin
                        // sample 8 data bits LSB first (bits 2..9)
                        rx_shift_reg <= {rx_pin, rx_shift_reg[7:1]};
                    end
                end
            end
        end
    end

    // UART TX: 8N1 serial frame transmitter
    reg [6:0] tx_baud_cnt;
    reg [3:0] tx_bit_cnt;
    reg [9:0] tx_shift_reg;
    reg       tx_done_flag;

    wire tx_busy   = (tx_bit_cnt != 4'd0);
    assign uart_tx = tx_shift_reg[0];

    wire tx_start_req = is_io_decode && is_prt && (ir[7:5] == 3'b010);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_baud_cnt  <= 7'd0;
            tx_bit_cnt   <= 4'd0;
            tx_shift_reg <= 10'h3FF;
            tx_done_flag <= 1'b0;
        end else begin
            // clear tx_done_flag when CPU writes DOA or issues CLR pulse
            if (io_clear_tx_done) begin
                tx_done_flag <= 1'b0;
            end

            if (tx_bit_cnt == 4'd0) begin
                if (tx_start_req) begin
                    tx_shift_reg <= {1'b1, ac_dst_val[7:0], 1'b0};
                    tx_baud_cnt  <= 7'd0;
                    tx_bit_cnt   <= 4'd10;
                    tx_done_flag <= 1'b0;
                end
            end else begin
                if (tx_baud_cnt == BAUD_DIV - 7'd1) begin
                    tx_baud_cnt <= 7'd0;
                    if (tx_bit_cnt == 4'd1) begin
                        tx_done_flag <= 1'b1;
                        tx_bit_cnt   <= 4'd0;
                    end else begin
                        tx_shift_reg <= {1'b1, tx_shift_reg[9:1]};
                        tx_bit_cnt   <= tx_bit_cnt - 4'd1;
                    end
                end else begin
                    tx_baud_cnt <= tx_baud_cnt + 7'd1;
                end
            end
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
    wire [15:0] next_dest_ac = (exec_cycle == 2'd3) ?
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
            state               <= STATE_FETCH;
            pc                  <= 15'd0;
            ir                  <= 16'd0;
            ac0                 <= 16'd0;
            ac1                 <= 16'd0;
            carry_flag          <= 1'b0;
            carry_intermediate <= 1'b0;
            ea                  <= 15'd0;
            exec_cycle          <= 2'd0;
            ea_valid            <= 1'b0;
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
                        // arithmetic / logic class (ALC): enter 4-cycle nibble loop
                        state      <= STATE_EXEC_ALU;
                        exec_cycle <= 2'b00;

                        // setup initial carry-in based on c-field ir[11:10]
                        case (ir[11:10])
                            2'b00: carry_intermediate <= carry_flag;
                            2'b01: carry_intermediate <= 1'b0;
                            2'b10: carry_intermediate <= 1'b1;
                            2'b11: carry_intermediate <= ~carry_flag;
                        endcase
                    end else if (ir[2:1] == 2'b11) begin
                        // input / output class (I/O) using pre-decoded device signals
                        pc    <= (ir[7:5] == 3'b111 && io_skip) ? (pc + 15'd2) : (pc + 15'd1);
                        state <= (is_cpu && ir[7:5] == 3'b110) ? STATE_HALT : STATE_FETCH;

                        if (!is_cpu && ir[7:5] == 3'b001 && is_kbd) begin
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
                        end else if (ir[2:1] == 2'b01) begin
                            // LDA
                            state <= STATE_MEM_RD_WAIT;
                        end else if (ir[2:1] == 2'b10) begin
                            // STA
                            state <= STATE_MEM_WR_WAIT;
                        end else begin
                            // JMP / JSR (ir[2:1] == 2'b00)
                            pc    <= active_ea;
                            if (ir[3]) ac1 <= {1'b0, pc + 15'd1}; // JSR: return addr in ac1
                            state <= STATE_FETCH;
                        end
                    end
                end

                // ALC 4-cycle execution loop: nibble serial arithmetic
                STATE_EXEC_ALU: begin
                    carry_intermediate <= alu_carry_out;

                    if (exec_cycle == 2'd3) begin
                        carry_flag <= shifted_cout;
                        pc         <= alc_skip ? (pc + 15'd2) : (pc + 15'd1);
                        state      <= STATE_FETCH;
                    end else begin
                        exec_cycle <= exec_cycle + 2'd1;
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

                // memory read (LDA only) (RIP ISZ/DSZ)
                STATE_MEM_RD_WAIT: begin
                    if (!meow_busy) begin
                        if (ir[3])
                            ac1 <= mem_data_in;
                        else
                            ac0 <= mem_data_in;
                        pc    <= pc + 15'd1;
                        state <= STATE_FETCH;
                    end
                end

                // memory write (STA only)
                STATE_MEM_WR_WAIT: begin
                    if (!meow_busy) begin
                        pc    <= pc + 15'd1;
                        state <= STATE_FETCH;
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
