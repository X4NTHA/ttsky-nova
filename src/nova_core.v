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

    // architectural registers/state
    reg [15:0] ac0, ac1, ac2, ac3;
    reg [14:0] pc;
    reg [15:0] ir;
    reg        carry_flag;
    reg        carry_intermediate;
    reg [14:0] ea;

    // helper mux for selected destination accumulator
    wire [15:0] ac_dst_val = (ir[4:3] == 2'b00) ? ac0 :
                             (ir[4:3] == 2'b01) ? ac1 :
                             (ir[4:3] == 2'b10) ? ac2 : ac3;

    // peripheral control signals
    reg        tx_start_req;
    reg        io_clear_rx_done;
    reg        io_clear_tx_done;
    reg [3:0]  state;
    reg [1:0]  exec_cycle;

    // memory interface (nova_meow)
    wire [14:0] mem_addr = (state == 4'd0 || state == 4'd1 || state == 4'd2) ? pc : ea;
    wire [15:0] mem_data_in;
    wire [15:0] isz_dsz_val = (ir[4:3] == 2'b10) ? (mem_data_in + 16'd1) : (mem_data_in - 16'd1);
    wire [15:0] mem_data_out = (ir[2:1] == 2'b10) ? ac_dst_val : isz_dsz_val;
    reg         mem_read_req;
    reg         mem_write_req;
    wire        meow_busy;

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

    // continuous combinational routing of LSB nibbles for serial loop
    // accumulators rotate right by 4 bits each cycle; LSB is always presented to ALU
    always @(*) begin
        case (ir[2:1]) // ACS (source accumulator)
            2'b00: alu_in_a = ac0[3:0];
            2'b01: alu_in_a = ac1[3:0];
            2'b10: alu_in_a = ac2[3:0];
            2'b11: alu_in_a = ac3[3:0];
        endcase
        case (ir[4:3]) // ACD (destination accumulator)
            2'b00: alu_in_b = ac0[3:0];
            2'b01: alu_in_b = ac1[3:0];
            2'b10: alu_in_b = ac2[3:0];
            2'b11: alu_in_b = ac3[3:0];
        endcase
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

    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0] rx_state;
    reg [6:0] rx_baud_cnt;
    reg [2:0] rx_bit_cnt;
    reg [7:0] rx_shift_reg;
    reg       rx_done_flag;
    wire      rx_busy = (rx_state != RX_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync      <= 2'b11;
            rx_state     <= RX_IDLE;
            rx_baud_cnt  <= 7'd0;
            rx_bit_cnt   <= 3'd0;
            rx_shift_reg <= 8'd0;
            rx_done_flag <= 1'b0;
        end else begin
            // 2-stage input synchronizer to eliminate metastability
            rx_sync <= {rx_sync[0], uart_rx};

            // clear rx_done_flag if CPU reads DIA (Dev 0o10) or issues CLR pulse
            if (io_clear_rx_done) begin
                rx_done_flag <= 1'b0;
            end

            case (rx_state)
                // wait for falling edge of start bit
                RX_IDLE: begin
                    rx_baud_cnt <= 7'd0;
                    rx_bit_cnt  <= 3'd0;
                    if (rx_pin == 1'b0) begin
                        rx_state <= RX_START;
                    end
                end

                // validate start bit at center of baud window (reject sub-half-baud glitches)
                RX_START: begin
                    if (rx_baud_cnt == HALF_BAUD) begin
                        if (rx_pin == 1'b0) begin
                            rx_baud_cnt <= 7'd0;
                            rx_state    <= RX_DATA;
                        end else begin
                            rx_state    <= RX_IDLE;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end

                // sample 8 data bits at center of each bit period (LSB first)
                RX_DATA: begin
                    if (rx_baud_cnt == BAUD_DIV - 7'd1) begin
                        rx_baud_cnt  <= 7'd0;
                        rx_shift_reg <= {rx_pin, rx_shift_reg[7:1]};
                        if (rx_bit_cnt == 3'd7) begin
                            rx_state <= RX_STOP;
                        end else begin
                            rx_bit_cnt <= rx_bit_cnt + 3'd1;
                        end
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end

                // stop bit verification & latching
                RX_STOP: begin
                    if (rx_baud_cnt == BAUD_DIV - 7'd1) begin
                        rx_done_flag <= 1'b1;
                        rx_state     <= RX_IDLE;
                    end else begin
                        rx_baud_cnt <= rx_baud_cnt + 7'd1;
                    end
                end
            endcase
        end
    end

    // UART TX: 8N1 serial frame transmitter (1 start bit, 8 data bits LSB first, 1 stop bit)
    localparam TX_IDLE = 1'b0;
    localparam TX_SEND = 1'b1;

    reg       tx_state;
    reg [6:0] tx_baud_cnt;
    reg [3:0] tx_bit_cnt;
    reg [8:0] tx_shift_reg;
    reg       tx_done_flag;
    reg       tx_out_bit;

    wire tx_busy = (tx_state != TX_IDLE);
    assign uart_tx = tx_out_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state     <= TX_IDLE;
            tx_baud_cnt  <= 7'd0;
            tx_bit_cnt   <= 4'd0;
            tx_shift_reg <= 9'h1FF;
            tx_done_flag <= 1'b0;
            tx_out_bit   <= 1'b1;
        end else begin
            // clear tx_done_flag when CPU writes DOA or issues CLR pulse
            if (io_clear_tx_done) begin
                tx_done_flag <= 1'b0;
            end

            case (tx_state)
                TX_IDLE: begin
                    tx_out_bit <= 1'b1;
                    if (tx_start_req) begin
                        // frame: stop(1) + data(8)
                        tx_shift_reg <= {1'b1, ac_dst_val[7:0]};
                        tx_baud_cnt  <= 7'd0;
                        tx_bit_cnt   <= 4'd10;
                        tx_done_flag <= 1'b0;
                        tx_out_bit   <= 1'b0; // assert start bit immediately
                        tx_state     <= TX_SEND;
                    end
                end

                // shift out bits at 115200 baud intervals
                TX_SEND: begin
                    if (tx_baud_cnt == BAUD_DIV - 7'd1) begin
                        tx_baud_cnt <= 7'd0;
                        if (tx_bit_cnt == 4'd1) begin
                            tx_out_bit   <= 1'b1;
                            tx_done_flag <= 1'b1;
                            tx_state     <= TX_IDLE;
                        end else begin
                            tx_out_bit   <= tx_shift_reg[0];
                            tx_shift_reg <= {1'b1, tx_shift_reg[8:1]};
                            tx_bit_cnt   <= tx_bit_cnt - 4'd1;
                        end
                    end else begin
                        tx_baud_cnt <= tx_baud_cnt + 7'd1;
                    end
                end
            endcase
        end
    end

    // core control FSM: major cycles and instruction execution
    localparam STATE_FETCH         = 4'd0;
    localparam STATE_FETCH_ACK     = 4'd1;
    localparam STATE_FETCH_DONE    = 4'd2;
    localparam STATE_DECODE        = 4'd3;
    localparam STATE_EXEC_ALU      = 4'd4;
    localparam STATE_INDIR_REQ     = 4'd5;
    localparam STATE_INDIR_ACK     = 4'd6;
    localparam STATE_INDIR_DONE    = 4'd7;
    localparam STATE_MEM_RD_REQ    = 4'd8;
    localparam STATE_MEM_RD_ACK    = 4'd9;
    localparam STATE_MEM_RD_DONE   = 4'd10;
    localparam STATE_MEM_WR_REQ    = 4'd11;
    localparam STATE_MEM_WR_ACK    = 4'd12;
    localparam STATE_MEM_WR_DONE   = 4'd13;
    localparam STATE_HALT          = 4'd14;

    // effective address calculation for memory reference class (MRC)
    // index: 00 = Page 0 (0..255), 01 = PC-relative (+/-128), 10 = AC2-indexed, 11 = AC3-indexed
    wire [14:0] base_addr = (ir[7:6] == 2'b00) ? 15'd0 :
                            (ir[7:6] == 2'b01) ? pc :
                            (ir[7:6] == 2'b10) ? ac2[14:0] : ac3[14:0];

    wire [14:0] disp_ext = (ir[7:6] == 2'b00) ? {7'b0, ir[15:8]} :
                                                {{7{ir[15]}}, ir[15:8]};

    wire [14:0] calculated_ea = base_addr + disp_ext;

    // ALC shifter logic (applied on cycle 3 once full 16-bit word is reassembled)
    wire [15:0] raw_res_word = {alu_result, ac_dst_val[15:4]};
    wire        raw_cout     = alu_carry_out;

    reg [15:0] shifted_res;
    reg        shifted_cout;

    always @(*) begin
        case (ir[9:8])
            2'b00: begin // none
                shifted_res  = raw_res_word;
                shifted_cout = raw_cout;
            end
            2'b01: begin // rotate left through carry
                shifted_res  = {raw_res_word[14:0], raw_cout};
                shifted_cout = raw_res_word[15];
            end
            2'b10: begin // rotate right through carry
                shifted_res  = {raw_cout, raw_res_word[15:1]};
                shifted_cout = raw_res_word[0];
            end
            2'b11: begin // swap upper and lower 8-bit bytes
                shifted_res  = {raw_res_word[7:0], raw_res_word[15:8]};
                shifted_cout = raw_cout;
            end
        endcase
    end

    // ALC skip condition evaluation (skip next instruction if condition is true)
    reg alc_skip;
    always @(*) begin
        case (ir[15:13])
            3'b000: alc_skip = 1'b0;                                          // never
            3'b001: alc_skip = 1'b1;                                          // SKP (always)
            3'b010: alc_skip = (shifted_cout == 1'b0);                        // SZC (skip zero carry)
            3'b011: alc_skip = (shifted_cout == 1'b1);                        // SNC (skip non-zero carry)
            3'b100: alc_skip = (shifted_res == 16'h0000);                     // SZR (skip zero result)
            3'b101: alc_skip = (shifted_res != 16'h0000);                     // SNR (skip non-zero result)
            3'b110: alc_skip = (shifted_cout == 1'b0 || shifted_res == 16'h0); // SEZ (skip either zero)
            3'b111: alc_skip = (shifted_cout == 1'b1 && shifted_res != 16'h0); // SBN (skip both non-zero)
        endcase
    end

    // IO device status and skip condition evaluation
    wire io_dev_busy = (ir[15:10] == 6'o10) ? rx_busy :
                       (ir[15:10] == 6'o11) ? tx_busy : 1'b0;

    wire io_dev_done = (ir[15:10] == 6'o10) ? rx_done_flag :
                       (ir[15:10] == 6'o11) ? tx_done_flag : 1'b0;

    reg io_skip;
    always @(*) begin
        case (ir[9:8])
            2'b00: io_skip = io_dev_busy;  // SKPBN (skip busy non-zero)
            2'b01: io_skip = !io_dev_busy; // SKPBZ (skip busy zero / ready)
            2'b10: io_skip = io_dev_done;  // SKPDN (skip done non-zero / complete)
            2'b11: io_skip = !io_dev_done; // SKPDZ (skip done zero)
        endcase
    end

    // FSM main sequential logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= STATE_FETCH;
            pc                  <= 15'd0;
            ir                  <= 16'd0;
            ac0                 <= 16'd0;
            ac1                 <= 16'd0;
            ac2                 <= 16'd0;
            ac3                 <= 16'd0;
            carry_flag          <= 1'b0;
            carry_intermediate <= 1'b0;
            ea                  <= 15'd0;
            exec_cycle          <= 2'd0;
            mem_read_req        <= 1'b0;
            mem_write_req       <= 1'b0;
            tx_start_req        <= 1'b0;
            io_clear_rx_done    <= 1'b0;
            io_clear_tx_done    <= 1'b0;
        end else begin
            // default single-cycle strobe pulses
            tx_start_req     <= 1'b0;
            io_clear_rx_done <= 1'b0;
            io_clear_tx_done <= 1'b0;

            case (state)
                // instruction fetch: request 16-bit instruction word from SPI
                STATE_FETCH: begin
                    if (!meow_busy) begin
                        mem_read_req <= 1'b1;
                        state        <= STATE_FETCH_ACK;
                    end
                end

                STATE_FETCH_ACK: begin
                    if (meow_busy) begin
                        mem_read_req <= 1'b0;
                        state        <= STATE_FETCH_DONE;
                    end
                end

                STATE_FETCH_DONE: begin
                    if (!meow_busy) begin
                        ir    <= mem_data_in;
                        state <= STATE_DECODE;
                    end
                end

                // instruction decode & branching
                STATE_DECODE: begin
                    if (ir[0] == 1'b1) begin
                        // arithmetic / logic class (ALC): enter 4-cycle nibble loop
                        state      <= STATE_EXEC_ALU;
                        exec_cycle <= 2'b00;

                        // setup initial carry-in based on c-field ir[11:10] (Z=0, O=1, C=~C)
                        case (ir[11:10])
                            2'b00: carry_intermediate <= carry_flag;
                            2'b01: carry_intermediate <= 1'b0;
                            2'b10: carry_intermediate <= 1'b1;
                            2'b11: carry_intermediate <= ~carry_flag;
                        endcase
                    end else if (ir[2:1] == 2'b11) begin
                        // input / output class (I/O)
                        if (ir[15:10] == 6'o77) begin
                            // CPU control (Dev 0o77)
                            if (ir[7:5] == 3'b110) begin // HALT (DOC 0, CPU)
                                pc    <= pc + 15'd1;
                                state <= STATE_HALT;
                            end else if (ir[7:5] == 3'b101) begin // IORST (DICC 0, CPU)
                                io_clear_rx_done <= 1'b1;
                                io_clear_tx_done <= 1'b1;
                                pc               <= pc + 15'd1;
                                state            <= STATE_FETCH;
                            end else begin
                                pc    <= pc + 15'd1;
                                state <= STATE_FETCH;
                            end
                        end else begin
                            // standard I/O device (Keyboard 0o10 / Printer 0o11)
                            if (ir[7:5] == 3'b111) begin
                                // SKP on device flag condition (SKPBN, SKPBZ, SKPDN, SKPDZ)
                                pc    <= io_skip ? (pc + 15'd2) : (pc + 15'd1);
                                state <= STATE_FETCH;
                            end else begin
                                // data transfer (DIA: RX -> AC, DOA: AC -> TX)
                                if (ir[7:5] == 3'b001 && ir[15:10] == 6'o10) begin
                                    // DIA from keyboard: load RX shift buffer into AC[7:0], zero extend upper 8b
                                    case (ir[4:3])
                                         2'b00: ac0 <= {8'h00, rx_shift_reg};
                                         2'b01: ac1 <= {8'h00, rx_shift_reg};
                                         2'b10: ac2 <= {8'h00, rx_shift_reg};
                                         2'b11: ac3 <= {8'h00, rx_shift_reg};
                                    endcase
                                    io_clear_rx_done <= 1'b1;
                                end else if (ir[7:5] == 3'b010 && ir[15:10] == 6'o11) begin
                                    // DOA to printer: send AC[7:0] to UART TX transmitter
                                    tx_start_req <= 1'b1;
                                end

                                // explicit control: clear done flag if c-option (bits 9:8 == 2'b10)
                                if (ir[9:8] == 2'b10) begin
                                    if (ir[15:10] == 6'o10) io_clear_rx_done <= 1'b1;
                                    if (ir[15:10] == 6'o11) io_clear_tx_done <= 1'b1;
                                end

                                pc    <= pc + 15'd1;
                                state <= STATE_FETCH;
                            end
                        end
                    end else begin
                        // memory reference class (MRC)
                        ea <= calculated_ea;
                        if (ir[5] == 1'b1) begin
                            // indirect deferral: fetch pointer word from memory
                            state <= STATE_INDIR_REQ;
                        end else if (ir[2:1] == 2'b01 || ir[4] == 1'b1) begin
                            // LDA (01), ISZ (00,10), DSZ (00,11)
                            state <= STATE_MEM_RD_REQ;
                        end else if (ir[2:1] == 2'b10) begin
                            // STA (10)
                            state <= STATE_MEM_WR_REQ;
                        end else begin
                            // JMP (00,00) / JSR (00,01)
                            pc    <= calculated_ea;
                            if (ir[3] == 1'b1) ac3 <= {1'b0, pc + 15'd1};
                            state <= STATE_FETCH;
                        end
                    end
                end

                // ALC 4-cycle execution loop: nibble serial arithmetic
                STATE_EXEC_ALU: begin
                    carry_intermediate <= alu_carry_out;

                    if (exec_cycle == 2'd3) begin
                        // final cycle: apply shifter, skip, and no-load writeback suppression
                        carry_flag <= shifted_cout;
                        pc         <= alc_skip ? (pc + 15'd2) : (pc + 15'd1);
                        state      <= STATE_FETCH;

                        ac0 <= (ir[4:3] == 2'b00 && !ir[12]) ? shifted_res : {ac0[3:0], ac0[15:4]};
                        ac1 <= (ir[4:3] == 2'b01 && !ir[12]) ? shifted_res : {ac1[3:0], ac1[15:4]};
                        ac2 <= (ir[4:3] == 2'b10 && !ir[12]) ? shifted_res : {ac2[3:0], ac2[15:4]};
                        ac3 <= (ir[4:3] == 2'b11 && !ir[12]) ? shifted_res : {ac3[3:0], ac3[15:4]};
                    end else begin
                        // cycles 0..2: rotate 4-bit nibble into destination AC, rotate other ACs
                        ac0 <= (ir[4:3] == 2'b00) ? {alu_result, ac0[15:4]} : {ac0[3:0], ac0[15:4]};
                        ac1 <= (ir[4:3] == 2'b01) ? {alu_result, ac1[15:4]} : {ac1[3:0], ac1[15:4]};
                        ac2 <= (ir[4:3] == 2'b10) ? {alu_result, ac2[15:4]} : {ac2[3:0], ac2[15:4]};
                        ac3 <= (ir[4:3] == 2'b11) ? {alu_result, ac3[15:4]} : {ac3[3:0], ac3[15:4]};

                        exec_cycle <= exec_cycle + 2'd1;
                    end
                end

                // indirect addressing deferral: dereference pointer from memory
                STATE_INDIR_REQ: begin
                    if (!meow_busy) begin
                        mem_read_req <= 1'b1;
                        state        <= STATE_INDIR_ACK;
                    end
                end

                STATE_INDIR_ACK: begin
                    if (meow_busy) begin
                        mem_read_req <= 1'b0;
                        state        <= STATE_INDIR_DONE;
                    end
                end

                STATE_INDIR_DONE: begin
                    if (!meow_busy) begin
                        ea <= mem_data_in[14:0];
                        // route deferred target operation with resolved address
                        if (ir[2:1] == 2'b01 || ir[4] == 1'b1) begin
                            // LDA (01), ISZ (00,10), DSZ (00,11)
                            state <= STATE_MEM_RD_REQ;
                        end else if (ir[2:1] == 2'b10) begin
                            // STA (10)
                            state <= STATE_MEM_WR_REQ;
                        end else begin
                            // JMP (00,00) / JSR (00,01)
                            pc    <= mem_data_in[14:0];
                            if (ir[3] == 1'b1) ac3 <= {1'b0, pc + 15'd1};
                            state <= STATE_FETCH;
                        end
                    end
                end

                // memory read (LDA / ISZ / DSZ)
                STATE_MEM_RD_REQ: begin
                    if (!meow_busy) begin
                        mem_read_req <= 1'b1;
                        state        <= STATE_MEM_RD_ACK;
                    end
                end

                STATE_MEM_RD_ACK: begin
                    if (meow_busy) begin
                        mem_read_req <= 1'b0;
                        state        <= STATE_MEM_RD_DONE;
                    end
                end

                STATE_MEM_RD_DONE: begin
                    if (!meow_busy) begin
                        if (ir[2:1] == 2'b01) begin
                            // LDA: load memory word into destination accumulator
                            case (ir[4:3])
                                2'b00: ac0 <= mem_data_in;
                                2'b01: ac1 <= mem_data_in;
                                2'b10: ac2 <= mem_data_in;
                                2'b11: ac3 <= mem_data_in;
                            endcase
                            pc    <= pc + 15'd1;
                            state <= STATE_FETCH;
                        end else if (ir[4:3] == 2'b10 || ir[4:3] == 2'b11) begin
                            // ISZ / DSZ: write modified word back to memory
                            state <= STATE_MEM_WR_REQ;
                        end
                    end
                end

                // memory write (STA / ISZ / DSZ)
                STATE_MEM_WR_REQ: begin
                    if (!meow_busy) begin
                        mem_write_req <= 1'b1;
                        state         <= STATE_MEM_WR_ACK;
                    end
                end

                STATE_MEM_WR_ACK: begin
                    if (meow_busy) begin
                        mem_write_req <= 1'b0;
                        state         <= STATE_MEM_WR_DONE;
                    end
                end

                STATE_MEM_WR_DONE: begin
                    if (!meow_busy) begin
                        if (ir[2:1] == 2'b10) begin
                            // STA: write completed
                            pc    <= pc + 15'd1;
                            state <= STATE_FETCH;
                        end else if (ir[4:3] == 2'b10 || ir[4:3] == 2'b11) begin
                            // ISZ / DSZ: skip next word if modified result is zero
                            pc    <= (isz_dsz_val == 16'h0000) ? (pc + 15'd2) : (pc + 15'd1);
                            state <= STATE_FETCH;
                        end
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

endmodule
