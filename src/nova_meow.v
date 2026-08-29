/*
 * Nova MEOW: Memory Extension to Outside World (1-bit SPI) =o.o=
 * Copyright (c) 2026 X4NTHA, x4ntha.com
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module nova_meow (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [14:0] addr,
    input  wire [15:0] data_in,
    output wire [15:0] data_out,
    input  wire        read_req,
    input  wire        write_req,
    output wire        busy,

    input  wire        spi_miso,
    output reg         spi_mosi,
    output reg         spi_sck,
    output wire        spi_cs0_n,
    output wire        spi_cs1_n
);

    // SPI transaction states (Mode 0: CPOL=0, CPHA=0, 5 MHz SCK @ 10 MHz clk)
    localparam S_IDLE     = 2'd0;
    localparam S_CLK_LOW  = 2'd1;
    localparam S_CLK_HIGH = 2'd2;
    localparam S_DONE     = 2'd3;

    reg [1:0]  state;
    reg [5:0]  bit_cnt;
    reg [15:0] rx_shift;
    reg        is_write;
    reg        cs_select; // 0 = Flash (CS0#), 1 = PSRAM (CS1#)

    assign data_out = rx_shift;
    assign busy = (state != S_IDLE) || read_req || write_req;

    // chip selects derived combinationally from cs_select + transaction active
    // CS stays asserted through S_DONE for proper SPI hold time
    wire cs_active = (state != S_IDLE);
    assign spi_cs0_n = (cs_active && !cs_select) ? 1'b0 : 1'b1;
    assign spi_cs1_n = (cs_active &&  cs_select) ? 1'b0 : 1'b1;

    // SPI frame: 8-bit command + 24-bit address + 16-bit data = 48 bits
    // bit_cnt counts down from 47 to 0, selecting directly from tx_word
    wire [7:0]  spi_cmd  = is_write ? 8'h02 : 8'h03;
    wire [23:0] byte_adr = {8'h00, addr, 1'b0};
    wire [47:0] tx_word  = {spi_cmd, byte_adr, data_in};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b0;
            state     <= S_IDLE;
            rx_shift  <= 16'd0;
            bit_cnt   <= 6'd0;
            is_write  <= 1'b0;
            cs_select <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    if (read_req || write_req) begin
                        is_write  <= write_req;
                        cs_select <= addr[14];
                        bit_cnt   <= 6'd47;
                        state     <= S_CLK_LOW;
                    end
                end

                // SCK low phase: drive MOSI bit from tx_word
                S_CLK_LOW: begin
                    spi_sck  <= 1'b0;
                    spi_mosi <= tx_word[bit_cnt];
                    state    <= S_CLK_HIGH;
                end

                // SCK high phase: sample MISO bit on rising edge
                S_CLK_HIGH: begin
                    spi_sck <= 1'b1;
                    if (!is_write)
                        rx_shift <= {rx_shift[14:0], spi_miso};

                    if (bit_cnt == 6'd0)
                        state <= S_DONE;
                    else begin
                        bit_cnt <= bit_cnt - 6'd1;
                        state   <= S_CLK_LOW;
                    end
                end

                // transaction complete: deassert chip selects via cs_active going low
                S_DONE: begin
                    spi_sck <= 1'b0;
                    state   <= S_IDLE;
                end
            endcase
        end
    end

endmodule
