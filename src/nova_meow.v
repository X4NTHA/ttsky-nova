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
    output reg         busy,

    input  wire        spi_miso,
    output reg         spi_mosi,
    output reg         spi_sck,
    output reg         spi_cs0_n, // Flash (uio[0])
    output reg         spi_cs1_n  // PSRAM (uio[6])
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

    assign data_out = rx_shift;

    // standard SPI NOR flash / PSRAM commands: 0x03 (read), 0x02 (page write)
    wire [7:0]  spi_cmd  = is_write ? 8'h02 : 8'h03;
    // convert 15-bit Nova word address to 24-bit byte address (word * 2)
    wire [23:0] byte_adr = {8'h00, addr, 1'b0};

    // dynamic MOSI bit selection from transaction field based on bit_cnt
    reg next_mosi;
    wire [2:0] cmd_idx = bit_cnt[2:0];
    wire [4:0] adr_idx = bit_cnt[4:0] - 5'd16;
    always @(*) begin
        if (bit_cnt >= 6'd40) begin
            next_mosi = spi_cmd[cmd_idx];
        end else if (bit_cnt >= 6'd16) begin
            next_mosi = byte_adr[adr_idx];
        end else begin
            next_mosi = data_in[bit_cnt[3:0]];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_cs0_n <= 1'b1;
            spi_cs1_n <= 1'b1;
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b0;
            busy      <= 1'b0;
            state     <= S_IDLE;
            rx_shift  <= 16'd0;
            bit_cnt   <= 6'd0;
            is_write  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    if (read_req || write_req) begin
                        busy     <= 1'b1;
                        is_write <= write_req;

                        // partition: 0x0000..0x3FFF -> Flash (CS0#), 0x4000..0x7FFF -> PSRAM (CS1#)
                        if (!addr[14]) begin
                            spi_cs0_n <= 1'b0;
                            spi_cs1_n <= 1'b1;
                        end else begin
                            spi_cs0_n <= 1'b1;
                            spi_cs1_n <= 1'b0;
                        end

                        bit_cnt <= 6'd47;
                        state   <= S_CLK_LOW;
                    end else begin
                        spi_cs0_n <= 1'b1;
                        spi_cs1_n <= 1'b1;
                        busy      <= 1'b0;
                    end
                end

                // SCK low phase: drive next MOSI bit on falling edge
                S_CLK_LOW: begin
                    spi_sck  <= 1'b0;
                    spi_mosi <= next_mosi;
                    state    <= S_CLK_HIGH;
                end

                // SCK high phase: sample MISO bit on rising edge
                S_CLK_HIGH: begin
                    spi_sck <= 1'b1;
                    if (!is_write) begin
                        rx_shift <= {rx_shift[14:0], spi_miso};
                    end

                    if (bit_cnt == 6'd0) begin
                        state <= S_DONE;
                    end else begin
                        bit_cnt <= bit_cnt - 6'd1;
                        state   <= S_CLK_LOW;
                    end
                end

                // transaction complete: deassert chip selects and clear busy
                S_DONE: begin
                    spi_cs0_n <= 1'b1;
                    spi_cs1_n <= 1'b1;
                    spi_sck   <= 1'b0;
                    busy      <= 1'b0;
                    state     <= S_IDLE;
                end
            endcase
        end
    end

endmodule