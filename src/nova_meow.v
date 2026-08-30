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

    assign data_out = rx_shift;
    assign busy = (state != S_IDLE) || read_req || write_req;

    // chip selects derived combinationally from addr[14] + transaction active
    // CS stays asserted through S_DONE for proper SPI hold time
    wire cs_active = (state != S_IDLE);
    assign spi_cs0_n = (cs_active && !addr[14]) ? 1'b0 : 1'b1;
    assign spi_cs1_n = (cs_active &&  addr[14]) ? 1'b0 : 1'b1;

    // MOSI output bit generation
    // frame decoded from bit_cnt[5:4]:
    //   bit_cnt[5:4] == 2'b10 (47..32):
    //     bit_cnt[3] == 1 (47..40): 8-bit command (8'h02 for write, 8'h03 for read)
    //     bit_cnt[3] == 0 (39..32): upper 8 address bits (always 8'h00)
    //   bit_cnt[5:4] == 2'b01 (31..16): 16-bit address {addr[14:0], 1'b0}
    //   bit_cnt[5:4] == 2'b00 (15..0) : 16-bit data_in
    reg mosi_bit;
    always @(*) begin
        case (bit_cnt[5:4])
            2'b10: begin // 47..32 (command and upper addrs)
                if (bit_cnt[3]) begin
                    // command: 8'h02 (00000010) or 8'h03 (00000011)
                    case (bit_cnt[2:0])
                        3'd1:    mosi_bit = 1'b1;
                        3'd0:    mosi_bit = !is_write;
                        default: mosi_bit = 1'b0;
                    endcase
                end else begin
                    mosi_bit = 1'b0; // upper 8 address bits are 0x00
                end
            end
            2'b01: begin // 31..16 ({addr[14:0], 1'b0})
                mosi_bit = (bit_cnt[3:0] == 4'd0) ? 1'b0 : addr[bit_cnt[3:0] - 4'd1];
            end
            default: begin // 15..0 (data_in)
                mosi_bit = data_in[bit_cnt[3:0]];
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b0;
            state     <= S_IDLE;
            rx_shift  <= 16'd0;
            bit_cnt   <= 6'd0;
            is_write  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    if (read_req || write_req) begin
                        is_write <= write_req;
                        bit_cnt  <= 6'd47;
                        state    <= S_CLK_LOW;
                    end
                end

                // SCK low phase: drive MOSI bit
                S_CLK_LOW: begin
                    spi_sck  <= 1'b0;
                    spi_mosi <= mosi_bit;
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
