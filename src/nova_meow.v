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
    reg [1:0]  phase;       // 0 = CMD+UpperAddr, 1 = Address, 2 = Data
    reg [3:0]  bit_idx;     // 4-bit nibble/word bit index (15 down to 0)
    reg [15:0] shift_reg;   // unified 16-bit shift transceiver for MOSI & MISO
    reg        is_write;

    assign data_out = shift_reg;
    assign busy = (state != S_IDLE) || read_req || write_req;

    // chip selects derived combinationally from addr[14] + transaction active
    // CS stays asserted through S_DONE for proper SPI hold time
    wire cs_active = (state != S_IDLE);
    assign spi_cs0_n = (cs_active && !addr[14]) ? 1'b0 : 1'b1;
    assign spi_cs1_n = (cs_active &&  addr[14]) ? 1'b0 : 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_sck   <= 1'b0;
            spi_mosi  <= 1'b0;
            state     <= S_IDLE;
            phase     <= 2'd0;
            bit_idx   <= 4'd0;
            shift_reg <= 16'd0;
            is_write  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    spi_sck <= 1'b0;
                    if (read_req || write_req) begin
                        is_write  <= write_req;
                        phase     <= 2'd0;
                        bit_idx   <= 4'd15;
                        // Phase 0: 8-bit command (0x02 write, 0x03 read) + 8-bit 0x00 upper address
                        shift_reg <= {6'b000000, 1'b1, !write_req, 8'h00};
                        state     <= S_CLK_LOW;
                    end
                end

                // SCK low phase: drive MOSI bit directly from shift register MSB
                S_CLK_LOW: begin
                    spi_sck  <= 1'b0;
                    spi_mosi <= shift_reg[15];
                    state    <= S_CLK_HIGH;
                end

                // SCK high phase: sample MISO bit into shift register LSB
                S_CLK_HIGH: begin
                    spi_sck   <= 1'b1;
                    shift_reg <= {shift_reg[14:0], spi_miso};

                    if (bit_idx == 4'd0) begin
                        if (phase == 2'd0) begin
                            // Advance to Phase 1: 16-bit word address {addr[14:0], 1'b0}
                            phase     <= 2'd1;
                            bit_idx   <= 4'd15;
                            shift_reg <= {addr[14:0], 1'b0};
                            state     <= S_CLK_LOW;
                        end else if (phase == 2'd1) begin
                            // Advance to Phase 2: 16-bit data
                            phase     <= 2'd2;
                            bit_idx   <= 4'd15;
                            shift_reg <= data_in;
                            state     <= S_CLK_LOW;
                        end else begin
                            // Phase 2 complete: transaction done
                            state <= S_DONE;
                        end
                    end else begin
                        bit_idx <= bit_idx - 4'd1;
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
