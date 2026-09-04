/*
 * Copyright (c) 2026 X4NTHA, x4ntha.com
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_x4ntha_nova (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    wire spi_cs0_n;
    wire spi_cs1_n;
    wire spi_sck;
    wire spi_mosi;
    wire spi_miso;
    wire uart_tx;
    wire [6:0] blinkenlights;

    // Dedicated Outputs
    assign uo_out[0]   = uart_tx;
    assign uo_out[7:1] = blinkenlights;

    // tri-state all bidirectional pins while in reset so the demoboard RP2354B
    // (or an external flasher) can freely program the QSPI PMOD without bus contention
    wire bus_oe = rst_n;

    // Bidirectional IO mapping (Tiny Tapeout QSPI Pmod)
    // uio[0]: Flash Chip Select (/CS0, Active Low)
    assign uio_out[0] = spi_cs0_n;
    assign uio_oe[0]  = bus_oe;

    // uio[1]: Flash/PSRAM MOSI (SIO0)
    assign uio_out[1] = spi_mosi;
    assign uio_oe[1]  = bus_oe;

    // uio[2]: Flash/PSRAM MISO (SIO1, Input)
    assign spi_miso   = uio_in[2];
    assign uio_out[2] = 1'b0;
    assign uio_oe[2]  = 1'b0;

    // uio[3]: SPI SCK (Serial Clock)
    assign uio_out[3] = spi_sck;
    assign uio_oe[3]  = bus_oe;

    // uio[4]: SIO2 / WP# (Driven HIGH to disable write-protect in 1-bit SPI mode)
    assign uio_out[4] = 1'b1;
    assign uio_oe[4]  = bus_oe;

    // uio[5]: SIO3 / HOLD# (Driven HIGH to disable hold in 1-bit SPI mode)
    assign uio_out[5] = 1'b1;
    assign uio_oe[5]  = bus_oe;

    // uio[6]: PSRAM Chip Select (/CS1, Active Low)
    assign uio_out[6] = spi_cs1_n;
    assign uio_oe[6]  = bus_oe;

    // uio[7]: Unused Pmod Pin (tri-stated / Inactive)
    assign uio_out[7] = 1'b0;
    assign uio_oe[7]  = 1'b0;

    // Instantiate CPU Core
    nova_core cpu_core (
        .clk(clk),
        .rst_n(rst_n),
        .uart_rx(ui_in[0]),
        .uart_tx(uart_tx),
        .blinkenlights(blinkenlights),
        .spi_miso(spi_miso),
        .spi_mosi(spi_mosi),
        .spi_sck(spi_sck),
        .spi_cs0_n(spi_cs0_n),
        .spi_cs1_n(spi_cs1_n)
    );

    // Sink unused input nets
    wire _unused = &{ena, ui_in[7:1], uio_in[7:3], uio_in[1:0]};

endmodule
