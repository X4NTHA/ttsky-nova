/*
 * Nova ALU
 * Copyright (c) 2026 X4NTHA, x4ntha.com
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module nova_alu (
    input  wire [3:0] a_nib,
    input  wire [3:0] b_nib,
    input  wire       carry_in,
    input  wire [2:0] opcode,
    input  wire       is_first_cycle,
    output reg  [3:0] result,
    output reg        carry_out
);

    always @(*) begin
        case (opcode)
            3'b000: begin // COM
                result = ~a_nib;
                carry_out = carry_in;
            end
            3'b001: begin // NEG
                {carry_out, result} = {1'b0, ~a_nib} + {4'b0, carry_in};
            end
            3'b010: begin // MOV
                result = a_nib;
                carry_out = carry_in;
            end
            3'b011: begin // INC
                {carry_out, result} = {1'b0, a_nib} + (is_first_cycle ? 5'd1 : 5'd0) + {4'b0, carry_in};
            end
            3'b100: begin // ADC
                {carry_out, result} = {1'b0, ~a_nib} + {1'b0, b_nib} + {4'b0, carry_in};
            end
            3'b101: begin // SUB
                {carry_out, result} = {1'b0, b_nib} + {1'b0, ~a_nib} + {4'b0, carry_in};
            end
            3'b110: begin // ADD
                {carry_out, result} = {1'b0, a_nib} + {1'b0, b_nib} + {4'b0, carry_in};
            end
            3'b111: begin // AND
                result = a_nib & b_nib;
                carry_out = carry_in;
            end
            default: begin // default returns zero
                result = 4'b0000;
                carry_out = 1'b0;
            end
        endcase
    end

endmodule
