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

    // operand selection for single shared arithmetic adder
    // ADD (110) & INC (011) use A; NEG (001), ADC (100), SUB (101) use ~A
    wire [3:0] op_a = opcode[1] ? a_nib : ~a_nib;

    // ADD (110), ADC (100), SUB (101) use B; INC (011) & NEG (001) use 1 on first cycle
    wire [3:0] op_b = opcode[2] ? b_nib : {3'b000, opcode[0] & is_first_cycle};

    wire [4:0] adder_sum = {1'b0, op_a} + {1'b0, op_b} + {4'b0, carry_in};

    always @(*) begin
        case (opcode)
            3'b000, 3'b010: begin // COM, MOV (result = op_a for both)
            result    = op_a;
            carry_out = carry_in;
        end
            3'b111: begin // AND
                result    = a_nib & b_nib;
                carry_out = carry_in;
            end
            default: begin // NEG, INC, ADC, SUB, ADD (shared adder)
                result    = adder_sum[3:0];
                carry_out = adder_sum[4];
            end
        endcase
    end

endmodule
