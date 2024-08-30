// Fractional Divider (performs division by a fractional value on average)
module fractional_divider(input_clk, output_clk, divider_integer, divider_select, divider_fraction, rst_n, rst_ddsm_n);
    input input_clk, rst_n, rst_ddsm_n;
    input [7:0] divider_integer;
    input [2:0] divider_select;
    input [15:0] divider_fraction;
    output output_clk;
    
    wire [3:0] ddsm_control;
    wire [7:0] sum_result;
    reg [7:0] ddsm_shifted;
    wire carry_out;
    
    integer_divider I1 (.input_clk(input_clk), .divider(sum_result), .select(divider_select), .div_clk(output_clk), .rst_n(rst_n));
    third_order_ddsm T1(divider_fraction, ddsm_control, output_clk, rst_ddsm_n);
    
    always @(ddsm_control) begin
        if (ddsm_control[3] == 1) begin
            ddsm_shifted = {1'b1, 1'b1, 1'b1, 1'b1, ddsm_control};   
        end
        else begin
            ddsm_shifted = {1'b0, 1'b0, 1'b0, 2'b0, ddsm_control};
        end
    end
    
    adder_8_bit A1(divider_integer, ddsm_shifted, carry_out, sum_result);
endmodule

// 3rd-Order Digital Delta-Sigma Modulator
module third_order_ddsm(input_frac, output_control, mod_clk, rst_n);
    input [15:0] input_frac;
    input mod_clk, rst_n;
    output signed [3:0] output_control;
    
    wire [15:0] acc_out1, acc_out2, acc_out3;
    wire [3:0] mod_control1, mod_control2, mod_control3;
    reg signed [3:0] mod_control2_d, mod_control3_d; 
    wire signed [3:0] mod_control_sum;
    
    accumulator_16_bit ACC1(input_frac, mod_clk, acc_out1, mod_control1, rst_n);
    accumulator_16_bit ACC2(acc_out1, mod_clk, acc_out2, mod_control2, rst_n);
    accumulator_16_bit ACC3(acc_out2, mod_clk, acc_out3, mod_control3, rst_n);
    
    always @(posedge mod_clk) begin
        if (rst_n == 1) begin
            mod_control2_d <= 0;
            mod_control3_d <= 0;
        end
        else begin
            mod_control2_d <= mod_control2;
            mod_control3_d <= mod_control_sum;
        end
    end
    
    assign mod_control_sum = mod_control1 + mod_control2 - mod_control2_d;
    assign output_control = mod_control1 + mod_control_sum - mod_control3_d;
endmodule

// Integer Divider with Cascading 8 2-3 Vaucher Divider
module integer_divider(input_clk, divider, select, output_clk, div_clk, rst_n);
    input input_clk, rst_n;
    input [7:0] divider;
    input [2:0] select;
    output output_clk, div_clk;
    
    wire [7:0] intermediate_t, control_bits, divider_output, intermediate_mn;
    
    genvar i, j;
    generate
        for (i = 0; i < 7; i = i + 1) begin: or_gates
            or g(intermediate_mn[i], divider_output[i+1], control_bits[i]);
        end
    endgenerate
    
    generate
        for (j = 1; j < 7; j = j + 1) begin: vaucher
            vaucher_2_3 v(intermediate_t[j], intermediate_mn[j], divider[j], divider_output[j], intermediate_t[j+1], rst_n);
        end
    endgenerate
    
    vaucher_2_3 V0(input_clk, intermediate_mn[0], divider[0], div_clk, intermediate_t[1], rst_n);
    vaucher_2_3 V1(intermediate_t[7], intermediate_mn[7], divider[7], divider_output[7], output_clk, rst_n);
    
    assign div_clk = divider_output[0];
    assign control_bits[0] = intermediate_mn[7];
    
    decoder3x8 DEC(select, control_bits);
endmodule

// 16-bit Accumulator
module accumulator_16_bit(input_value, clk, acc_out, control_out, rst_n);
    input [15:0] input_value;
    input clk, rst_n;
    output reg [15:0] acc_out;
    output [3:0] control_out;
    
    wire [15:0] sum_result;
    adder_16bit A1(input_value, acc_out, sum_result, control_out);
    
    always @(posedge clk) begin
        if (rst_n == 1) begin
            acc_out <= 0;
        end
        else begin
            acc_out <= sum_result;
        end
    end
endmodule

// 16-bit Full Adder
module adder_16bit(input1, input2, output_sum, carry_out);
    input [15:0] input1, input2;
    output [3:0] carry_out;
    output [15:0] output_sum;
    
    assign {carry_out, output_sum} = input1 + input2;
endmodule

// Divider that Can Switch Between 2 or 3 Dividing Ratios (Vaucher Divider)
module vaucher_2_3(input_clk, intermediate_min, divider_bit, mout, output_clk, rst_n);
    input input_clk, intermediate_min, divider_bit, rst_n;
    output output_clk, mout;
    
    reg q0, q1;
    wire t0, t1;
    
    xnor g1(t0, q0, q1);
    and g2(t1, mout, divider_bit);
    and g3(mout, q0, intermediate_min);
    
    assign output_clk = ~q0;
    
    always @(posedge input_clk) begin
        if (rst_n == 1) begin
            q0 <= 0;
            q1 <= 0;
        end
        else begin
            q0 <= t0;
            q1 <= t1;
        end
    end
endmodule

// 3x8 Divider
module decoder3x8(input_select, output_bits);
    input [2:0] input_select;
    output reg [7:0] output_bits;
    
    always @(*) begin
        case (input_select)
            0: output_bits = 1;
            1: output_bits = 2;
            2: output_bits = 4;
            3: output_bits = 8;
            4: output_bits = 16;
            5: output_bits = 32;
            6: output_bits = 64;
            7: output_bits = 128;
            default: output_bits = 1;
        endcase
    end
endmodule

// 8-bit Adder
module adder_8_bit(input_a, input_b, carry_out, sum_out);
    input [7:0] input_a, input_b;
    output carry_out;
    output [7:0] sum_out;
    
    wire [6:0] carry;
    
    FA f1(input_a[0], input_b[0], 1'b0, sum_out[0], carry[0]);
    FA f2(input_a[1], input_b[1], carry[0], sum_out[1], carry[1]);
    FA f3(input_a[2], input_b[2], carry[1], sum_out[2], carry[2]);
    FA f4(input_a[3], input_b[3], carry[2], sum_out[3], carry[3]);
    FA f5(input_a[4], input_b[4], carry[3], sum_out[4], carry[4]);
    FA f6(input_a[5], input_b[5], carry[4], sum_out[5], carry[5]);
    FA f7(input_a[6], input_b[6], carry[5], sum_out[6], carry[6]);
    FA f8(input_a[7], input_b[7], carry[6], sum_out[7], carry_out);
endmodule

// Full Adder
module FA(bit_a, bit_b, carry_in, sum_out, carry_out);
    input bit_a, bit_b, carry_in;
    output sum_out, carry_out;
    
    assign sum_out = bit_a ^ bit_b ^ carry_in;
    assign carry_out = (bit_a & bit_b) | (bit_a & carry_in) | (bit_b & carry_in);
endmodule
