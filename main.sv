module regfile (
  input logic clk, write_reg,
  input logic [4:0] rs1, rs2, rs3, rd,
  input logic [31:0] wd, 
  output logic [31:0] d1, d2, d3
);
  reg [31:0] registers [31:0];
  assign d1 = registers[rs1];
  assign d2 = registers[rs2];
  assign d3 = registers[rs3];
  always @(posedge clk) if (write_reg) registers[rd] <= (|rd) ? wd : 32'b0;
endmodule


module ula (
    input logic [31:0] a,
    input logic [31:0] b,
    input logic [4:0] op,
    output logic [31:0] out,
    output logic zero
);
    localparam OP_ADD = 3'b000;
    localparam OP_SUB = 3'b001;
    localparam OP_AND = 3'b010;
    localparam OP_OR = 3'b011;
    localparam OP_SLT = 3'b100;
    always_comb begin
        out = 32'b0;

        case(op)
            OP_ADD: out <= a + b;
            OP_SUB: out <= a - b;
            OP_AND: out <= a & b;
            OP_OR: out <= a | b;
            OP_SLT: begin
                if($signed(a) < $signed(b)) out = 32'd1;
                else out = 32'd0;
            end
            default: out <= 32'd0;
        endcase
    end
    assign zero = (out==32'd0);
endmodule


module mux32_1(
	input logic [31:0] s0,
	input logic [31:0] s1,
	input logic c,
	output logic [31:0] out
);
	assign out = (c) ? s1 : s0;
endmodule


module mux32_2(
	input logic [31:0] s0,
	input logic [31:0] s1,
	input logic [31:0] s2,
	input logic [31:0] s3,
	input logic [1:0] c,
	output logic out
);
	always_comb begin
		case (c)
			2'b00: out <= s0;
			2'b01: out <= s1;
			2'b10: out <= s2;
			2'b11: out <= s3;
		endcase
	end
endmodule


module reg32(
	input logic clk,
	input logic [31:0] in,
	output logic [31:0] out
);
reg [31:0] storage;
assign out = storage;
always @(posedge clk) storage <= in;
endmodule


module imd_generator(
	input logic [31:0] instruction,
	output logic [31:0] imd
);
	always @ (*)
    case (instruction[6:0])
        7'b0000011,
        7'b0010011,
        7'b1100111: imd <= {{20{instruction[31]}}, instruction[31:20]};
        7'b0100011: imd <= {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        7'b1100011: imd <= {{20{instruction[31]}}, instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        7'b1101111: imd <= {{12{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};				
        default:
            imd <= 32'b0;
    endcase
endmodule


module instruction_reg(
	input logic we,
	input logic clk,
	input logic [31:0] in,
	output logic [31:0] out
);
	logic [31:0] data;
	assign out = data;
  always @(posedge clk) data <= (we) ? in : data;
endmodule

module ctrl_block();
endmodule



module main(
  input logic clk,
  output logic clk_div,
  // PROPÓSITO DE TESTE
  input logic write_reg, write_ir,
  input logic [4:0] rs3,
  input logic [31:0] wd, mem_out, 
  output logic [31:0] rs3_data, mem_data,
	output logic [31:0] instruct,
	output logic [31:0] data_a, data_b
);
  initial clk_div = 1'b1;
  always @(posedge clk) clk_div = ~clk_div;

	instruction_reg ir(
		.we(write_ir), .clk(clk), .in(mem_out),
		.out(instruct)
	);

	reg32 mem_dt_reg(
		.clk(clk), .in(mem_out),
		.out(mem_data)
	);
	
  regfile regs(
		.clk(clk), .write_reg(write_reg), .rs1(instruct[19:15]), .rs2(instruct[24:20]), .rs3(rs3), .rd(instruct[11:7]), .wd(reg_write_data),
		.d1(rs1_data), .d2(rs2_data), .d3(rs3_data)
	);

	reg32 d1_reg(
		.clk(clk), .in(rs1_data),
		.out(data_a)
	);

	reg32 d2_reg(
		.clk(clk), .in(rs2_data),
		.out(data_b)
	);

	mux32_2 reg_src_mux(		
		.s0(32'b0), .s1(32'b0), .s2(mem_data), .s3(wd), .c(reg_write_src),
		.out(reg_write_data)
	);

	// Parei por aqui

	mux32_2 ula_src_a(		
		.s0(), .s1(), .s2(), .s3(), .c(),
		.out()
	);

	mux32_2 ula_src_b(		
		.s0(), .s1(), .s2(), .s3(), .c(),
		.out()
	);

	ula ula_block(
    .a(), .b(), .op(),
    .out(), .zero()
	);

endmodule
