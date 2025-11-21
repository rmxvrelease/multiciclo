module regfile (
  input logic clk, write_reg,
  input logic [4:0] rs1, rs2, rs3, rd,
  input logic [31:0] wd, 
  output logic [31:0] d1,
	output logic [31:0] d2,
	output logic [31:0] d3
);
  reg [31:0] registers [31:0];
  assign d1 = registers[rs1];
  assign d2 = registers[rs2];
  assign d3 = registers[rs3];
  always @(posedge clk) if (write_reg) begin registers[rd][31:0] <= (|rd) ? wd : 32'b0; end
endmodule


module ula (
    input logic [31:0] a,
    input logic [31:0] b,
    input logic [4:0] op,
    output logic [31:0] out,
    output logic zero
);
    localparam OP_ADD = 5'b000;
    localparam OP_SUB = 5'b001;
    localparam OP_AND = 5'b010;
    localparam OP_OR = 5'b011;
    localparam OP_SLT = 5'b100;
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
	output logic [31:0] out
);
	always_comb begin
		case (c)
			2'b00: out <= s0;
			2'b01: out <= s1;
			2'b10: out <= s2;
			default: out <= s3;
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


module pc_write_control_circuit(
	input logic zero, write_cond, write,
	output logic out
);
	assign out = (zero & write_cond) | write;
endmodule


module ctrl_ula_r(
	input logic [6:0] funct7,
	input logic [2:0] funct3,
	output logic [4:0] out
);
	always_comb begin
		case (funct3)
			3'b000: out <= (funct7[1]) ? 5'b01 : 5'b00;
			3'b111: out <= 5'b010;
			3'b110: out <= 5'b011;
			3'b010: out <= 5'b100;
			default: out <= 5'b001;
		endcase
	end
endmodule


module control_unit(
	input logic clk,
	input logic [31:0] instruction,

	output logic pc_write_cond,
	output logic pc_write,
	output logic write_ir,
	output logic pc_src,
	output logic [4:0] ula_op,
	output logic [1:0] ula_src_a,
	output logic [1:0] ula_src_b,
	output logic write_pc_bkp,
	output logic write_reg,
	output logic [1:0] reg_write_src,
	output logic i_ou_d,
	output logic read_mem,
	output logic write_mem
);
	wire [2:0] funct3;
	wire [6:0] funct7;
	wire [6:0] opcode;
	wire [4:0] ula_op_r;
	assign funct3 = instruction[14:12];
	assign funct7 = instruction[31:25];
	assign opcode = instruction[6:0];

	reg [2:0] stage;

	ctrl_ula_r ctrl_ula_r_1(		
	.funct7(funct7),
	.funct3(funct3),
	.out(ula_op_r)
	);
	always @(posedge clk) begin
		case (stage)
			3'b000: begin // estagio 1a
				i_ou_d <= 0;
				read_mem <= 1;
				write_ir <= 1;
				pc_write <= 0;
				ula_src_a <= 2'b10; // pc
				ula_src_b <= 2'b01; // 4
				write_pc_bkp <= 1;
				ula_op <= 5'b0;
				pc_src <= 0;
				// remaining
				write_reg <= 0;
				reg_write_src <= 0;
				pc_write_cond <= 0;
				write_mem <= 0;
				stage <= stage + 1'b1;
			end
			3'b001: begin // estagio 1b
				i_ou_d <= 0;
				read_mem <= 1;
				write_ir <= 1;
				pc_write <= 1; // escreve pc
				pc_src <= 0;
				ula_src_a <= 2'b10; // pc
				ula_src_b <= 2'b01; // 4
				ula_op <= 5'b0;
				write_pc_bkp <= 0;
				// remaining
				write_reg <= 0;
				reg_write_src <= 0;
				pc_write_cond <= 0;
				write_mem <= 0;
				stage <= stage + 1'b1;
			end
			3'b010: begin // estagio 2
				ula_src_a <= 2'b00; // 
				ula_src_b <= 2'b10;
				// remaining
				i_ou_d <= 0;
				read_mem <= 0;
				write_ir <= 0;
				pc_write <= 0;
				write_pc_bkp <= 0;
				ula_op <= 5'b0;
				pc_src <= 0;
				write_reg <= 0;
				reg_write_src <= 0;
				pc_write_cond <= 0;
				write_mem <= 0;
				stage <= stage + 1'b1;
			end
			3'b011: begin // estagio 3
					case (opcode)
						7'b0110011: begin // tipo r
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b00;
							ula_op <= ula_op_r;
							// remaining
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0; // escreve pc
							pc_src <= 0;
							write_pc_bkp <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= stage + 1'b1;
						end
						7'b0010011: begin // addi
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b10;
							ula_op <= 5'b0;
							// remaining
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0; // escreve pc
							pc_src <= 0;
							write_pc_bkp <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= stage + 1'b1;
						end
						7'b0110111: begin // lui
							write_reg <= 1;
							reg_write_src <= 2'b11;
							// remaining
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b10;
							ula_op <= 5'b0;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0; // escreve pc
							pc_src <= 0;
							write_pc_bkp <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= 0;
						end
						7'b0010011: begin // beq
							pc_src <= 1;
							ula_src_a <= 2'b1;
							ula_src_b <= 2'b0;
							ula_op <= 5'b1;
							pc_write_cond <= 1;
							// remaining
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0; // escreve pc
							write_pc_bkp <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							write_mem <= 0;
							stage <= 0;
						end
						7'b1101111: begin // desvio incondicional jal
							write_reg <= 1;
							reg_write_src <= 2'b1;
							pc_src <= 1;
							pc_write <= 1;
							// remaining							
							ula_src_a <= 2'b0;
							ula_src_b <= 2'b0;
							ula_op <= 5'b0;
							pc_write_cond <= 0;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							write_pc_bkp <= 0;
							write_mem <= 0;
							stage <= 0;
						end
						7'b1100111: begin // desvio incondivional jalr
							write_reg <= 1;
							reg_write_src <= 2'b1;
							ula_src_a <= 2'b1;
							ula_src_b <= 2'b10;
							pc_src <= 0;
							pc_write <= 1;
							ula_op <= 5'b0;
							// remaining
							pc_write_cond <= 0;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							write_pc_bkp <= 0;
							write_mem <= 0;
							stage <= 0;
						end
						7'b0000011, 7'b0100011: begin // acesso a memoria
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b10;
							//remaining
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= stage + 1'b1;
						end
						default: begin
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b10;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= 0;
						end
					endcase
			end
			3'b100: begin // estagio 4
					case (opcode)
						7'b0110011, 7'b0010011: begin // tipo r , addi
							reg_write_src <= 2'b0;
							write_reg <= 1;
							// remaining
							ula_src_a <= 2'b00;
							ula_src_b <= 2'b00;
							ula_op <= ula_op_r;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0; // escreve pc
							pc_src <= 0;
							write_pc_bkp <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= 0;
						end
						7'b0000011: begin // load word
							read_mem <= 1;
							i_ou_d <= 1;
							//remaining
							ula_src_a <= 2'b0;
							ula_src_b <= 2'b00;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= stage + 1'b1;
						end
						7'b0100011: begin // store word
							i_ou_d <= 1;
							write_mem <= 1;
							//remaining
							read_mem <= 0;
							ula_src_a <= 2'b0;
							ula_src_b <= 2'b00;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							stage <= stage + 1'b1;
						end
						default: begin
							ula_src_a <= 2'b01;
							ula_src_b <= 2'b10;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= 0;
						end
					endcase
			end
			3'b101: begin
				case (opcode)
						7'b0000011: begin // load word
							read_mem <= 1;
							i_ou_d <= 1;
							//remaining
							ula_src_a <= 2'b0;
							ula_src_b <= 2'b00;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							write_mem <= 0;
							stage <= stage + 1'b1;
						end
						7'b0100011: begin // store word
							i_ou_d <= 1;
							write_mem <= 1;
							//remaining
							read_mem <= 0;
							ula_src_a <= 2'b0;
							ula_src_b <= 2'b00;
							i_ou_d <= 0;
							read_mem <= 0;
							write_ir <= 0;
							pc_write <= 0;
							write_pc_bkp <= 0;
							ula_op <= 5'b0;
							pc_src <= 0;
							write_reg <= 0;
							reg_write_src <= 0;
							pc_write_cond <= 0;
							stage <= 0;
						end
				endcase
			end
			3'b110: begin
				write_reg <= 0;
				reg_write_src <= 2'b10;
				// remaining
				read_mem <= 0;
				i_ou_d <= 0;
				ula_src_a <= 2'b0;
				ula_src_b <= 2'b00;
				i_ou_d <= 0;
				read_mem <= 0;
				write_ir <= 0;
				pc_write <= 0;
				write_pc_bkp <= 0;
				ula_op <= 5'b0;
				pc_src <= 0;
				pc_write_cond <= 0;
				write_mem <= 0;
			end
			default: begin
				stage <= 0;
			end

		endcase
	end
endmodule

module memory(
	input logic clk,
	input logic [31:0] add,
	input logic [31:0] write_data,
	input logic write,
	input logic read,
	input logic i_ou_d,
	output logic [31:0] out
);
	wire [31:0] i_data, d_data;
	instruction_test i(	
		.address(add[11:2]),
		.clock(clk),
		.data(write_data),
		.rden((i_ou_d) ? read : 1'b0),
		.wren((i_ou_d) ? write : 1'b0),
		.q(i_data)
		);
	data_test d(	
		.address(add[11:2]),
		.clock(clk),
		.data(write_data),
		.rden((i_ou_d) ? 1'b0 : read ),
		.wren((i_ou_d) ? 1'b0 : write),
		.q(d_data)
		);
		assign out = (i_ou_d) ? i_data : d_data;
endmodule

module main(
  input logic clk,
  // PROPÓSITO DE TESTE
  // Sinais de controle do data path
	// saida da memória de dados
	// numero do registrador de display
	input logic [4:0] rs3,

	// saidas do controle
	output logic pc_write_cond,
	output logic pc_write,
	output logic write_ir,
	output logic pc_src,
	output logic [4:0] ula_op,
	output logic [1:0] ula_src_a,
	output logic [1:0] ula_src_b,
	output logic write_pc_bkp,
	output logic write_reg,
	output logic [1:0] reg_write_src,
	output logic [31:0] debug_ula_a, debug_ula_b,
	output logic i_ou_d,
	output logic read_mem,
	output logic write_mem,
	// saidas debug
	output logic [31:0] mem_out,
	output logic [31:0] instruct,
	output logic [4:0] reg_src_1,
	output logic [4:0] reg_src_2,
	output logic [4:0] reg_write_dest,
	output logic [31:0] rs3_data,
	output logic [31:0] immediate,
	output logic [31:0] reg_write_data,
	output logic [31:0] mem_data,
	output logic [31:0] data_a,
	output logic [31:0] data_b,	
	output logic [31:0] ula_a, ula_b,
	output logic [31:0] ula_result,
	output logic [31:0] ula_result_bkp,
	output logic [31:0] pc_in,
	output logic [31:0] pc_out,
	output logic [31:0] pc_bkp,
 	output logic pc_ctrl_signal,
	output logic ula_zero
);
	assign reg_src_1 = instruct[19:15];
	assign reg_src_2 = instruct[24:20];
	assign reg_write_dest = instruct[11:7];

	control_unit ctrl(
	.clk(clk), .instruction(instruct),
	.pc_write_cond(pc_write_cond),
	.pc_write(pc_write),
	.write_ir(write_ir),
	.pc_src(pc_src),
	.ula_op(ula_op),
	.ula_src_a(ula_src_a),
	.ula_src_b(ula_src_b),
	.write_pc_bkp(write_pc_bkp),
	.write_reg(write_reg),
	.reg_write_src(reg_write_src),
	.i_ou_d(i_ou_d),
	.read_mem(read_mem),
	.write_mem(write_mem)
	);

	instruction_reg ir(
		.we(write_ir), .clk(clk), .in(mem_out),
		.out(instruct)
	);

	reg32 mem_dt_reg(
		.clk(clk), .in(mem_out),
		.out(mem_data)
	);

	wire [31:0] rs1_data, rs2_data;
	
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
		.s0(ula_result_bkp), .s1(pc_out), .s2(mem_data), .s3(immediate), .c(reg_write_src),
		.out(reg_write_data)
	);

	// Parei por aqui

	imd_generator imd(	
		.instruction(instruct),
		.imd(immediate)
	);

	memory mem(
	.clk(clk),
	.add(mem_add_in),
	.write_data(data_b),
	.write(write_mem),
	.read(read_mem),
	.i_ou_d(i_ou_d),
	.out(mem_out)
	);

	//wire [31:0] pc_bkp;

	//wire [31:0] ula_a, ula_b;

	mux32_2 ula_src_a_mux(		
		.s0(pc_bkp), .s1(data_a), .s2(pc_out), .s3(debug_ula_a), .c(ula_src_a),
		.out(ula_a)
	);

	mux32_2 ula_src_b_mux(		
		.s0(data_b), .s1(32'b100), .s2(immediate), .s3(debug_ula_b), .c(ula_src_b),
		.out(ula_b)
	);

	wire [31:0] mem_add_in;

	mux32_1 mem_add_mux(
		.s0(pc_out), .s1(ula_result_bkp), .c(i_ou_d),
		.out(mem_add_in)
	);

	ula ula_block(
    .a(ula_a), .b(ula_b), .op(ula_op),
    .out(ula_result), .zero(ula_zero)
	);

	reg32 ula_res_reg(		
		.clk(clk), .in(ula_result),
		.out(ula_result_bkp)
	);

	mux32_1 pc_src_mux(
		.s0(ula_result), .s1(ula_result_bkp), .c(pc_src),
		.out(pc_in)
	);

	pc_write_control_circuit pc_ctrl(		
		.zero(ula_zero), .write_cond(pc_write_cond), .write(pc_write),
		.out(pc_ctrl_signal)
	);

	instruction_reg pc(
		.we(pc_ctrl_signal), .clk(clk), .in(pc_in),
		.out(pc_out)
	);

	instruction_reg pc_bkp_reg(
		.we(write_pc_bkp), .clk(clk), .in(pc_out),
		.out(pc_bkp)
	);

endmodule
