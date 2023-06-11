 module signextmux(
	 input [23:0] in,
	 input [1:0] select,
	 output reg[31:0] extVal);
 
	integer i;
	always @ (*) begin
	case(select)
	0 : begin
		extVal[7:0]=in[7:0];
		for(i=8;i<32;i=i+1) extVal[i]=in[7];
		end
	1 : begin
		extVal[11:0]=in[11:0];
		for(i=12;i<32;i=i+1) extVal[i]='b0;
		end
	2 : begin
		extVal[1:0]=2'b00;
		extVal[25:2]=in[23:0];
		for(i=26;i<32;i=i+1) extVal[i]=in[23];
		end
	default : extVal='h00000000;
	endcase
	end
endmodule
module register_1bit(
	input regin,
	input clk,
	input write,
	input reset,
	output reg regout);
	
	always @ (posedge clk,posedge reset)
	if(reset) begin
		regout<=0;
	end
	else if(write) begin
		regout<=regin;
	end
	
endmodule
module register_4bit(
	input [3:0] regin,
	input clk,
	input write,
	input reset,
	output reg[3:0] regout);
	
	always @ (posedge clk, posedge reset)
	if(reset) begin
		regout<=4b'0;
	end
	else if(write) begin
		regout<=regin;
	end
	
endmodule

module register(
	input[31:0] regin,
	input clk,
	input write,
	input reset,
	output reg[31:0] regout);
	
	always @ (posedge clk)
	if(reset)
		regout<='h0000;
	else if(write)
		regout<=regin;
endmodule
module decoder_4to16(
	input[3:0] in,
	output reg[15:0] out);
	
	always @ (*)
	case(in)
	0 : out='h0001;
	1 : out='h0002;
	2 : out='h0004;
	3 : out='h0008;
	4 : out='h0010;
	5 : out='h0020;
	6 : out='h0040;
	7 : out='h0080;
	8 : out='h0100;
	9 : out='h0200;
	10 : out='h0400;
	11 : out='h0800;
	12 : out='h1000;
	13 : out='h2000;
	14 : out='h4000;
	15 : out='h8000;
    endcase
endmodule
module registerfile(
	input[3:0] reg1,
	input[3:0] reg2,
	input[3:0] regdst,
	input[31:0] regsrc,
	input clk,
	input reset,
	input we,
	output[31:0] out1,
	output[31:0] out2,
	output[31:0] pc);
	
	wire[15:0] write;
	wire[31:0] registers [15:0];
	
	decoder_4to16 selecter(.in (regdst), .out (write));
	
	register register0 (.regin (regsrc), .write (write[0]&we), .clk (clk), .reset (reset), .regout (registers[0]));
	register register1 (.regin (regsrc), .write (write[1]&we), .clk (clk), .reset (reset), .regout (registers[1]));
	register register2 (.regin (regsrc), .write (write[2]&we), .clk (clk), .reset (reset), .regout (registers[2]));
	register register3 (.regin (regsrc), .write (write[3]&we), .clk (clk), .reset (reset), .regout (registers[3]));
	register register4 (.regin (regsrc), .write (write[4]&we), .clk (clk), .reset (reset), .regout (registers[4]));
	register register5 (.regin (regsrc), .write (write[5]&we), .clk (clk), .reset (reset), .regout (registers[5]));
	register register6 (.regin (regsrc), .write (write[6]&we), .clk (clk), .reset (reset), .regout (registers[6]));
	register register7 (.regin (regsrc), .write (write[7]&we), .clk (clk), .reset (reset), .regout (registers[7]));
	register register8 (.regin (regsrc), .write (write[8]&we), .clk (clk), .reset (reset), .regout (registers[8]));
	register register9 (.regin (regsrc), .write (write[9]&we), .clk (clk), .reset (reset), .regout (registers[9]));
	register register10 (.regin (regsrc), .write (write[10]&we), .clk (clk), .reset (reset), .regout (registers[10]));
	register register11 (.regin (regsrc), .write (write[11]&we), .clk (clk), .reset (reset), .regout (registers[11]));
	register register12 (.regin (regsrc), .write (write[12]&we), .clk (clk), .reset (reset), .regout (registers[12]));
	register register13 (.regin (regsrc), .write (write[13]&we), .clk (clk), .reset (reset), .regout (registers[13]));
	register register14 (.regin (regsrc), .write (write[14]&we), .clk (clk), .reset (reset), .regout (registers[14]));
	register register15 (.regin (regsrc), .write (write[15]&we), .clk (clk), .reset (reset), .regout (registers[15]));

	assign out1=registers[reg1];
	assign out2=registers[reg2];
	assign pc=registers[15];

endmodule
module stage_register_ifid (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_inst_F,
    output [31:0] o_inst_D
);

    wire [31:0] inst_D;

    register reg_inst_D (
        .regin(i_inst_F),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(inst_D)
    );

    assign o_inst_D = inst_D;

endmodule
module stage_register_idex (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_read1_D,
    input [31:0] i_read2_D,
    input [31:0] i_imm32_D,
    input i_PCSrc_D,
    input i_RegWrite_D,
    input i_MemtoReg_D,
    input i_MemWrite_D,
    input [3:0] i_InstOp_D,
    input i_Branch_D,
    input i_ALUSrc1_D,
    input i_ALUSrc2_D,
    input i_FlagWrite_D,
    input [3:0] i_Cond_D,
	input [3:0] i_WA3_D
    output [31:0] o_read1_E,
    output [31:0] o_read2_E,
    output [31:0] o_imm32_E,
    output o_PCSrc_E,
    output o_RegWrite_E,
    output o_MemtoReg_E,
    output o_MemWrite_E,
    output [3:0] o_InstOp_E,
    output o_Branch_E,
    output o_ALUSrc1_E,   
    output o_ALUSrc2_E,
    output o_FlagWrite_E,
    output [3:0] o_Cond_E,
	output [3:0] o_WA3_E
);

    wire [31:0] read1_E;
    wire [31:0] read2_E;
    wire [31:0] imm32_E;
    wire PCSrc_E;
    wire RegWrite_E;
    wire MemtoReg_E;
    wire MemWrite_E;
    wire [3:0] InstOp_E;
    wire Branch_E;
    wire ALUSrc1_E;
    wire ALUSrc2_E;
    wire FlagWrite_E;
    wire [3:0] Cond_E;
	wire [3:0] WA3_E;


    register reg_read1_E (
        .regin(i_read1_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(read1_E)
    );

    register reg_read2_E (
        .regin(i_read2_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(read2_E)
    );

    register reg_imm32_E (
        .regin(i_imm32_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(imm32_E)
    );

    register_1bit reg_PCSrc_E (
        .regin(i_PCSrc_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(PCSrc_E)
    );

    register_1bit reg_RegWrite_E (
        .regin(i_RegWrite_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(RegWrite_E)
    );

    register_1bit reg_MemtoReg_E (
        .regin(i_MemtoReg_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(MemtoReg_E)
    );

    register_1bit reg_MemWrite_E (
        .regin(i_MemWrite_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(MemWrite_E)
    );

    register_4bit reg_InstOp_E (
        .regin(i_InstOp_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(InstOp_E)
    );

    register_1bit reg_Branch_E (
        .regin(i_Branch_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(Branch_E)
    );

    register_1bit reg_ALUSrc1_E (
        .regin(i_ALUSrc1_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(ALUSrc1_E)
    );

    register_1bit reg_ALUSrc2_E (
        .regin(i_ALUSrc2_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(ALUSrc2_E)
    );

    register_1bit reg_FlagWrite_E (
        .regin(i_FlagWrite_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(FlagWrite_E)
    );

    register_4bit reg_Cond_E (
        .regin(i_Cond_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(Cond_E)
    );

	register_4bit reg_WA3_E(
		.regin(i_WA3_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(WA3_E)
	);

    assign o_read1_E = read1_E;
    assign o_read2_E = read2_E;
    assign o_imm32_E = imm32_E;
    assign o_PCSrc_E = PCSrc_E;
    assign o_RegWrite_E = RegWrite_E;
    assign o_MemtoReg_E = MemtoReg_E;
    assign o_MemWrite_E = MemWrite_E;
    assign o_InstOp_E = InstOp_E;
    assign o_Branch_E = Branch_E;
    assign o_ALUSrc1_E = ALUSrc1_E;
    assign o_ALUSrc2_E = ALUSrc2_E;
    assign o_FlagWrite_E = FlagWrite_E;
    assign o_Cond_E = Cond_E;
	assign o_WA3_E = WA3_E;
endmodule
module stage_register_exmem (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_ALUResult_E,
    input [31:0] i_WriteData_E,
    input [3:0] i_WA3_E,
    input i_PCSrc_E,
    input i_RegWrite_E,
    input i_MemtoReg_E,
    input i_MemWrite_E,
    output [31:0] o_inst_M,
    output [31:0] o_ALUOut_M,
    output [31:0] o_WriteData_M,
    output [3:0] o_WA3_M,
    output o_PCSrc_M,
    output o_RegWrite_M,
    output o_MemtoReg_M,
    output o_MemWrite_M);

    wire [31:0] ALUOut_M;
    wire [31:0] WriteData_M;
    wire [3:0] WA3_M;
    wire PCSrc_M;
    wire RegWrite_M;
    wire MemtoReg_M;
    wire MemWrite_M;


    register reg_ALUOut_M (
        .regin(i_ALUResult_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(ALUOut_M)
    );

    register reg_WriteData_M (
        .regin(i_WriteData_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(WriteData_M)
    );

    register_4bit reg_WA3_M (
        .regin(i_WA3_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(WA3_M)
    );

    register_1bit reg_PCSrc_M (
        .regin(i_PCSrc_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(PCSrc_M)
    );

    register_1bit reg_RegWrite_M (
        .regin(i_RegWrite_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(RegWrite_M)
    );

    register_1bit reg_MemtoReg_M (
        .regin(i_MemtoReg_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(MemtoReg_M)
    );

    register_1bit reg_MemWrite_M (
        .regin(i_MemWrite_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(MemWrite_M)
    );

    assign o_ALUOut_M = ALUOut_M;
    assign o_WriteData_M = WriteData_M;
    assign o_WA3_M = WA3_M;
    assign o_PCSrc_M = PCSrc_M;
    assign o_RegWrite_M = RegWrite_M;
    assign o_MemtoReg_M = MemtoReg_M;
    assign o_MemWrite_M = MemWrite_M;

endmodule
module stage_register_memwb (
    input clk, 
    input reset, 
    input flush, 
    input stop,
	input i_PCSrc_M,
	input i_RegWrite_M,
	input i_MemtoReg_M,
	input [31:0] i_ReadData_M,
	input [31:0] i_ALUOut_M,
	input [3:0] i_WA3_M,
	output o_PCSrc_W,
	output o_RegWrite_W,
	output o_MemtoReg_W,
	output [31:0] o_ReadData_W,
	output [31:0] o_ALUOut_W,
	output [3:0] o_WA3_W);

	wire PCSrc_W;	
	wire RegWrite_W;
	wire MemtoReg_W;
	wire [31:0] ReadData_W;
	wire [31:0] ALUOut_W;
	wire [3:0] WA3_W;

	register_1bit reg_PCSrc_W (
	.regin(i_PCSrc_M),
	.clk(clk),
	.write(~stop),
	.reset(reset),
	.regout(PCSrc_W)
    );

    register_1bit reg_RegWrite_W (
        .regin(i_RegWrite_M),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(RegWrite_W)
    );

    register_1bit reg_MemtoReg_W (
        .regin(i_MemtoReg_M),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(MemtoReg_W)
    );

	register reg_ReadData_W(
		.regin(i_ReadData_M),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(ReadData_W)
	);

	register reg_ALUOut_W(
		.regin(i_ALUOut_M),
		.clk(clk),
		.write(~stop),
		.reset(reset),
		.regout(ALUOut_W)
	);

	register_4bit reg_WA3_W(
		.regin(i_WA3_M),
		.clk(clk),
		.write(~stop),
		.reset(reset),
		.regout(WA3_W)
	);

	assign o_PCSrc_W = PCSrc_W;
	assign o_RegWrite_W = RegWrite_W;
	assign o_MemtoReg_W = MemtoReg_W;
	assign o_ReadData_W = ReadData_W;
	assign o_ALUOut_W = ALUOut_W;
	assign o_WA3_W = WA3_W;

endmodule
module armreduced(
	input clk,
	input reset,
	input nIRQ,
	input[31:0] inst,
	input[31:0] readdata,
	output memread,
	output memwrite,
	output [3:0] be,
	output[31:0] pc,
	output[31:0] memaddr,
	output[31:0] writedata);
	
	wire [31:0] wire_pc;

	// for IFID
	wire [31:0] o_inst_D;

	// for IDEX
	wire i_PCSrc_D;
	wire i_RegWrite_D;
	wire i_MemtoReg_D;
	wire i_MemWrite_D;
	wire i_Branch_D;
	wire i_ALUSrc1_D;
	wire i_ALUSrc2_D;
	wire [3:0] i_InstOp_D;
	wire i_FlagWrite_D;
	wire [31:0] i_read1_D;
	wire [31:0] i_read2_D;
	wire [31:0] i_imm32_D;
	wire o_PCSrc_E;
	wire o_RegWrite_E;
	wire o_MemtoReg_E;
	wire o_MemWrite_E;
	wire o_Branch_E;
	wire o_ALUSrc1_E;
	wire o_ALUSrc2_E;
	wire [3:0] o_InstOp_E;
	wire o_FlagWrite_E;
	wire [31:0] o_read1_E;
	wire [31:0] o_read2_E;
	wire [31:0] o_imm32_E;
	wire [3:0] o_WA3_E;

	// for EXMEM
  	wire [31:0] i_ALUResult_E;
    wire [31:0] i_WriteData_E;
    wire [3:0] i_WA3_E;
    wire i_PCSrc_E;
    wire i_RegWrite_E;
    wire i_MemtoReg_E;
    wire i_MemWrite_E;
    wire [31:0] o_inst_M;
    wire [31:0] o_ALUOut_M;
    wire [31:0] o_WriteData_M;
    wire [3:0] o_WA3_M;
    wire o_PCSrc_M;
    wire o_RegWrite_M;
    wire o_MemtoReg_M;
    wire o_MemWrite_M;

	// for memwb
	wire i_PCSrc_M;
	wire i_RegWrite_M;
	wire i_MemtoReg_M;
	wire [31:0] i_ReadData_M;
	wire [31:0] i_ALUOut_M;
	wire [3:0] i_WA3_M;
	wire o_PCSrc_W;
	wire o_RegWrite_W;
	wire o_MemtoReg_W;
	wire [31:0] o_ReadData_W;
	wire [31:0] o_ALUOut_W;
	wire [3:0] o_WA3_W;



	wire [3:0] i_NZCV;
	wire [3:0] o_NZCV;


	wire [1:0] immSrc_D;
	wire [3:0] reg2_D [1:0];
	wire [31:0] reg1_D [1:0];

	wire CondExE;
	wire [31:0] ALU_A [1:0];
	wire [31:0] ALU_B [1:0];
	wire [2:0] ALUop_E;

	wire [31:0] Result_W [1:0];

	register_4bit NZCVRegister(.regin(i_NZCV), .clk(clk), .write(o_FlagWrite_E), .reset(reset), .regout(o_NZCV));

	stage_register_ifid IFID(.clk(clk), .reset(reset), .flush(), .stop(), .i_inst_F(inst), .o_inst_D(o_inst_D));
	
	newControlUnit ControlUnit(.inst(o_inst_D), .PCSrc(i_PCSrc_D), .RegWrite(i_RegWrite_D), .MemtoReg(i_MemtoReg_D), .MemWrite(i_MemWrite_D), .Branch(i_Branch_D), .InstOp(i_InstOp_D), .FlagWrite(i_FlagWrite_D), immSrc(immSrc_D), .ALUSrc1(i_ALUSrc1_D), .ALUSrc2(i_ALUSrc2_D));
	registerfile registerFile(.clk(clk), .reset(reset), .reg1(inst[19:16]), .reg2(reg2_D[RegSrcD]), .regdst(o_WA3_W), .regsrc(Result_W[o_MemtoReg_W]), .we(o_RegWrite_W), .pc(wire_pc), .out1(reg1_D[0]), .out2(i_read2_D));
	signextmux Extend(.in(o_inst_D[23:0]), .select(immSrc_D), .extVal(i_imm32_D));
	i_read1_D = reg1_D[RegSrcD];
	reg2_D[0] = inst[3:0];
	reg2_D[1] = inst[15:12];
	stage_register_idex IDEX(
    .clk(clk), 
    .reset(reset), 
    .flush(), 
    .stop(),
    .i_read1_D(i_read1_D),
    .i_read2_D(i_read2_D),
    .i_imm32_D(i_imm32_D),
    .i_PCSrc_D(i_PCSrc_D),
    .i_RegWrite_D(i_RegWrite_D),
    .i_MemtoReg_D(i_MemtoReg_D),
    .i_MemWrite_D(i_MemWrite_D),
    .i_InstOp_D(i_InstOp_D),
    .i_Branch_D(i_Branch_D),
    .i_ALUSrc1_D(i_ALUSrc1_D),
    .i_ALUSrc2_D(i_ALUSrc2_D),
    .i_FlagWrite_D(i_FlagWrite_D),
    .i_Cond_D(o_inst_D[31:28]),
	.i_WA3_D(o_inst_D[15:12]),
    .o_read1_E(o_read1_E),
    .o_read2_E(o_read2_E),
    .o_imm32_E(o_imm32_E),
    .o_PCSrc_E(o_PCSrc_E),
    .o_RegWrite_E(o_RegWrite_E),
    .o_MemtoReg_E(o_MemtoReg_E),
    .o_MemWrite_E(o_MemWrite_E),
    .o_InstOp_E(o_InstOp_E),
    .o_Branch_E(o_Branch_E),
    .o_ALUSrc1_E(o_ALUSrc1_E),   
    .o_ALUSrc2_E(o_ALUSrc2_E),
    .o_FlagWriteE(o_FlagWrite_E),
    .o_Cond_E(o_Cond_E),
	.o_WA3_E(o_WA3_E)
	);

	ALU_A[0] = o_read1_E;
	ALU_A[1] = 0;
	ALU_B[0] = o_read2_E;
	ALU_B[1] = o_imm32_E;
	i_WriteData_E = o_read2_E;

	ALUopdecoder ALUopDecoder(
		.instop (o_InstOp_E),
		.aluop (ALUop_E));
	ALU32bit ALU(
		.inpa (ALU_A[o_ALUSrc1_E]),
		.inpb (ALU_B[o_ALUSrc2_E]),
		.cin (o_NZCV[1]),
		.aluop (ALUop_E),
		.result (i_ALUResult_E),
		.negative (i_NZCV[3]),
		.zero (i_NZCV[2]),
		.cout (i_NZCV[1]),
		.overflow (i_NZCV[0]));
	newCondUnit CondUnit(.FlagWriteE(o_FlagWrite_E), .CondE(o_Cond_E), .FlagsE(o_NZCV), .CondExE(CondExE));
	i_PCSrc_E = (o_PCSrc_E && CondExE) || (o_Branch_E && CondExE);
	i_RegWrite_E = o_RegWrite_E && CondExE;
	i_MemtoReg_E = o_MemtoReg_E;
	i_MemWrite_E = o_MemWrite_E && CondExE;
	

	stage_register_exmem EXMEM(
	.clk(clk),
    .reset(reset),
    .flush(),
    .stop(),
    .i_ALUResult_E(i_ALUResult_E),
    .i_WriteData_E(i_WriteData_E),
    .i_WA3_E(o_WA3_E),
    .i_PCSrc_E(i_PCSrc_E),
    .i_RegWrite_E(i_RegWrite_E),
    .i_MemtoReg_E(i_MemtoReg_E),
    .i_MemWrite_E(i_MemWrite_E),
    .o_inst_M(o_inst_M),
    .o_ALUOut_M(o_ALUOut_M),
    .o_WriteData_M(o_WriteData_M),
    .o_WA3_M(o_WA3_M),
    .o_PCSrc_M(o_PCSrc_M),
    .o_RegWrite_M(o_RegWrite_M),
    .o_MemtoReg_M(o_MemtoReg_M),
    .o_MemWrite_M(o_MemWrite_M));

	assign memwrite = o_Memwrite_M;
	assign memaddr = o_ALUOut_M;
	assign writedata = o_WriteData_M;
	assign memread = 1b'1;
	assign be = 4b'1111;

	stage_register_memwb MEMWB(
    .clk(clk),
    .reset(reset),
    .flush()
    .stop()
	.i_PCSrc_M(o_PCSrc_M),
	.i_RegWrite_M(o_RegWrite_M),
	.i_MemtoReg_M(o_MemtoReg_M),
	.i_ReadData_M(readdata),
	.i_ALUOut_M(o_ALUOut_M),
	.i_WA3_M(o_WA3_M),
	.o_PCSrc_W(o_PCSrc_W),
	.o_RegWrite_W(o_RegWrite_W),
	.o_MemtoReg_W(o_MemtoReg_W),
	.o_ReadData_W(o_ReadData_W),
	.o_ALUOut_W(o_ALUOut_W)
	.o_WA3_W(o_WA3_W));

	Result_W[0] = o_ALUOut_W;
	Result_W[1] = o_ReadData_W;


	

	assign pc = wire_pc + 4; 
	// need to change :

endmodule