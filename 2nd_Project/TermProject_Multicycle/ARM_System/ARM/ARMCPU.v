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
    input [31:0] i_pc_F, 
    input [31:0] i_inst_F,
    output [31:0] o_pc_D, 
    output [31:0] o_inst_D
);

    wire [31:0] pc_D;
    wire [31:0] inst_D;

    register reg_pc_D (
        .regin(i_pc_F),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(pc_D)
    );

    register reg_inst_D (
        .regin(i_inst_F),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(inst_D)
    );

    assign o_pc_D = pc_D;
    assign o_inst_D = inst_D;

endmodule
module stage_register_idex (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_pc_D, 
    input [31:0] i_inst_D,
    input [31:0] i_read1_D,
    input [31:0] i_read2_D,
    input [31:0] i_imm32_D,
    input i_PCsrc_D,
    input i_RegWrite_D,
    input i_MemtoReg_D,
    input i_MemWrite_D,
    input [3:0] i_ALUOp_D,
    input i_Branch_D,
    input i_ALUSrc1_D,
    input i_ALUSrc2_D,
    input i_FlagWrite_D,
    input [3:0] i_Cond_D,
    input [3:0] i_Flags_D,
    output [31:0] o_pc_E, 
    output [31:0] o_inst_E,
    output [31:0] o_read1_E,
    output [31:0] o_read2_E,
    output [31:0] o_imm32_D,
    output o_PCSrc_E,
    output o_RegWrite_E,
    output o_MemtoReg_E,
    output o_MemWrite_E,
    output [3:0] o_ALUOp_E,
    output o_Branch_E,
    output o_ALUSrc1_E,   
    output o_ALUSrc2_E,
    output o_FlagWriteE,
    output [3:0] o_Cond_E,
    output [3:0] o_Flags_E
);

    wire [31:0] pc_E;
    wire [31:0] inst_E;
    wire [31:0] read1_E;
    wire [31:0] read2_E;
    wire [31:0] imm32_E;
    wire PCsrc_E;
    wire RegWrite_E;
    wire MemtoReg_E;
    wire MemWrite_E;
    wire [3:0] ALUOp_E;
    wire Branch_E;
    wire ALUSrc1_E;
    wire ALUSrc2_E;
    wire FlagWrite_E;
    wire [3:0] Cond_E;
    wire [3:0] Flags_E;

    register reg_pc_E (
        .regin(i_pc_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(pc_E)
    );

    register reg_inst_E (
        .regin(i_inst_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(inst_E)
    );

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
        .regin(i_PCsrc_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(PCsrc_E)
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

    register_4bit reg_ALUOp_E (
        .regin(i_ALUOp_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(ALUOp_E)
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

    register_4bit reg_Flags_E (
        .regin(i_Flags_D),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(Flags_E)
    );

    assign o_pc_E = pc_E;
    assign o_inst_E = inst_E;
    assign o_read1_E = read1_E;
    assign o_read2_E = read2_E;
    assign o_imm32_D = imm32_E;
    assign o_PCSrc_E = PCsrc_E;
    assign o_RegWrite_E = RegWrite_E;
    assign o_MemtoReg_E = MemtoReg_E;
    assign o_MemWrite_E = MemWrite_E;
    assign o_ALUOp_E = ALUOp_E;
    assign o_Branch_E = Branch_E;
    assign o_ALUSrc1_E = ALUSrc1_E;
    assign o_ALUSrc2_E = ALUSrc2_E;
    assign o_FlagWriteE = FlagWrite_E;
    assign o_Cond_E = Cond_E;
    assign o_Flags_E = Flags_E;

endmodule
module stage_register_exmem (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_pc_E, 
    input [31:0] i_inst_E,
    input [31:0] i_ALUResult_E,
    input [31:0] i_WriteData_E,
    input [3:0] i_WA3_E,
    input i_PCSrc_E,
    input i_RegWrite_E,
    input i_MemtoReg_E,
    input i_MemWrite_E,
    output [31:0] o_pc_M, 
    output [31:0] o_inst_M,
    output [31:0] o_ALUOut_M,
    output [31:0] o_WriteData_M,
    output [3:0] o_WA3_M,
    output o_PCSrc_M,
    output o_RegWrite_M,
    output o_MemtoReg_M,
    output o_MemWrite_M
);

    wire [31:0] pc_M;
    wire [31:0] inst_M;
    wire [31:0] ALUOut_M;
    wire [31:0] WriteData_M;
    wire [3:0] WA3_M;
    wire PCSrc_M;
    wire RegWrite_M;
    wire MemtoReg_M;
    wire MemWrite_M;

    register reg_pc_M (
        .regin(i_pc_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(pc_M)
    );

    register reg_inst_M (
        .regin(i_inst_E),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(inst_M)
    );

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

    assign o_pc_M = pc_M;
    assign o_inst_M = inst_M;
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
    input [31:0] i_pc_M, 
    input [31:0] i_inst_M,
		input i_PCSrc_M,
		input i_RegWrite_M,
		input i_MemtoReg_M,
    output [31:0] o_pc_W, 
    output [31:0] o_inst_W
		output o_PCSrc_W,
		output o_RegWrite_W,
		output o_MemtoReg_W,
);

    wire [31:0] pc_W;
    wire [31:0] inst_W;
		wire PCSrc_W;
		wire RegWrite_W;
		wire MemtoReg_W;

    register reg_pc_W (
        .regin(i_pc_M),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(pc_W)
    );

    register reg_inst_W (
        .regin(i_inst_M),
        .clk(clk),
        .write(~stop),
        .reset(reset),
        .regout(inst_W)
    );

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

    assign o_pc_D = pc_D;
    assign o_inst_D = inst_D;
		assign o_PCSrc_W = PCSrc_W;
		assign o_RegWrite_W = RegWrite_W;
		assign o_MemtoReg_W = MemtoReg_W;

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
	
	//signals
  wire IRwrite,regwrite,NZCVwrite;
  wire [1:0] regdst,regsrc,ALUsrcA,immsrc,ALUsrcB;
  wire [2:0] ALUop;
  wire [3:0] instop;
  wire regBdst;

  
  //wires_out
  wire[31:0] imm;
  
  //wires_in
  wire [3:0] ALUflags;
  wire [31:0] ALUresult,readA,readB;
  
  wire [3:0] RFdst [2:0];
  wire [31:0] RFsrc [3:0];
  wire [31:0] ALUnum1 [2:0];
  wire [31:0] ALUnum2 [3:0];
  
  wire [3:0] regBread [1:0];
  
  //registers
  assign be = 4'b1111;
  wire[31:0] A,B,instructions,mdr,ALUout;
  reg n,v;
  wire z,c;
  
  register MDR(.regin (readdata), .write ('b1), .clk (clk), .reset (reset), .regout (mdr));
  register ALUoutRegister(.regin (ALUresult), .write ('b1), .clk (clk), .reset (reset), .regout (ALUout));
  register A_Register(.regin (readA), .write ('b1), .clk (clk), .reset (reset), .regout (A));
  register B_Register(.regin (readB), .write ('b1), .clk (clk), .reset (reset), .regout (B));
  register InstructionRegister(.regin (inst), .write (IRwrite), .clk (clk), .reset (reset), .regout (instructions));
  //NZCVregister NZCV(.regin (ALUflags), .write(NZCVwrite), .clk (clk), .reset (reset), .regout({n,z,c,v}));
  register_1bit Z(.regin (ALUflags[2]), .reset (reset), .clk (clk), .write (NZCVwrite), .regout (z));
  register_1bit C(.regin (ALUflags[1]), .reset (reset), .clk (clk), .write (NZCVwrite), .regout (c));
	
	//mux
	assign RFdst[0]=instructions[15:12];
	assign RFdst[1]=4'b1111;
	assign RFdst[2]=4'b1110;

	assign RFsrc[0]=mdr;
	assign RFsrc[1]=ALUout;
	assign RFsrc[2]=ALUresult;
	assign RFsrc[3]=B;

	assign regBread[0]=instructions[3:0];
	assign regBread[1]=instructions[15:12];

	assign ALUnum1[0]=pc;
	assign ALUnum1[1]=A;
	assign ALUnum1[2]='h00000000;

	assign ALUnum2[0]='h00000004;
	assign ALUnum2[1]='h00000008;
	assign ALUnum2[2]=imm;
	assign ALUnum2[3]=B<<instructions[11:7];

	assign writedata=B;
	assign memaddr=ALUout;

	signalunit SignalControl(
		.clk (clk),
		.reset (reset),
		.flags (instructions[31:20]),
		.zero (z),
		.Mwrite (memwrite),
		.IRwrite (IRwrite),
		.Mread (memread),
		.regwrite (regwrite),
		.regdst (regdst),
		.regsrc (regsrc),
		.ALUsrcA (ALUsrcA),
		.ALUsrcB (ALUsrcB),
		.ALUop (instop),
		.NZCVwrite (NZCVwrite),
		.immsrc (immsrc),
		.regbdst (regBdst));  
	
	registerfile RegisterFile(
		.reg1 (instructions[19:16]),
		.reg2 (regBread[regBdst]),
		.regdst (RFdst[regdst]),
		.regsrc (RFsrc[regsrc]),
		.clk (clk),
		.reset (reset),
		.we (regwrite),
		.out1 (readA),
		.out2 (readB),
		.pc (pc));
 
	signextmux Immidiate(instructions[23:0],immsrc,imm);

	ALUopdecoder ALUopDecoder(
		.instop (instop),
		.aluop (ALUop));
	
	ALU32bit ALU(
		.inpa (ALUnum1[ALUsrcA]),
		.inpb (ALUnum2[ALUsrcB]),
		.cin (c),
		.aluop (ALUop),
		.result (ALUresult),
		.negative (ALUflags[3]),
		.zero (ALUflags[2]),
		.cout (ALUflags[1]),
		.overflow (ALUflags[0]));
	
endmodule