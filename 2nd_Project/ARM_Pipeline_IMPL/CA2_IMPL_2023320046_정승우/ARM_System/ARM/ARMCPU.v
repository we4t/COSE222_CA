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
module register_2bit(
   input [1:0] regin,
   input clk,
   input write,
   input reset,
   output reg [1:0] regout);
   
   always @ (posedge clk,posedge reset)
   if(reset) begin
      regout<=2'b00;
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
      regout<='b0000;
   end
   else if(write) begin
      regout<=regin;
   end
   
endmodule

module register_5bit(
   input [4:0] regin,
   input clk,
   input write,
   input reset,
   output reg[4:0] regout);
   
   always @ (posedge clk, posedge reset)
   if(reset) begin
      regout<='b00000;
   end
   else if(write) begin
      regout<=regin;
   end
   
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
   input[31:0] R15,
   input clk,
   input reset,
   input we,
   input BL,
   output[31:0] out1,
   output[31:0] out2,
   output[31:0] pc);
   
   wire[15:0] write;
   wire[31:0] registers [15:0];
   wire[31:0] R14_src, R15_src;
   wire R14Write;
   
   decoder_4to16 selecter(.in (regdst), .out (write));
   
   assign R14_src = BL ? registers[15] : regsrc;
   assign R14Write = BL ? 1'b1 : (write[14] && we);
   assign R15_src = (write[15]&&we) ? regsrc : R15;
   
   register register0 (.regin (regsrc), .write (write[0]&&we), .clk (clk), .reset (reset), .regout (registers[0]));
   register register1 (.regin (regsrc), .write (write[1]&&we), .clk (clk), .reset (reset), .regout (registers[1]));
   register register2 (.regin (regsrc), .write (write[2]&&we), .clk (clk), .reset (reset), .regout (registers[2]));
   register register3 (.regin (regsrc), .write (write[3]&&we), .clk (clk), .reset (reset), .regout (registers[3]));
   register register4 (.regin (regsrc), .write (write[4]&&we), .clk (clk), .reset (reset), .regout (registers[4]));
   register register5 (.regin (regsrc), .write (write[5]&&we), .clk (clk), .reset (reset), .regout (registers[5]));
   register register6 (.regin (regsrc), .write (write[6]&&we), .clk (clk), .reset (reset), .regout (registers[6]));
   register register7 (.regin (regsrc), .write (write[7]&&we), .clk (clk), .reset (reset), .regout (registers[7]));
   register register8 (.regin (regsrc), .write (write[8]&&we), .clk (clk), .reset (reset), .regout (registers[8]));
   register register9 (.regin (regsrc), .write (write[9]&&we), .clk (clk), .reset (reset), .regout (registers[9]));
   register register10 (.regin (regsrc), .write (write[10]&&we), .clk (clk), .reset (reset), .regout (registers[10]));
   register register11 (.regin (regsrc), .write (write[11]&&we), .clk (clk), .reset (reset), .regout (registers[11]));
   register register12 (.regin (regsrc), .write (write[12]&&we), .clk (clk), .reset (reset), .regout (registers[12]));
   register register13 (.regin (regsrc), .write (write[13]&&we), .clk (clk), .reset (reset), .regout (registers[13]));
   register register14 (.regin (R14_src), .write (R14Write), .clk (clk), .reset (reset), .regout (registers[14]));
   register register15 (.regin (R15_src), .write (1'b1), .clk (clk), .reset (reset), .regout (registers[15]));

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
        .reset(reset||flush),
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
    input i_ALUSrc1_D,
    input i_ALUSrc2_D,
    input [3:0] i_WA3_D,
    input [4:0] i_shmt5_D,
    input i_MemRead_D,
    input i_NZCVWrite_D,
    input [3:0] i_Cond_D,
    input [3:0] i_Flags_D,
    output [31:0] o_read1_E,
    output [31:0] o_read2_E,
    output [31:0] o_imm32_E,
    output o_PCSrc_E,
    output o_RegWrite_E,
    output o_MemtoReg_E,
    output o_MemWrite_E,
    output [3:0] o_InstOp_E,
    output o_ALUSrc1_E,   
    output o_ALUSrc2_E,
    output [3:0] o_WA3_E,
    output [4:0] o_shmt5_E,
    output o_MemRead_E,
    output o_NZCVWrite_E,
    output [3:0] o_Cond_E,
    output [3:0] o_Flags_E);

    wire [31:0] read1_E;
    wire [31:0] read2_E;
    wire [31:0] imm32_E;
    wire PCSrc_E;
    wire RegWrite_E;
    wire MemtoReg_E;
    wire MemWrite_E;
    wire MemRead_E;
    wire [3:0] InstOp_E;
    wire ALUSrc1_E;
    wire ALUSrc2_E;
    wire [3:0] WA3_E;
    wire [4:0] shmt5_E;
    wire NZCVWrite_E;
    wire [3:0] Cond_E;
    wire [3:0] Flags_E;
    
   register_4bit reg_Cond_E (
        .regin(i_Cond_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(Cond_E)
    );  
      
    register_4bit reg_Flags_E (
        .regin(i_Flags_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(Flags_E)
    );

    register reg_read1_E (
        .regin(i_read1_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(read1_E)
    );

    register reg_read2_E (
        .regin(i_read2_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(read2_E)
    );

    register reg_imm32_E (
        .regin(i_imm32_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(imm32_E)
    );

    register_1bit reg_PCSrc_E (
        .regin(i_PCSrc_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(PCSrc_E)
    );

    register_1bit reg_RegWrite_E (
        .regin(i_RegWrite_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(RegWrite_E)
    );

    register_1bit reg_MemtoReg_E (
        .regin(i_MemtoReg_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemtoReg_E)
    );

    register_1bit reg_MemWrite_E (
        .regin(i_MemWrite_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemWrite_E)
    );

    register_4bit reg_InstOp_E (
        .regin(i_InstOp_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(InstOp_E)
    );

    register_1bit reg_ALUSrc1_E (
        .regin(i_ALUSrc1_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(ALUSrc1_E)
    );

    register_1bit reg_ALUSrc2_E (
        .regin(i_ALUSrc2_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(ALUSrc2_E)
    );

   register_4bit reg_WA3_E(
      .regin(i_WA3_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(WA3_E)
   );

    register_5bit reg_shmt5_E(
        .regin(i_shmt5_D),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(shmt5_E)
    );
    
    register_1bit reg_MemRead_E(
      .regin(i_MemRead_D),
      .clk(clk),
      .write(~stop),
        .reset(reset||flush),
      .regout(MemRead_E)
   );
   
   register_1bit reg_NZCVWrite_E(
      .regin(i_NZCVWrite_D),
      .clk(clk),
      .write(~stop),
        .reset(reset||flush),
      .regout(NZCVWrite_E));
   

    assign o_read1_E = read1_E;
    assign o_read2_E = read2_E;
    assign o_imm32_E = imm32_E;
    assign o_PCSrc_E = PCSrc_E;
    assign o_RegWrite_E = RegWrite_E;
    assign o_MemtoReg_E = MemtoReg_E;
    assign o_MemWrite_E = MemWrite_E;
    assign o_InstOp_E = InstOp_E;
    assign o_ALUSrc1_E = ALUSrc1_E;
    assign o_ALUSrc2_E = ALUSrc2_E;
   assign o_WA3_E = WA3_E;
    assign o_shmt5_E = shmt5_E;
    assign o_MemRead_E = MemRead_E;
    assign o_NZCVWrite_E = NZCVWrite_E;
    assign o_Cond_E = Cond_E;
    assign o_Flags_E = Flags_E;

endmodule
module stage_register_exmem (
    input clk, 
    input reset, 
    input flush, 
    input stop,
    input [31:0] i_ALUResult_E,
    input [31:0] i_WriteData_E,
    input [3:0] i_WA3_E,
    input i_RegWrite_E,
    input i_MemtoReg_E,
    input i_MemWrite_E,
    input i_MemRead_E,
    output [31:0] o_ALUOut_M,
    output [31:0] o_WriteData_M,
    output [3:0] o_WA3_M,
    output o_RegWrite_M,
    output o_MemtoReg_M,
    output o_MemWrite_M,
    output o_MemRead_M);

    wire [31:0] ALUOut_M;
    wire [31:0] WriteData_M;
    wire [3:0] WA3_M;
    wire RegWrite_M;
    wire MemtoReg_M;
    wire MemWrite_M;
    wire MemRead_M;


    register reg_ALUOut_M (
        .regin(i_ALUResult_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(ALUOut_M)
    );

    register reg_WriteData_M (
        .regin(i_WriteData_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(WriteData_M)
    );

    register_4bit reg_WA3_M (
        .regin(i_WA3_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(WA3_M)
    );

    register_1bit reg_RegWrite_M (
        .regin(i_RegWrite_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(RegWrite_M)
    );

    register_1bit reg_MemtoReg_M (
        .regin(i_MemtoReg_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemtoReg_M)
    );

    register_1bit reg_MemWrite_M (
        .regin(i_MemWrite_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemWrite_M)
    );
    
    register_1bit reg_MemRead_M (
        .regin(i_MemRead_E),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemRead_M)
    );

    assign o_ALUOut_M = ALUOut_M;
    assign o_WriteData_M = WriteData_M;
    assign o_WA3_M = WA3_M;
    assign o_RegWrite_M = RegWrite_M;
    assign o_MemtoReg_M = MemtoReg_M;
    assign o_MemWrite_M = MemWrite_M;
    assign o_MemRead_M = MemRead_M;

endmodule
module stage_register_memwb (
    input clk, 
    input reset, 
    input flush, 
    input stop,
   input i_RegWrite_M,
   input i_MemtoReg_M,
   input [31:0] i_ALUOut_M,
   input [3:0] i_WA3_M,
   output o_RegWrite_W,
   output o_MemtoReg_W,
   output [31:0] o_ALUOut_W,
   output [3:0] o_WA3_W);

   wire RegWrite_W;
   wire MemtoReg_W;
   wire [31:0] ALUOut_W;
   wire [3:0] WA3_W;


    register_1bit reg_RegWrite_W (
        .regin(i_RegWrite_M),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(RegWrite_W)
    );

    register_1bit reg_MemtoReg_W (
        .regin(i_MemtoReg_M),
        .clk(clk),
        .write(~stop),
        .reset(reset||flush),
        .regout(MemtoReg_W)
    );

   register reg_ALUOut_W(
      .regin(i_ALUOut_M),
      .clk(clk),
      .write(~stop),
      .reset(reset||flush),
      .regout(ALUOut_W)
   );

   register_4bit reg_WA3_W(
      .regin(i_WA3_M),
      .clk(clk),
      .write(~stop),
      .reset(reset||flush),
      .regout(WA3_W)
   );

   assign o_RegWrite_W = RegWrite_W;
   assign o_MemtoReg_W = MemtoReg_W;
   assign o_ALUOut_W = ALUOut_W;
   assign o_WA3_W = WA3_W;
endmodule
module armreduced(
	input clk,
	input reset,
	output[31:0] pc,
	input[31:0] inst,
	input nIRQ,
	output [3:0] be,
	output[31:0] memaddr,
	output memwrite,
	output memread,
	output[31:0] writedata,
	input[31:0] readdata);

	//-----------wires-----------
	wire [31:0] i_wire_pc;
	wire [31:0] o_wire_pc;

	// for IFID
	wire [31:0] o_inst_D;
	wire [31:0] input_inst;

	// for IDEX
	wire i_PCSrc_D;
	wire i_RegWrite_D;
	wire i_MemtoReg_D;
	wire i_MemWrite_D;
	wire i_RegSrc1_D;
	wire i_RegSrc2_D;
	wire i_ALUSrc1_D;
	wire i_ALUSrc2_D;
	wire [3:0] i_WA3_D;
	wire [3:0] i_InstOp_D;
	wire [31:0] i_read1_D;
	wire [31:0] i_read2_D;
	wire [31:0] i_imm32_D;
	wire [11:7] i_shmt5_D;
	wire i_MemRead_D;
	wire i_NZCVWrite_D;
	wire [3:0] i_Flags_D;
	wire [3:0] i_Cond_D;
	wire BL;
	wire o_PCSrc_E;
	wire o_RegWrite_E;
	wire o_MemtoReg_E;
	wire o_MemWrite_E;
	wire o_ALUSrc1_E;
	wire o_ALUSrc2_E;
	wire [3:0] o_InstOp_E;
	wire [4:0] o_shmt5_E;
	wire [31:0] o_read1_E;
	wire [31:0] o_read2_E;
	wire [31:0] o_imm32_E;
	wire [3:0] o_Cond_E;
	wire [3:0] o_WA3_E;
	wire o_MemRead_E;
	wire [3:0] o_Flags;
	wire o_NZCVWrite_E;
	wire [3:0] o_Flags_E;

	// for EXMEM
	wire [31:0] i_ALUResult_E;
	wire [31:0] i_WriteData_E;
	wire [3:0] i_WA3_E;
	wire i_RegWrite_E;
	wire i_MemtoReg_E;
	wire i_MemWrite_E;
	wire i_MemRead_E;
	wire [31:0] o_ALUOut_M;
	wire [31:0] o_WriteData_M;
	wire [3:0] o_WA3_M;
	wire o_RegWrite_M;
	wire o_MemtoReg_M;
	wire o_MemWrite_M;
	wire o_MemRead_M;

	// for memwb
	wire i_RegWrite_M;
	wire i_MemtoReg_M;
	wire [31:0] i_ALUOut_M;
	wire [3:0] i_WA3_M;
	wire o_RegWrite_W;
	wire o_MemtoReg_W;
	wire [31:0] o_ALUOut_W;
	wire [3:0] o_WA3_W;

	// NZCV (Updates at CondUnit, writes at IDEX stage register)
	wire [3:0] o_NZCV;
	wire [3:0] after_ALU_Flags;

	// immedieate mux selector
	wire [1:0] immSrc_D;

	// MUX selctions
	wire [3:0] reg2_D [1:0];
	wire [3:0] reg1_D [1:0];
	wire [31:0] PCNext [1:0];
	wire [31:0] DataHzrdPCNext[1:0];
	wire [31:0] ALU_A [1:0];
	wire [31:0] ALU_B [1:0];
	wire [31:0] Result_W [1:0];

	// ALUOp
	wire [2:0] ALUop_E;

	// hazardDetection
	wire dataHzrdDetected, ctrlHzrdDetected;
	wire CondExE;
	wire stallIFID;
	wire flushIDEX;
	wire flushIFID;

	// for registerfile
	wire [3:0] reg1, reg2, regdst;
	wire [31:0] regsrc, R15;

	//--------------------------------------------------IFID-------------------------------------------------
	stage_register_ifid IFID(.clk(clk), 
	.reset(reset), 
	.flush(flushIFID), 
	.stop(stallIFID), 
	//.flush('b0),
	//.stop('b0),
	.i_inst_F(input_inst), 
	.o_inst_D(o_inst_D));
	//-------------------------------------------------------------------------------------------------------
	//------------------------------------------------IDEX----------------------------------------------------
	stage_register_idex IDEX(
		.clk(clk), 
		.reset(reset), 
		.flush(flushIDEX), 
		.stop('b0),
		//.flush('b0),
		//.stop('b0),
		.i_read1_D(i_read1_D),
		.i_read2_D(i_read2_D),
		.i_imm32_D(i_imm32_D),
		.i_PCSrc_D(i_PCSrc_D),
		.i_RegWrite_D(i_RegWrite_D),
		.i_MemtoReg_D(i_MemtoReg_D),
		.i_MemWrite_D(i_MemWrite_D),
		.i_InstOp_D(i_InstOp_D),
		.i_ALUSrc1_D(i_ALUSrc1_D),
		.i_ALUSrc2_D(i_ALUSrc2_D),
		.i_WA3_D(i_WA3_D),
		.i_shmt5_D(i_shmt5_D),
		.i_MemRead_D(i_MemRead_D),
		.i_NZCVWrite_D(i_NZCVWrite_D),
		.i_Flags_D(i_Flags_D),
		.i_Cond_D(i_Cond_D),
		.o_read1_E(o_read1_E),
		.o_read2_E(o_read2_E),
		.o_imm32_E(o_imm32_E),
		.o_PCSrc_E(o_PCSrc_E),
		.o_RegWrite_E(o_RegWrite_E),
		.o_MemtoReg_E(o_MemtoReg_E),
		.o_MemWrite_E(o_MemWrite_E),
		.o_InstOp_E(o_InstOp_E),
		.o_ALUSrc1_E(o_ALUSrc1_E),   
		.o_ALUSrc2_E(o_ALUSrc2_E),
		.o_WA3_E(o_WA3_E),
		.o_shmt5_E(o_shmt5_E),
		.o_MemRead_E(o_MemRead_E),
		.o_NZCVWrite_E(o_NZCVWrite_E),
		.o_Flags_E(o_Flags_E),
		.o_Cond_E(o_Cond_E));
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------EXMEM---------------------------------------------------
	stage_register_exmem EXMEM(
	.clk(clk),
	.reset(reset),
	.flush('b0),
	.stop('b0),
	.i_ALUResult_E(i_ALUResult_E),
	.i_WriteData_E(i_WriteData_E),
	.i_WA3_E(i_WA3_E),
	.i_RegWrite_E(i_RegWrite_E),
	.i_MemtoReg_E(i_MemtoReg_E),
	.i_MemWrite_E(i_MemWrite_E),
	.i_MemRead_E(i_MemRead_E),
	.o_ALUOut_M(o_ALUOut_M),
	.o_WriteData_M(o_WriteData_M),
	.o_WA3_M(o_WA3_M),
	.o_RegWrite_M(o_RegWrite_M),
	.o_MemtoReg_M(o_MemtoReg_M),
	.o_MemWrite_M(o_MemWrite_M),
	.o_MemRead_M(o_MemRead_M));
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------MEMWB---------------------------------------------------
	stage_register_memwb MEMWB(
	.clk(clk),
	.reset(reset),
	.flush('b0),
	.stop('b0),
	.i_RegWrite_M(i_RegWrite_M),
	.i_MemtoReg_M(i_MemtoReg_M),
	.i_ALUOut_M(i_ALUOut_M),
	.i_WA3_M(i_WA3_M),
	.o_RegWrite_W(o_RegWrite_W),
	.o_MemtoReg_W(o_MemtoReg_W),
	.o_ALUOut_W(o_ALUOut_W),
	.o_WA3_W(o_WA3_W));
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------IF_WORK------------------------------------------------
	assign input_inst = inst;
	//-------------------------------------------------------------------------------------------------------
	//------------------------------------------------ID_WORK------------------------------------------------
	assign i_WA3_D = o_inst_D[15:12];
	assign i_shmt5_D = o_inst_D[11:7];
	assign i_Flags_D = o_Flags;
	assign i_Cond_D = o_inst_D[31:28];
	assign reg2_D[0] = o_inst_D[3:0];
	assign reg2_D[1] = o_inst_D[15:12];
	assign reg1_D[0] = o_inst_D[19:16];
	assign reg1_D[1] = 'b1111;
	signextmux Extend(.in(o_inst_D[23:0]), .select(immSrc_D), .extVal(i_imm32_D));
	newControlUnit ControlUnit(
	.inst(o_inst_D[27:20]),
	.Rd(o_inst_D[15:12]),
	.RegSrc1(i_RegSrc1_D), 
	.RegSrc2(i_RegSrc2_D), 
	.immSrc(immSrc_D), 
	.BL(BL), 
	.NZCVWrite(i_NZCVWrite_D),
	.ALUSrc1(i_ALUSrc1_D),
	.ALUSrc2(i_ALUSrc2_D),
	.InstOp(i_InstOp_D),
	.PCSrc(i_PCSrc_D),
	.MemWrite(i_MemWrite_D),
	.RegWrite(i_RegWrite_D),
	.MemtoReg(i_MemtoReg_D),
	.MemRead(i_MemRead_D));   
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------HazardUnit----------------------------------------------
	newHazardUnit HazardUnit(
	.read1(reg1_D[i_RegSrc1_D]),
	.read2(reg2_D[i_RegSrc2_D]),
	.o_RegWrite_E(o_RegWrite_E),
	.o_WA3_E(o_WA3_E),
	.o_RegWrite_M(o_RegWrite_M),
	.o_WA3_M(o_WA3_M),
	.o_RegWrite_W(o_RegWrite_W),
	.o_WA3_W(o_WA3_W),
	.i_PCSrc_D(o_PCSrc_E && CondExE),
	.dataHzrdDetected(dataHzrdDetected),
	.ctrlHzrdDetected(ctrlHzrdDetected),
	.stallIFID(stallIFID),
	.flushIFID(flushIFID),
	.flushIDEX(flushIDEX));
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------EX_WORK-------------------------------------------------
	assign ALU_A[0] = o_read1_E;
	assign ALU_A[1] = 0;
	assign ALU_B[0] = o_imm32_E;
	assign ALU_B[1] = o_read2_E << o_shmt5_E;
	assign i_WriteData_E = o_read2_E << o_shmt5_E;
	ALUopdecoder ALUopDecoder(
	.instop (o_InstOp_E),
	.aluop (ALUop_E));
	ALU32bit ALU(
	.inpa (ALU_A[o_ALUSrc1_E]),
	.inpb (ALU_B[o_ALUSrc2_E]),
	.cin (o_Flags_E[1]),
	.aluop (ALUop_E),
	.result (i_ALUResult_E),
	.negative (after_ALU_Flags[3]),
	.zero (after_ALU_Flags[2]),
	.cout (after_ALU_Flags[1]),
	.overflow (after_ALU_Flags[0]));
	newConditionUnit CondUnit(
	.condition(o_Cond_E),
	.curFlags(o_Flags_E),
	.ALUFlags(after_ALU_Flags),
	.flagWrite(o_NZCVWrite_E),
	.execute(CondExE),
	.outputFlags(o_Flags));
	assign i_WA3_E = o_WA3_E;
	assign i_RegWrite_E = o_RegWrite_E && CondExE;
	assign i_MemtoReg_E = o_MemtoReg_E && CondExE;
	assign i_MemWrite_E = o_MemWrite_E && CondExE;
	assign i_MemRead_E = o_MemRead_E && CondExE;
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------MEM_WORK------------------------------------------------
	assign memwrite = o_MemWrite_M;
	assign memaddr = o_ALUOut_M;
	assign writedata = o_WriteData_M;
	assign memread = o_MemRead_M;
	assign be = 'b1111;
	assign i_RegWrite_M = o_RegWrite_M;
	assign i_MemtoReg_M = o_MemtoReg_M;
	assign i_ALUOut_M = o_ALUOut_M;
	assign i_WA3_M = o_WA3_M;
	//--------------------------------------------------------------------------------------------------------
	//------------------------------------------------WB_WORK-------------------------------------------------
	assign Result_W[0] = o_ALUOut_W;
	assign Result_W[1] = readdata;
	assign PCNext[0] = i_wire_pc + 4;
	assign PCNext[1] = i_ALUResult_E;
	assign DataHzrdPCNext[0] = PCNext[o_PCSrc_E && CondExE];
	assign DataHzrdPCNext[1] = i_wire_pc; // current pc, same
	assign reg1 = reg1_D[i_RegSrc1_D];
	assign reg2 = reg2_D[i_RegSrc2_D];
	assign regdst = o_WA3_W;
	assign regsrc = Result_W[o_MemtoReg_W];
	assign R15 = DataHzrdPCNext[dataHzrdDetected];
	registerfile registerFile(.clk(clk), .reset(reset), .reg1(reg1), 
	.reg2(reg2), .regdst(regdst), .regsrc(regsrc), 
	.R15(R15), .BL(BL), .we(o_RegWrite_W), 
	.pc(o_wire_pc), .out1(i_read1_D), .out2(i_read2_D));
	assign i_wire_pc = o_wire_pc;
	assign pc = o_wire_pc;
	//--------------------------------------------------------------------------------------------------------

	
endmodule
