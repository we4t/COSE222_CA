module signalcontrol(
	input[11:0] flags,
	input zero,
	output reg [2:0] total,
	output reg [19:0] s2,
	output reg [19:0] s3,
	output reg [19:0] s4);
	
 always @ (*) begin
  if((flags[11]&flags[10]&flags[9])||(flags[8]^zero)) begin
   if(flags[7]) begin //B, BL
    if(~flags[4]) begin //B
    s2=20'b00010110001001000100;
    s3=20'bxxxxxxxxxxxxxxxxxxxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=2;
    end
   else begin //BL
    s2=20'b00011001001001000100;
    s3=20'b00010101xxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3;
   end
  end
  else if(flags[6]) begin //LDR, STR
   s2={10'b0001010101,(flags[5]==1 ? 2'b11 : 2'b10),(flags[3]==1 ? 4'b0100 : 4'b0010), 3'b001, (flags[0]==1 ? 1'b0 : 1'b1)};
  
   if(~flags[0]) begin //STR
    s3=20'b1000xxxxxxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3;
   end
   else begin //LDR
    s3=20'b0010xxxxxxxxxxxx0xxx;
    s4=20'b00010000xxxxxxxx0xxx;
    total=4;
   end 
  end
  
  else //Else
   case(flags[4:1])/*
      0 : //AND
      1 : //EOR
      2 : //SUB
      4 : //ADD
      5 : //ADC
      6 : //SBC
      12 : //ORR*/
    10 : begin //CMP
    s2={10'b0001010101, (flags[5]==1 ? 2'b10 : 2'b11), 8'b00101000};
    s3=20'bxxxxxxxxxxxxxxxxxxxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=2;
    end
    13 : begin //MOV
    s2={10'b0001010110, (flags[5]==1 ? 2'b10 : 2'b11), 4'b0100 ,flags[0], 3'b000};
    s3=20'b00010001xxxxxxxx0xxx;
    s4=20'bxxxxxxxxxxxxxxxxxxxx;
    total=3; 
    end
    default : begin //ALU
     s2={10'b0001010101, (flags[5]==1 ? 2'b10 : 2'b11), flags[4:0], 3'b000};
     s3=20'b00010001xxxxxxxx0xxx;
     s4=20'bxxxxxxxxxxxxxxxxxxxx;
     total=3;
    end
   endcase
  end
  else begin //Recovery
   s2=20'b00010101xxxxxxxx0xxx;
   s3=20'bxxxxxxxxxxxxxxxxxxxx;
   s4=20'bxxxxxxxxxxxxxxxxxxxx;
   total=2;
  end
 end
endmodule 

module oneAdder(
	input clk,
	input reset,
	input [2:0] current,
	output reg[2:0] regout);
	
	wire last;
	
	assign last=current==regout;
	
	always @ (posedge clk,posedge reset)
	if(reset)
		regout<='b000;
	else if(last)
		regout<='b000;
	else
		regout<=regout+'b001;	
endmodule

module signalunit
	(input clk,
	 input reset,
	 input[11:0] flags,
	 input zero,
	 output Mwrite,
	 output IRwrite,
	 output Mread,
	 output regwrite,
	 output[1:0] regdst,
	 output[1:0] regsrc,
	 output[1:0] ALUsrcA,
	 output[1:0] ALUsrcB,
	 output[3:0] ALUop,
	 output NZCVwrite,
	 output [1:0] immsrc,
	 output regbdst);
  
  wire [19:0] s [4:0];
  
  assign s[0] = 20'b01110110000101000xxx;
  assign s[1] = 20'b0000xxxx000000100xxx;
  
  wire [2:0] total;
  wire [2:0] step;
  
  oneAdder Step(.clk (clk), .reset (reset), .current (total), .regout (step));
  
  signalcontrol bringSignal(
	.flags (flags),
	.zero (zero),
	.total (total),
	.s2 (s[2]),
	.s3 (s[3]),
	.s4 (s[4]));  
  
  assign Mwrite=s[step][19];
  assign IRwrite=s[step][18];
  assign Mread=s[step][17];
  assign regwrite=s[step][16];
  assign regdst=s[step][15:14];
  assign regsrc=s[step][13:12];
  assign ALUsrcA=s[step][11:10];
  assign ALUsrcB=s[step][9:8];
  assign ALUop=s[step][7:4];
  assign NZCVwrite=s[step][3];
  assign immsrc=s[step][2:1];
  assign regbdst=s[step][0];
  
endmodule

module newHazardUnit(
    // Data Hazard
    input [3:0] read1, // reg1[i_RegSrc1_D]
    input [3:0] read2, // reg2[i_RegSrc2_D]
    input o_RegWrite_E,
    input [3:0] o_WA3_E,
    input o_RegWrite_M,
    input [3:0] o_WA3_M,
    input o_RegWrite_W,
    input [3:0] o_WA3_W,
    
    // Control Hazard
    input i_PCSrc_D,

    // Output. if dataHzrd == 1, Detected.
    output dataHzrdDetected,
    output ctrlHzrdDetected);

    assign dataHzrdDetected = (o_RegWrite_E && (read1 == o_WA3_E || read2 == o_WA3_E)) || (o_RegWrite_M && (read1 == o_WA3_M || read2 == o_WA3_M)) || (o_RegWrite_W && (read1 == o_WA3_W || read2 == o_WA3_W));
    assign ctrlHzrdDetected = i_PCSrc_D;

endmodule

module newControlUnit(
	input [31:20] inst,
	input [3:0] Flags,
	
	// IDRF
	output RegSrc1,
	output RegSrc2,
	output [1:0]immSrc,
	output BL,
	
	// EXE
	output NZCVWrite,
	output ALUSrc1,
	output ALUSrc2,
	output [3:0] InstOp,
	output PCSrc,
	
	// MEM
	output MemWrite,
	output MemRead,
	
	// WB
	output RegWrite,
	output MemtoReg);
	
	reg [16:0] control;
	
	always @ (*) begin
		if ((inst[31] && inst[30] && inst[29]) || (Flags[3] ^ inst[28])) begin // AL or EQ or NE 
			if(inst[27]) begin //B, BL
				if(~inst[24]) begin //B
					control = 17'b10100010010010000;
					// control = 17'b1010001001001xxxx;
				end
				else begin //BL
					control = 17'b10101010010010000;
					// control = 17'b1010101001001xxxx;
				end
			end
			
			else if(inst[26]) begin //LDR, STR
				if(~inst[20]) begin //STR
					control = {7'b0101001, inst[25], (inst[23] ? 4'b0100 : 4'b0010), 5'b01000};
					// control = {7'b0101001, inst[25], (inst[23] ? 4'b0100 : 4'b0010), 5'b010xx};
				end
				else begin //LDR
					control = {7'b0001001, inst[25], (inst[23] ? 4'b0100 : 4'b0010), 5'b00111};
				end
			end
			
			else // ALU
				case(inst[24:21])/*
					0 : //AND
					1 : //EOR
					2 : //SUB
					4 : //ADD
					5 : //ADC
					6 : //SBC
					12 : //ORR*/
					10 : begin //CMP
						control = {7'b0000011, ~inst[25], 9'b001000000};
						// control = {7'b0000011, ~inst[25], 9'b00100xxxx};
					end
					13 : begin //MOV
						control = {5'b00000, inst[20], 1'b0, ~inst[25], 9'b01000010};
						// control = {5'b00000, inst[20], 1'b0, ~inst[25], 9'b01000xx10};
					end
					default : begin //ALU
						control = {5'b00000, inst[20], 1'b0, ~inst[25], inst[24:21], 5'b00010};
						// control = {5'b00000, inst[20], 1'b0, ~inst[25], inst[24:21], 5'b0xx10};
					end
				endcase
			end
		else begin // Recovery
			control = 17'b00000000000000000;
		end
	end
	
	assign {RegSrc1, RegSrc2, immSrc, BL, NZCVWrite, ALUSrc1, ALUSrc2, InstOp, PCSrc, MemWrite, MemRead, RegWrite, MemtoReg} = control;
	
endmodule

