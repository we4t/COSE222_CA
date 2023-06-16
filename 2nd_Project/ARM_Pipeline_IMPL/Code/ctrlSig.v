module newControlUnit(
   input [27:20] inst,
   input [15:12] Rd,
   
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
      if(inst[27] || Rd == 4'b1111) begin //B, BL
         if(~inst[24] || Rd == 4'b1111) begin //B
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
         assign {RegSrc1, RegSrc2, immSrc, BL, NZCVWrite, ALUSrc1, ALUSrc2, InstOp, PCSrc, MemWrite, MemRead, RegWrite, MemtoReg} = control;

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
  input i_PCSrc_D,
  output dataHzrdDetected,
  output ctrlHzrdDetected,
  output stallIFID,
  output flushIFID,
  output flushIDEX);

   assign dataHzrdDetected = (o_RegWrite_E && (read1 == o_WA3_E || read2 == o_WA3_E)) ||
                            (o_RegWrite_M && (read1 == o_WA3_M || read2 == o_WA3_M)) ||
                            (o_RegWrite_W && (read1 == o_WA3_W || read2 == o_WA3_W));
   assign ctrlHzrdDetected = i_PCSrc_D;
   assign stallIFID = dataHzrdDetected;
   assign flushIFID = ctrlHzrdDetected;
   assign flushIDEX = ctrlHzrdDetected || dataHzrdDetected;

endmodule


module newConditionUnit(
  // decide to make this instruction valid or not
  // also, write flags if it is necessary
  // thus, this unit do two things : update flag, check the flag and condition 
  // only check the zero flag, since it works well without any other opcodes.

  input [31:28] condition,
  input [3:0] curFlags,
  input [3:0] ALUFlags,
  input flagWrite,

  output execute,
  output[3:0] outputFlags
);


	assign execute = (condition == 4'b1110) ? 1'b1 : ((condition[28] ^ curFlags[2]) ? 1'b1 : 1'b0);
	assign outputFlags = (execute && flagWrite) ? ALUFlags : curFlags;


endmodule