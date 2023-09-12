module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, 
	    output [15:0] ALU_out, output Z);

reg [15:0] ALU_out1;
reg Z1;

assign ALU_out = ALU_out1;
assign Z = Z1;

always @(*)
  begin
   case(ALU_op)
	2'b00: ALU_out1 = val_A+val_B;
	2'b01: ALU_out1 = val_A-val_B;
	2'b10: ALU_out1 = val_A&val_B;
	2'b11: ALU_out1 = ~val_B;
   endcase

   case(ALU_out1)
	16'b0: Z1 = 1;
  	default: Z1 = 0;
   endcase
end
endmodule
