module shifter(input [15:0] shift_in, input [1:0] shift_op,
	       output reg [15:0] shift_out);

  always_comb begin
	case(shift_op)
	  2'b00: shift_out = shift_in;
	  2'b01: shift_out = shift_in << 1;
	  2'b10: shift_out = shift_in >> 1;
	  2'b11: begin 
		 shift_out = shift_in >> 1;
		 shift_out[15] = shift_in[15];
		 end
	 endcase 
   end

endmodule
