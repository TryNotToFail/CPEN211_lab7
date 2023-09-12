module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
		input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out, output Z_out, output N_out, output V_out);

	reg [15:0] w_data;
	wire [15:0] r_data;
	reg [15:0] A_out;
	reg [15:0] B_out;
	wire [15:0] shift_out;
	wire [15:0] val_A;
	wire [15:0] val_B;
	wire [15:0] ALU_out;
	reg [15:0] data_out;
	reg Z_result;
	wire Z;

	assign datapath_out = data_out;
	
	//first muxes select in and out
	always @ (*) begin
	case(wb_sel)
	  2'b11: w_data = mdata;
	  2'b10: w_data = sximm8;
	  2'b01: w_data = {8'b0,pc};
	  2'b00: w_data = data_out;
	endcase
	end
	
	//regfile
	regfile got_file(
			.w_data(w_data),
			.w_addr(w_addr),
			.w_en(w_en),
			.r_addr(r_addr),
			.clk(clk),
			.r_data(r_data)
			);
	
	//register A, B, C, Z, N, V
	always @ (posedge clk) begin
		if (en_A) A_out <= r_data;
		if (en_B) B_out <= r_data;
		if (en_C) data_out <= ALU_out;
		if (en_status) Z_result <= Z;
	end
	
	//shifter 
	shifter shifted_value(
				.shift_in(B_out),
				.shift_op(shift_op),
				.shift_out(shift_out)
			      );
								
	//Muxes A and B
	assign val_A = sel_A ? 16'b0 : A_out;
	assign val_B = sel_B ? sximm5 : B_out;
	
	//ALU_op
	ALU combination(
			.val_A(val_A),
			.val_B(val_B),
			.ALU_op(ALU_op),
			.ALU_out(ALU_out),
			.Z(Z)
			);
	
	//negative dectect
	assign N_out = data_out[15] ? 1'b1 : 1'b0;
	
	//Overflowed/Underflowed
	assign V_out = (~data_out[15]&val_A[15]&val_B[15])|(data_out[15]&~val_A[15]&~val_B[15]); 

endmodule

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

module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out, output Z);

reg [15:0] ALU_out1;
reg Z1;

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

assign ALU_out = ALU_out1;
assign Z = Z1;

endmodule

module regfile(input logic clk, input logic [15:0] w_data, input logic [2:0] w_addr, input logic w_en, input logic [2:0] r_addr, output logic [15:0] r_data);
    logic [15:0] m[0:7];
    assign r_data = m[r_addr];
    always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule
