module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);
 
  //all straight wire
  assign opcode = ir[15:13];
  assign ALU_op = ir[12:11];

  //shift_op
  reg [1:0] in_shift_op;
  assign shift_op = in_shift_op;

  always @ (*) begin
    if (({opcode,ALU_op} == 5'b01100) | ({opcode,ALU_op} == 5'b10000)) in_shift_op = 2'b00;
    else in_shift_op = ir[4:3];
  end

  //sign extend sximm 5 and sximm8
  reg [10:0] imm5;
  reg [7:0] imm8;

  assign imm5 = ir[4] ? 11'b11111111111 : 11'b0;
  assign imm8 = ir[7] ? 8'b11111111 : 8'b0;

  assign sximm5 = {imm5, ir[4:0]};
  assign sximm8 = {imm8, ir[7:0]};
  
  //reg_sel mux
  wire [2:0] Rn = ir[10:8];
  wire [2:0] Rd = ir[7:5];
  wire [2:0] Rm = ir[2:0];
  reg [2:0] mux_out;
  
  always @ (*) begin
    case(reg_sel)
      2'b10: mux_out = Rn;
      2'b01: mux_out = Rd; 
      2'b00: mux_out = Rm;
      2'b11: mux_out = mux_out;
    endcase
  end

  assign r_addr = mux_out;
  assign w_addr = mux_out;

endmodule
