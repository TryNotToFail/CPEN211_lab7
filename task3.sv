module task3(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);
  // your implementation here

  wire [15:0] ram_w_data;
  wire [15:0] ram_r_data;
  reg ram_w_en;
  wire [7:0] ram_addr;
  reg [7:0] ram_w_addr;
  reg [7:0] ram_r_addr;
  
  assign ram_r_addr = ram_addr;
  assign ram_w_addr = ram_addr;

  cpu get_cpu(
		.clk(clk),
		.rst_n(rst_n),
		.start_pc(start_pc),
		.ram_r_data(ram_r_data),
		.out(out),
		.ram_addr(ram_addr),
		.ram_w_en(ram_w_en),
		.ram_w_data(ram_w_data)
		);

  ram write_ram(
		.clk(clk),
		.ram_w_en(ram_w_en),
		.ram_r_addr(ram_r_addr),
		.ram_w_addr(ram_w_addr),
		.ram_w_data(ram_w_data),
		.ram_r_data(ram_r_data)
		);

endmodule: task3

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
	
	assign Z_out = Z_result;
	
	//negative dectect
	assign N_out = data_out[15] ? 1'b1 : 1'b0;
	
	//Overflowed/Underflowed
	assign V_out = (~data_out[15]&val_A[15]&val_B[15])|(data_out[15]&~val_A[15]&~val_B[15]); 

endmodule: datapath

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

endmodule: shifter

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

endmodule: ALU


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
      default: mux_out = mux_out;
    endcase
  end

  assign r_addr = mux_out;
  assign w_addr = mux_out;

endmodule: idecoder

module controller(input clk, input rst_n, input [2:0] opcode, input [1:0] ALU_op, 
 		  input [1:0] shift_op, 
		  output waiting,
                  output [1:0] reg_sel, output [1:0] wb_sel, output w_en,
                  output en_A, output en_B, output en_C, output en_status,
                  output sel_A, output sel_B, output load_ir, output ram_w_en, 
		  output sel_addr, output clear_pc, output load_pc,
		  output load_addr);

`define Reset	 	5'b00000
`define start		5'b00001
`define next		5'b00010
`define update		5'b00011
`define decode		5'b00100
`define immediate	5'b00101
`define load_A		5'b00110
`define load_B		5'b00111
`define ADD		5'b01000
`define CMP		5'b01001
`define AND		5'b01010
`define MVN		5'b01011
`define MOV		5'b01100
`define LOAD_ADDR	5'b01101
`define LDR		5'b01110
`define LOAD_Rd		5'b01111
`define PREP_STR	5'b10000
`define STR		5'b10001
`define HALT		5'b10010
`define write_C 	5'b10011
		
  reg [4:0] state;
  reg [4:0] next_state;
  reg in_waiting;
  reg [1:0] in_reg_sel;
  reg [1:0] in_wb_sel;
  reg in_w_en;
  reg in_en_A;
  reg in_en_B;
  reg in_en_C;
  reg in_en_status;
  reg in_sel_A;
  reg in_sel_B;

  reg in_load_ir;
  reg in_ram_w_en;
  reg in_sel_addr;
  reg in_clear_pc;
  reg in_load_pc;
  reg in_load_addr;

  assign load_ir = in_load_ir;
  assign ram_w_en = in_ram_w_en;
  assign sel_addr = in_sel_addr;
  assign clear_pc = in_clear_pc;
  assign load_pc = in_load_pc;
  assign load_addr = in_load_addr;

  assign waiting = in_waiting;
  assign reg_sel = in_reg_sel;
  assign wb_sel = in_wb_sel;
  assign w_en = in_w_en;
  assign en_A = in_en_A;
  assign en_B = in_en_B;
  assign en_C = in_en_C;
  assign en_status = in_en_status;
  assign sel_A = in_sel_A;
  assign sel_B = in_sel_B;

  always @ (posedge clk) begin
    if(~rst_n) begin
      	next_state = `Reset;
	in_clear_pc = 1'b1;
	in_load_pc = 1'b1;
    end else begin
      case(state)
	`Reset: begin //enters when reset pressed
		next_state = `start;
		in_waiting = 1'b1;
		in_reg_sel = 2'b11;
		in_wb_sel = 2'b00;		
		in_w_en = 1'b0;
		in_en_A = 1'b0;
		in_en_B = 1'b0;
		in_en_C = 1'b0;
		in_en_status = 1'b0;
		in_sel_A = 1'b0;
		in_sel_B = 1'b0;

		in_load_ir = 1'b0;
		in_ram_w_en = 1'b0;
		in_sel_addr = 1'b0;
		in_load_addr = 1'b0;
	end
	`start: begin //always return if reset is not pressed
		next_state = `next;
		in_reg_sel = 2'b10;
		in_wb_sel = 2'b01;
		in_w_en = 1'b0;
		in_en_A = 1'b0;
		in_en_B = 1'b0;
		in_en_C = 1'b0;
		in_en_status = 1'b0;
		in_sel_A = 1'b0;
		in_sel_B = 1'b0;
		
		in_ram_w_en = 1'b0; //turn off ram_w_en
		in_load_pc = 1'b0;
		in_clear_pc = 1'b0;	//clear_pc only 1 at reset and 0 otherwise
		in_sel_addr = 1'b1;	//sel the addr to read from ram
		
	        end 
	`next: begin	
		 next_state = `update;
		 in_load_ir = 1'b1;
		 end
	`update: begin //enters only from next
		  next_state = `decode;
		  in_load_ir = 1'b0;
	 	  in_sel_addr = 1'b0;
		  in_load_pc = 1'b1;
		 end
	`decode: begin //enters only from update
		 in_waiting = 1'b0;
		 in_load_pc = 1'b0;
		  if ({opcode,ALU_op} == 5'b11010) next_state = `immediate; // simple mov only Rn and im8
		  else if ({opcode,ALU_op} == 5'b11100) next_state = `HALT;  //HALT instr
		  else if (({opcode,ALU_op} == 5'b11000) | ({opcode,ALU_op} == 5'b10111)) begin // mov shift or negative mov shift
			next_state = `load_B;
		  end 
		  else next_state = `load_A; //add, cmp, and, ldr and str
		 end //end decode state
	`immediate: begin //enters only through simple move and turn on w_en
			in_reg_sel = 2'b10; //select Rn
			in_wb_sel = 2'b10; //select sximm8
			in_w_en = 1'b1; //enable write to Rn selected
			next_state = `start; //go back to start
		    end
	`load_A: begin 	//enters only through complex
		 	in_reg_sel = 2'b10; //select Rn
			in_en_A = 1'b1; //enable A
			next_state = `load_B; //go to load_B
		 end //end load_A
	`load_B: begin //enters through load_A, mov shift and mov negative
			in_reg_sel = 2'b00; //select Rm
			in_en_A = 1'b0; //keep value A
			in_en_B = 1'b1; //Rm goes into B
			if ({opcode,ALU_op} == 5'b10100) next_state = `ADD;
			else if ({opcode,ALU_op} == 5'b10101) next_state = `CMP;
			else if ({opcode,ALU_op} == 5'b10110) next_state = `AND;
			else if ({opcode,ALU_op} == 5'b10111) next_state = `MVN;
		  	else if ({opcode,ALU_op} == 5'b01100) next_state = `ADD;  //LDR
			else if ({opcode,ALU_op} == 5'b10000) next_state = `ADD;  //STR
			else next_state = `MOV;
		    end //end shifted_B
	`ADD: begin //enters from load_B
		if ({opcode,ALU_op} == 5'b01100) begin //LDR instr
		  in_en_B = 1'b0; //keep value B
		  in_sel_A = 1'b0; //select A
		  in_sel_B = 1'b1; //select sximm5
		  in_en_C = 1'b1; //open C
		  next_state = `LOAD_ADDR;
		end
		else if ({opcode,ALU_op} == 5'b10000) begin //STR instr
		  in_en_B = 1'b0;  //keep value B
		  in_sel_A = 1'b0; //select A
		  in_sel_B = 1'b1; //select sximm5
		  in_en_C = 1'b1;  //open C
		  next_state = `LOAD_ADDR;
		end
		else //Do addition
		begin
		  in_en_B = 1'b0; //keep value B
		  in_sel_A = 1'b0; //select A
		  in_sel_B = 1'b0; //select B
		  in_en_C = 1'b1; //open C
		  next_state =  `write_C;
		end
	      end
	`CMP: begin //enters from load_B, only this state has en_status
		in_en_B = 1'b0; // keep value B
		in_sel_A = 1'b0; //select A
		in_sel_B = 1'b0; //select B
		in_en_C = 1'b1; //open C
		in_en_status = 1'b1; //turn on status
		next_state = `write_C;
	      end
	`AND: begin //enters from load_B
		in_en_B = 1'b0;
		in_sel_A = 1'b0; //select A
		in_sel_B = 1'b0; //select B
		in_en_C = 1'b1; //open C
		next_state = `write_C;
 	      end
	`MVN: begin //enters from load_B, negative shifted value B
		in_en_B = 1'b0;
		in_sel_A = 1'b1; //set value A to zero
		in_sel_B = 1'b0; //select value B
		in_en_C = 1'b1;
		next_state = `write_C;
	      end
	`MOV: begin //enters from load_B, shifted value B
		in_en_B = 1'b0;
		in_sel_A = 1'b1; //set value A to zero
		in_sel_B = 1'b0; //select value B
		in_en_C = 1'b1;
		next_state = `write_C;
   	      end
	`LOAD_ADDR: begin //enters only through ADD
		in_en_C = 1'b0; //keep the value C
		in_load_addr = 1'b1;  //let addr pass though
		in_sel_addr = 1'b0;
		if ({opcode, ALU_op} == 5'b01100) next_state = `LDR;
		else if({opcode, ALU_op} == 5'b10000) next_state = `LOAD_Rd;
		end
	`LDR: begin //enters from load_B 
		//load from memory to register
		next_state = `write_C;
		in_load_addr = 1'b0; //keep the addr data
	      end
	`LOAD_Rd: begin //enters from load_addr
		next_state = `PREP_STR;
		in_load_addr = 1'b0;
		in_reg_sel = 2'b01; //selects Rd
		in_en_B = 1'b1;  //load Rd into reg B
	      end
	`PREP_STR: begin //enters only from load_Rd
		next_state = `STR;
		in_sel_A = 1'b1; //select 16'b0 avoid 
		in_sel_B = 1'b0; //select Rd
		in_en_B = 1'b0;  //keep the value reg B
		in_en_C = 1'b1; //produce value C
		end
	`STR: begin //enters only from Prep_STR only ram_w_en is turn on
		in_ram_w_en = 1'b1;
		next_state = `start;
		end
	`HALT: next_state = `HALT;
	`write_C: begin //last state and the only state when w_en is 1
		 	if({opcode, ALU_op} == 5'b01100) begin
			 next_state = `start;
			 in_reg_sel = 2'b01;	//select Rd
			 in_wb_sel = 2'b11;	//select mdata which is the addtion of sximm5 and Rn 
			 in_w_en = 1'b1;	// turn on w_en
			end
			in_w_en = 1'b1; // turn on w_en
			in_reg_sel = 2'b01; // select the reg Rd
			in_wb_sel = 2'b00; // select the value output
			next_state = `start; //go back to start
		  end
	default: begin
		  state = `start;
		  next_state = `start;
		 end
      endcase
    end
   state <= next_state;
 end
endmodule: controller

module cpu(input clk, input rst_n, input [7:0] start_pc, input [15:0] ram_r_data,
 	   output [15:0] out,
    	   output [7:0] ram_addr, output ram_w_en, output [15:0] ram_w_data);

  wire [1:0] reg_sel;
  wire [2:0] opcode;
  wire [1:0] ALU_op;
  wire [1:0] shift_op;
  wire [15:0] sximm5;
  wire [15:0] sximm8;
  wire [2:0] r_addr;
  wire [2:0] w_addr;

  wire waiting;
  wire en_A;
  wire en_B;
  wire en_C;
  wire en_status;
  wire w_en;
  reg [1:0] wb_sel;
  wire sel_A;
  wire sel_B;
  wire load_ir;
  reg sel_addr;
  reg clear_pc;
  reg load_pc;
  reg load_addr;
  reg [7:0] add_pc;
  reg [15:0] instr_out;

  reg Z;
  reg N;
  reg V;

  assign out = ram_w_data;

 //muxes start_pc
  reg [7:0] next_pc;
  assign next_pc = clear_pc ? start_pc : add_pc;

  //pc adder
  reg [7:0] pc;
  assign add_pc = pc + 1'b1;

  always @ (posedge clk) begin
    //Program counter
    if (load_pc) pc = next_pc;
    else pc = pc;

    //Intrs reg
    if(load_ir) instr_out = ram_r_data;
    else instr_out = instr_out;
  end

  //Data address always blk
  reg [7:0] data_addr_out;
  wire [7:0] data_out;
  assign data_out = ram_w_data[7:0]; 

  always @ (posedge clk) begin
    if (load_addr) data_addr_out = data_out;
    else data_addr_out = data_addr_out;   
  end

  //Mux determine the ram address
  assign ram_addr = sel_addr ? pc : data_addr_out;

  idecoder decode_version(
			.ir(ram_r_data),
			.reg_sel(reg_sel),
			.opcode(opcode),
			.ALU_op(ALU_op),
			.shift_op(shift_op),
			.sximm5(sximm5),
			.sximm8(sximm8),
			.r_addr(r_addr),
			.w_addr(w_addr)
			);

  controller control(
			.clk(clk),
			.rst_n(rst_n),
			.opcode(opcode),
			.ALU_op(ALU_op),
			.shift_op(shift_op),
			.waiting(waiting),
			.reg_sel(reg_sel),
			.wb_sel(wb_sel),
			.w_en(w_en),
			.en_A(en_A),
			.en_B(en_B),
			.en_C(en_C),
			.en_status(en_status),
			.sel_A(sel_A),
			.sel_B(sel_B),
			.load_ir(load_ir),
			.ram_w_en(ram_w_en),
			.sel_addr(sel_addr),
			.clear_pc(clear_pc),
			.load_pc(load_pc),
			.load_addr(load_addr)
			);
  
  datapath modified_datapath(
			.clk(clk),
			.mdata(ram_r_data),
			.pc(pc),
			.wb_sel(wb_sel),
			.w_addr(w_addr),
			.w_en(w_en),
			.r_addr(r_addr),
			.en_A(en_A),
			.en_B(en_B),
			.shift_op(shift_op),
			.sel_A(sel_A),
			.sel_B(sel_B),
			.ALU_op(ALU_op),
			.en_C(en_C),
			.en_status(en_status),
			.sximm8(sximm8),
			.sximm5(sximm5),
			.datapath_out(ram_w_data),
			.Z_out(Z),
			.N_out(N),
			.V_out(V)
			);

endmodule: cpu

module ram(input clk, input ram_w_en, input [7:0] ram_r_addr, input [7:0] ram_w_addr,
           input [15:0] ram_w_data, output reg [15:0] ram_r_data);
    reg [15:0] m[255:0];
    always_ff @(posedge clk) begin
        if (ram_w_en) m[ram_w_addr] <= ram_w_data;
        ram_r_data <= m[ram_r_addr];
    end
    initial $readmemb("ram_init.txt", m);
endmodule: ram

