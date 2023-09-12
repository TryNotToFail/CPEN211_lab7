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
		
  reg [3:0] state;
  reg [3:0] next_state;
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
endmodule
