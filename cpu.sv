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
endmodule

