`include "verilog/sys_defs.svh"

// Complex icache implementation, not used 

// the size of our FIFO instruction buffer
`define INST_BUF_SIZE 8
`define INST_INDEX_SIZE $clog2(`INST_BUF_SIZE)

module ifetch (
    input clock,  // system clock
    input reset,  // system reset

    //          ***INPUTS***

    input if_valid,  // only pop the the instruction buffer head if valid

    // Program Counter Inputs   
    input [`XLEN-1:0] certain_branch_pc,
    input certain_branch_req,

    input [`XLEN-1:0] rob_target_pc,
    input rob_target_req,

    input [`XLEN-1:0] branch_pred_pc,
    input branch_pred_req,

    // Icache Request Response
    input [63:0] Icache2proc_data,  // data coming back from Instruction memory
    input Icache2proc_data_valid,  // data coming back from Instruction memory

    //          *** OUTPUTS ***

    // To cache to get inst from memory
    output [`XLEN-1:0] proc2Icache_addr,
    // To decode
    output IF_ID_PACKET if_packet,

    //          *** DEBUG ***
    output [3:0] req_debug,
    output [3:0] gnt_debug,
    output [`XLEN-1:0] PC_reg_debug,
    output waiting_on_inst_debug,
    output IF_ID_PACKET inst_buffer_debug[`INST_BUF_SIZE-1:0],
    output IF_ID_PACKET n_inst_buffer_debug[`INST_BUF_SIZE-1:0],
    output [`INST_INDEX_SIZE-1:0] inst_buffer_tail_debug,
    output new_inst_accepted_debug
);

  logic [`XLEN-1:0] PC_reg;  // PC we are currently fetching
  logic [`XLEN-1:0] n_PC_reg;  // PC we are currently fetching

  logic waiting_on_inst;
  logic n_waiting_on_inst;
  assign waiting_on_inst_debug = waiting_on_inst;

  logic n_new_inst_accepted;
  assign new_inst_accepted_debug = n_new_inst_accepted;

  IF_ID_PACKET inst_buffer[`INST_BUF_SIZE-1:0];
  IF_ID_PACKET n_inst_buffer[`INST_BUF_SIZE-1:0];
  logic inst_buffer_full;
  logic n_inst_buffer_full;

  assign inst_buffer_debug   = inst_buffer;
  assign n_inst_buffer_debug = n_inst_buffer;

  logic[`INST_INDEX_SIZE-1:0] inst_buffer_tail; // the index of the last non-default element in the buffer
  assign inst_buffer_tail_debug = inst_buffer_tail;
  logic [`INST_INDEX_SIZE-1:0] n_inst_buffer_tail;

  assign if_packet = inst_buffer[0];

  logic [3:0] req;
  assign req = {certain_branch_req, rob_target_req, branch_pred_req, 1'b1};
  logic [3:0] gnt;

  // DEBUG SIGNALS
  assign req_debug = req;
  assign gnt_debug = gnt;
  assign PC_reg_debug = PC_reg;

  psel_gen #(
      .WIDTH(4),
      .REQS (1)
  ) if_psel (
      .req(req),
      .gnt(gnt),
      .gnt_bus(),
      .empty()
  );

  always_comb begin

    // if cache can receive new request and the buffer not full, request the next pc
    if (!waiting_on_inst) begin
      unique case (gnt)
        4'b1000: n_PC_reg = certain_branch_pc;
        4'b0100: n_PC_reg = rob_target_pc;
        4'b0010: n_PC_reg = branch_pred_pc;
        4'b0001: n_PC_reg = PC_reg + 4;
        default: n_PC_reg = 32'hdeadbeef;
      endcase

      n_waiting_on_inst = 1;
      n_inst_buffer_full = inst_buffer_full;
      n_inst_buffer_tail = inst_buffer_tail;

      n_inst_buffer[n_inst_buffer_tail].inst = `NOP;
      n_inst_buffer[n_inst_buffer_tail].PC = 0;
      n_inst_buffer[n_inst_buffer_tail].NPC = 0;
      n_inst_buffer[n_inst_buffer_tail].valid = 0;
    end else if (waiting_on_inst) begin
      n_PC_reg = PC_reg;
      if (Icache2proc_data_valid && waiting_on_inst) begin
        // pushing to the tail of inst buffer
        n_waiting_on_inst = 0;

        // Determining the index of the new element or whether rejected
        if (if_valid || inst_buffer_full) begin
          n_inst_buffer_tail = inst_buffer_tail;
          n_inst_buffer_full = inst_buffer_full;
        end else begin  // appending new inst
          if (inst_buffer_tail == `INST_BUF_SIZE - 1) begin
            n_inst_buffer_tail = inst_buffer_tail;
            n_inst_buffer_full = 1;
          end else begin
            n_inst_buffer_tail = inst_buffer_tail + 1;
            n_inst_buffer_full = 0;
          end
        end

        if (!inst_buffer_full) begin
          n_inst_buffer[n_inst_buffer_tail].inst = PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];
          n_inst_buffer[n_inst_buffer_tail].PC = PC_reg;
          n_inst_buffer[n_inst_buffer_tail].NPC = PC_reg + 4;
          n_inst_buffer[n_inst_buffer_tail].valid = 1;
          n_new_inst_accepted = 1;
        end else begin
          n_new_inst_accepted = 0;
        end
      end else if (!Icache2proc_data_valid && waiting_on_inst) begin
        n_waiting_on_inst  = waiting_on_inst;
        n_inst_buffer_tail = inst_buffer_tail;
        n_inst_buffer_full = inst_buffer_full;

        if (!inst_buffer_full) begin
          n_inst_buffer[n_inst_buffer_tail].inst = `NOP;
          n_inst_buffer[n_inst_buffer_tail].PC = PC_reg;
          n_inst_buffer[n_inst_buffer_tail].NPC = PC_reg + 4;
          n_inst_buffer[n_inst_buffer_tail].valid = 0;
          n_new_inst_accepted = 1;
        end
      end
    end

    if (if_valid) begin
      // shift the buffer to the left by one
      for (int i = 0; i < n_inst_buffer_tail; i++) begin
        n_inst_buffer[i] = inst_buffer[i+1];
      end
      for (int i = n_inst_buffer_tail + 1; i < `INST_BUF_SIZE; i++) begin
        n_inst_buffer[i].inst <= `NOP;
        n_inst_buffer[i].PC <= 0;
        n_inst_buffer[i].NPC <= 0;
        n_inst_buffer[i].valid <= 0;
      end
    end else begin
      n_inst_buffer = inst_buffer;
    end
  end

  // synopsys sync_set_reset "reset"
  always_ff @(posedge clock) begin
    if (reset) begin
      waiting_on_inst <= 0;
      PC_reg <= 0;
      for (int i = 0; i < `INST_BUF_SIZE; i++) begin
        inst_buffer[i].inst <= `NOP;
        inst_buffer[i].PC <= 0;
        inst_buffer[i].NPC <= 0;
        inst_buffer[i].valid <= 0;
      end
      inst_buffer_tail <= 0;
      inst_buffer_full <= 0;
    end else begin
      PC_reg <= n_PC_reg;
      inst_buffer <= n_inst_buffer;
      inst_buffer_tail <= n_inst_buffer_tail;
      inst_buffer_full <= n_inst_buffer_full;
      waiting_on_inst <= n_waiting_on_inst;
    end
  end

  // address of the instruction we're fetching (64 bit memory lines)
  // mem always gives us 8=2^1 words, so ignore the last bit
  assign proc2Icache_addr = {PC_reg[`XLEN-1:2], 2'b0};
endmodule
