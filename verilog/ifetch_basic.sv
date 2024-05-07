module ifetch_basic (
    input clock,  // system clock
    input reset,  // system reset

    //          ***INPUTS***

    input if_valid,  // only pop the the instruction buffer head if valid

    // Program Counter Inputs   
    input [`XLEN-1:0] certain_branch_pc,
    input certain_branch_req,

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

    // Debug 

    output logic [`XLEN-1:0] PC_reg,
    output logic [`XLEN-1:0] n_PC_reg,

    output logic waiting_on_inst,
    output logic n_waiting_on_inst,

    output IF_ID_PACKET n_if_packet,

    output logic pending_branch,
    n_pending_branch,
    output logic [`XLEN-1:0] pending_branch_pc,
    n_pending_branch_pc

);

  always_comb begin

    /* Initialize, might be cleared later */
    if (certain_branch_req) begin
      n_pending_branch = 1;
      n_pending_branch_pc = certain_branch_pc;
    end else begin
      n_pending_branch = pending_branch;
      n_pending_branch_pc = pending_branch_pc;
    end

    /* if_valid is true if dispatch accepted the packet in if_packet */
    if (if_valid) begin
      if (!waiting_on_inst) begin
        if (pending_branch) begin
          n_PC_reg = pending_branch_pc;
          /* Used up pending branch */
          n_pending_branch = 0;
          n_pending_branch_pc = 0;
        end else begin
          n_PC_reg = PC_reg + 4;
        end

        n_waiting_on_inst = 1;

        /* Clear the output, last packet was accepted */
        n_if_packet.inst = `NOP;
        n_if_packet.PC = PC_reg;
        n_if_packet.NPC = PC_reg + 4;
        n_if_packet.valid = 0;
      end else if (waiting_on_inst) begin

        /* if_packet accepted, but still waiting on an inst */

        n_PC_reg = PC_reg;

        if (Icache2proc_data_valid) begin
          // pushing to the tail of inst buffer
          /* Instruction just came it, take it */
          n_waiting_on_inst = 0;
          n_if_packet.inst = PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];
          n_if_packet.PC = PC_reg;
          n_if_packet.NPC = PC_reg + 4;
          n_if_packet.valid = 1;
        end else if (!Icache2proc_data_valid) begin
          /* Still waiting, keep packet as nop */
          n_waiting_on_inst = 1;
          n_if_packet.inst = `NOP;
          n_if_packet.PC = PC_reg;
          n_if_packet.NPC = PC_reg + 4;
          n_if_packet.valid = 0;
        end
      end
    end else begin
      /* If_packet not yet accepted, so try again next cycle */

      n_waiting_on_inst = waiting_on_inst;
      n_PC_reg = PC_reg;
      n_if_packet = if_packet;

    end


    /* invalidate any future instructions if branch arrived */
    if (certain_branch_req || pending_branch) begin
      n_if_packet.inst = `NOP;
      n_if_packet.PC = 0;
      n_if_packet.NPC = 0;
      n_if_packet.valid = 0;
    end


  end

  // synopsys sync_set_reset "reset"
  always_ff @(posedge clock) begin
    if (reset) begin
      waiting_on_inst <= 1;
      PC_reg <= 0;
      if_packet <= 0;
      pending_branch <= 0;
      pending_branch_pc <= 0;
    end else begin
      PC_reg <= n_PC_reg;
      waiting_on_inst <= n_waiting_on_inst;
      if_packet <= n_if_packet;

      pending_branch <= n_pending_branch;
      pending_branch_pc <= n_pending_branch_pc;

`ifdef DEBUG_PRINT
      if (if_packet.valid) begin
        $display("[IF] fetched valid instruction, PC %p", if_packet.PC);
      end else begin
        $display("[IF] no valid instruction fetched");
      end
`endif

    end
  end

  // address of the instruction we're fetching (64 bit memory lines)
  // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
  assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};
endmodule
