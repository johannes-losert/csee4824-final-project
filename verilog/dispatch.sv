module dispatch (
    input clock,
    input reset,
    input IF_ID_PACKET if_id_packet,  // passed from ifetch stage 

    // Input from CDB (a Phys reg to mark ready)
    input logic                      cdb_broadcast_en,  //  broadcast from CDB
    input logic [`PHYS_REG_IDX_SZ:0] cdb_ready_reg,     //  ready register from RS

    // Rollback signal
    input logic                 rollback,             //  will clear ROB and revert Map Table 
    input logic [`REG_IDX_SZ:0] rollback_immune_reg,  //  will not clear these registers
    input PREG                  rollback_immune_preg,

    // Input from retire stage
    input logic                 retire_move_head,  //  retire signal to move head
    input PREG                  retire_phys_reg,
    input logic [`REG_IDX_SZ:0] retire_arch_reg,

    // Input from complete stage, freeing reservation station (TODO make into enum)
    input logic [   `NUM_FU_ALU-1:0] free_alu,    //  free ALU entry
    input logic [  `NUM_FU_MULT-1:0] free_mult,   //  free MULT entry
    input logic [  `NUM_FU_LOAD-1:0] free_load,   //  free LOAD entry
    input logic [ `NUM_FU_STORE-1:0] free_store,  //  free STORE entry
    input logic [`NUM_FU_BRANCH-1:0] free_branch, //  free BRANCH entry

    output ID_IS_PACKET id_packet,  // outputs  
    output logic        stall,      //  stall signal to ifetch stage (structural hazard)

    // Output to retire stage
    output logic [$clog2(`ROB_SZ)-1:0] rob_head_idx,

    // Debug outputs
    output logic rob_full,
    output logic rs_full,
    output logic free_list_empty
);

  // Declare Map Table

  ID_IS_PACKET decoded_packet;
  /* Don't get passed along, only needed to decide whether to mark in RS */
  logic has_rs1, has_rs2;

  decoder decoder1 (
      // inputs
      .inst(if_id_packet.inst),
      .valid(if_id_packet.valid),
      // outputs
      .opa_select(decoded_packet.opa_select),
      .opb_select(decoded_packet.opb_select),
      .has_dest(decoded_packet.has_dest),
      .has_rs1(has_rs1),
      .has_rs2(has_rs2),
      .alu_func(decoded_packet.alu_func),
      .rd_mem(decoded_packet.rd_mem),
      .wr_mem(decoded_packet.wr_mem),
      .cond_branch(decoded_packet.cond_branch),
      .uncond_branch(decoded_packet.uncond_branch),
      .csr_op(decoded_packet.csr_op),
      .halt(decoded_packet.halt),
      .illegal(decoded_packet.illegal),
      .function_type(decoded_packet.function_type)
  );

  // Declare Map Table

  // Inputs to getting from map table
  logic [`REG_IDX_SZ:0] mt_arch_reg1_idx;
  logic [`REG_IDX_SZ:0] mt_arch_reg2_idx;

  // Outputs from getting from map table
  PREG mt_preg1_out;
  PREG mt_preg2_out;

  logic [`REG_IDX_SZ:0] mt_arch_dest_idx;
  logic mt_set_dest_enable;
  logic [`PHYS_REG_IDX_SZ:0] mt_new_dest_pr_idx;

  PREG mt_old_dest_pr;

  map_table map_table_0 (
      .clk  (clock),
      .reset(reset),

      /* GET operation 1 (opa) */
      // input
      .arch_reg1_idx(mt_arch_reg1_idx),
      //output
      .preg1_out(mt_preg1_out),

      /* GET operation 2 (opb) */
      // input
      .arch_reg2_idx(mt_arch_reg2_idx),
      // output
      .preg2_out(mt_preg2_out),

      /* GET/SET operation (get old dest, replace with new dest) */
      // inputs
      .arch_dest_idx(mt_arch_dest_idx),
      .set_dest_enable(mt_set_dest_enable),
      .new_dest_pr_idx(mt_new_dest_pr_idx),
      // output
      .old_dest_pr(mt_old_dest_pr),

      /* Retire operation */
      .retire_enable  (retire_move_head),
      .retire_arch_reg(retire_arch_reg),
      .retire_phys_reg(retire_phys_reg),

      /* Restore operation */
      // inputs
      .restore_enable(rollback),
      .immune_reg_idx(rollback_immune_reg),  // Set to zero for none
      .immune_preg(rollback_immune_preg),

      /* SET READY operation (CDB) */
      // inputs
      .set_ready_enable(cdb_broadcast_en),
      .ready_phys_idx  (cdb_ready_reg)
  );


  // Declare Free List

  logic fl_dequeue_en;
  logic [`PHYS_REG_IDX_SZ:0] fl_dequeue_pr;

  logic [`PHYS_REG_IDX_SZ:0] fl_enqueue_pr;

  logic fl_enqueue_en, fl_was_enqueued;

  logic [`PHYS_REG_SZ-1:0] fl_rollback_mask;

  free_list free_list_0 (
      .clk  (clock),
      .reset(reset),

      /* READ from head operation */

      // inputs 
      .dequeue_en(fl_dequeue_en),  // If raised, will actually deuqueue from the head

      // outputs
      .is_empty(free_list_empty),  // IF is_empty, front_head_pr is undefined 

      // debug outputs
      .dequeue_pr(fl_dequeue_pr),


      /* WRITE to tail operation */
      // inputs
      .enqueue_en(fl_enqueue_en),  // If raised, will enqueue fl_enqueue_pr at the tail
      .enqueue_pr(fl_enqueue_pr),  // PR to enqueue if fl_enqueue_en raised

      /* ROLLBACK operation */
      // input 
      .rollback(rollback),  // If raised, will clear all bits in rollback_mask
      .rollback_mask(fl_rollback_mask)  // mask from ROB, regs to free on rollback

  );

  //  Declare ROB

  logic [$clog2(`ROB_SZ)-1:0] head;  // Index of the current head printer in the ROB
  logic [$clog2(`ROB_SZ)-1:0] rob_index;  // Index of the current instruction in the ROB

  assign rob_head_idx = head;


  reorder_buffer reorder_buffer_0 (
      .clock(clock),
      .reset(reset),

      // Instruction input to store in ROB (if valid)
      .write(decoded_packet.valid),
      .inst(decoded_packet.inst),
      .inst_PC(decoded_packet.PC),
      .dest_tag(decoded_packet.dest_reg),

      // input from retire stage
      .move_head(retire_move_head),

      .undo(rollback),  // If signal raised, will move next tail back to undo_index (not undo_index+1, trick to prevent rolling back branch)
      .undo_index(head),  // always roll back all the way to head

      .rollback_mask(fl_rollback_mask),

      // Ouput to free physical register at index free_index
      .update_free_list(fl_enqueue_en),
      .free_index(fl_enqueue_pr),

      .phys_told(mt_old_dest_pr.reg_num),

      // output 
      .inst_index(rob_index),
      .full(rob_full),
      .head(head)
  );

  // Declare RS
  PREG rs_ready_reg;
  assign rs_ready_reg.reg_num = cdb_ready_reg;
  assign rs_ready_reg.ready   = cdb_broadcast_en;

  logic
      load_entries_full,
      store_entries_full,
      alu_entries_full,
      mult_entries_full,
      branch_entries_full;
  logic load_store_entries_full;

  assign load_store_entries_full = load_entries_full | store_entries_full;


  logic issue_enable, ls_issue_enable, next_ls_issue_enable;
  assign issue_enable = 1;

  reservation_station reservation_station_0 (
      .clock(clock),
      .reset(reset),

      /* Allocate */
      .allocate(decoded_packet.valid),
      .input_packet(decoded_packet),

      .done(alloc_done),

      /* Update */
      .update(cdb_broadcast_en),
      .ready_reg(rs_ready_reg),

      /* Issue */
      .issue_enable(issue_enable),
      .ls_issue_enable(ls_issue_enable),
      .issued_packet(id_packet),

      /* Head of ROB, so branch knows if it can issue */
      .rob_head_index(head),

      // Output whether or not each type of FU is full 
      .alu_entries_full(alu_entries_full),
      .mult_entries_full(mult_entries_full),
      .load_entries_full(load_entries_full),
      .store_entries_full(store_entries_full),
      .branch_entries_full(branch_entries_full),

      /* Free */
      .free_alu(free_alu),
      .free_mult(free_mult),
      .free_load(free_load),
      .free_store(free_store),
      .free_branch(free_branch),

      /* Rollback */
      .rollback(rollback)

  );

  // stall when issue a load or store
  always_comb begin
    if (ls_issue_enable) begin
      if (id_packet.function_type == LOAD || id_packet.function_type == STORE) begin
`ifdef DEBUG_PRINT
        $display("[RS] stops issuing, waiting for load,store, or branch to finish");
`endif
        next_ls_issue_enable = 0;
      end else begin
        next_ls_issue_enable = 1;
      end
    end else begin
      if ((free_load != 0) || (free_store != 0)) begin
`ifdef DEBUG_PRINT
        $display("load, store, or branch finished, {RS} starts to issue");
`endif
        next_ls_issue_enable = 1;
      end else begin
`ifdef DEBUG_PRINT
        $display("[RS] stops issuing, waiting for load or store to finish");
`endif
        next_ls_issue_enable = 0;
      end
    end
  end

  always_ff @(posedge clock) begin
    if (reset || rollback) begin
      ls_issue_enable <= 1;
    end else begin
      ls_issue_enable <= next_ls_issue_enable;
    end
  end

  /* Just forward some info */
  assign decoded_packet.inst = if_id_packet.inst;
  assign decoded_packet.PC = if_id_packet.PC;
  assign decoded_packet.NPC = if_id_packet.NPC;
  assign decoded_packet.rob_index = rob_index;

  /* Calculate STALL logic */
  assign rs_full = if_id_packet.valid && ((alu_entries_full & (decoded_packet.function_type == ALU))
                   | (mult_entries_full & (decoded_packet.function_type == MULT)) 
                   | (load_store_entries_full & (decoded_packet.function_type == LOAD || decoded_packet.function_type == STORE))
                   | (branch_entries_full & (decoded_packet.function_type == BRANCH)));

  /* Stall if current packet couldn't be accepted, must be resent */
  assign stall = (rob_full | rs_full | free_list_empty);

  /* Access map table (convert arch rs1/rs2 to physical) */

  // Get arch regs from decoded packet
  assign mt_arch_reg1_idx = has_rs1 ? decoded_packet.inst.r.rs1 : `ZERO_REG;
  assign mt_arch_reg2_idx = has_rs2 ? decoded_packet.inst.r.rs2 : `ZERO_REG;

  // Update decoded packet with physical regs from map table
  assign decoded_packet.src1_reg = mt_preg1_out;
  assign decoded_packet.src2_reg = mt_preg2_out;

  /* Mark decoded packet as valid if it is valid and we are ready to pass to RS/ROB (no stall)*/
  assign decoded_packet.valid = if_id_packet.valid & ~decoded_packet.illegal & (!stall);


  /* Dequeue from free list (generating a physical dest) */

  assign fl_dequeue_en = decoded_packet.valid & decoded_packet.has_dest & (decoded_packet.inst.r.rd != `ZERO_REG);
  assign decoded_packet.dest_reg.reg_num = fl_dequeue_en ? fl_dequeue_pr : `ZERO_REG;

  // If a packet doesn't have destination, it must be ready
  assign decoded_packet.dest_reg.ready = ~decoded_packet.has_dest;

  /* Update Map Table with newly dequeued pr dest if it should exist */
  assign mt_set_dest_enable = fl_dequeue_en;
  assign mt_arch_dest_idx = mt_set_dest_enable ? decoded_packet.inst.r.rd : `ZERO_REG;
  assign mt_new_dest_pr_idx = decoded_packet.dest_reg.reg_num;

  assign decoded_packet.arch_dest_reg_num = mt_arch_dest_idx;

endmodule
