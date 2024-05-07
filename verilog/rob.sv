module reorder_buffer (
    input clock,
    input reset,

    /* New instruction from fetch/dispatch */
    input INST inst,
    input logic [`XLEN-1:0] inst_PC,

    /* Signal from free list, indicating which register is ready (PREG) */
    input PREG dest_tag,

    /* Enable to write to ROB 
       connect to the valid signal from dispatch stage*/
    input logic write,

    /* Retire stage tells the rob to move its head */
    input logic move_head,

    /* If undo, rollback to former index 
       undo may connect to take_branch
       undo_index may connect to head_index*/
    input undo,
    input logic [$clog2(`ROB_SZ)-1:0] undo_index,

    /* Signal to indicate we should update free list  */
    output logic update_free_list,
    /* Index of reg in the free list, indicating it's now free  */
    output logic [`PHYS_REG_IDX_SZ:0] free_index,

    /* Signal to indicate we should update arch map and send packet( reg and corresponding old tag) to arch table */
    // Same as the MAP table 
    output logic reg_retire_en,
    output logic [`REG_IDX_SZ:0] retire_reg_arch_idx,
    output PREG retire_reg_preg,
    /* signal to update arch_map*/
    output logic [`REG_IDX_SZ:0] arch_told,
    input logic [`PHYS_REG_IDX_SZ:0] phys_told,

    /* Instruction corresponds to entry in ROB */
    // Pass thru pipeline so during branch mispredict, we can roll back to inst_index
    // Index of currently written instruction 
    output logic [$clog2(`ROB_SZ)-1:0] inst_index,

    /* Signal to free list on rollback, indicating which registers are now free */
    output logic [`PHYS_REG_SZ-1:0] rollback_mask,


    /* Signal to indicate the ROB is full */
    output logic full,

    /* passing the head's index to the retire stage */
    output logic [$clog2(`ROB_SZ)-1:0] head,


    output ROB_ENTRY [`ROB_SZ-1:0] inst_buffer
);

  // ROB_ENTRY [`ROB_SZ-1:0] inst_buffer;
  logic [$clog2(`ROB_SZ)-1:0] tail;
  logic [$clog2(`ROB_SZ)-1:0] tail_h;
  logic [$clog2(`ROB_SZ)-1:0] next_head;
  logic [$clog2(`ROB_SZ)-1:0] next_tail;
  logic next_full;

  assign tail_h = tail + 1;
  assign arch_told = inst.r.rd;

  always_comb begin
    // the addition will truncate to last 3 bits if tail is 7(111), which will generate a 000
    if (tail_h == head) begin
      next_full = 1;
    end else begin
      next_full = 0;
    end
  end

  /* calculate rollback mask */
  always_comb begin
    rollback_mask = 0;
    if (undo) begin
      /* For each instruction between head and tai; (including tail, not including head), add T to rollback mask */
      for (int i = 0; i < `ROB_SZ; i++) begin
        /* If tail >= head, then 'active' entries are those s.t. head < i <= tail */
        if (tail >= head) begin
          if (i > head && i <= tail) begin
            rollback_mask[inst_buffer[i].T.reg_num] = 1;
          end
        end 
                /* If tail < head, then 'active' entries are those s.t. i > head || i <= tail */
        else begin
          assert (tail != head);
          if (i > head || i <= tail) begin
            rollback_mask[inst_buffer[i].T.reg_num] = 1;
          end
        end
      end
    end
  end


  assign inst_index = tail;

  always_comb begin
    //update ROB & map table
    if (undo) begin
      next_tail = undo_index;
    end else begin
      if (write) begin
        inst_buffer[tail].inst = inst;
        inst_buffer[tail].PC   = inst_PC;
        if (inst.r.rd != `ZERO_REG) begin
          inst_buffer[tail].T = dest_tag;  // input from Free list
          inst_buffer[tail].Told.reg_num = phys_told;
        end else begin
          inst_buffer[tail].T.reg_num = `ZERO_REG;
          inst_buffer[tail].Told.reg_num = `ZERO_REG;
          inst_buffer[tail].T.ready = 1;
          inst_buffer[tail].Told.ready = 1;
        end
        if (!full) begin
          next_tail = tail + 1;
        end else begin
          next_tail = tail;
        end
      end else begin
        inst_buffer[tail].T.reg_num = `ZERO_REG;
        inst_buffer[tail].Told.reg_num = `ZERO_REG;
        inst_buffer[tail].T.ready = 1;
        inst_buffer[tail].Told.ready = 1;
        next_tail = tail;
      end
    end

    //update arch map & free list

    if (move_head) begin
      update_free_list = 1;
      free_index = inst_buffer[head].Told.reg_num;
      reg_retire_en = 1;
      retire_reg_arch_idx = inst_buffer[head].inst.r.rd;
      retire_reg_preg = inst_buffer[head].T;
      retire_reg_preg.ready = 1;
      if (head != tail) begin
        next_head = head + 1;
      end else begin
        next_head = head;
      end
    end else begin
      next_head = head;
      update_free_list = 0;
      free_index = inst_buffer[head].Told.reg_num;
      reg_retire_en = 0;
      retire_reg_arch_idx = inst_buffer[head].inst.r.rd;
      retire_reg_preg = inst_buffer[head].T;
      retire_reg_preg.ready = 0;
    end
  end

  always_ff @(posedge clock) begin
`ifdef DEBUG_PRINT
    if (move_head) $display("[ROB] move head");
    if (undo) $display("[ROB] undo, rollback to %0d", undo_index);
`endif

    // print_reorder_buffer();
    if (reset) begin
      head <= 0;
      tail <= 0;
      full <= 0;
    end else begin
      full <= next_full;
      tail <= next_tail;
      head <= next_head;
    end
  end


`ifdef DEBUG_PRINT
  function void print_reorder_buffer();
    $display("REORDER BUFFER");
    $display("head:%0d tail:%0d", head, tail);
    $display("full? %0d", full);
    $display("ht \t| num \t| inst \t\t| T \t| Told");
    for (int i = 0; i < `ROB_SZ; i++) begin
      // If is the head, mark an h in the ht column 
      // If is the tail, mark a t in the ht column
      // If is the head and tail, mark a ht in the ht column
      string ht = "";
      if (i == head && i == tail) $write("ht \t|");
      else if (i == head) $write("h \t|");
      else if (i == tail) $write("t \t|");
      else $write(" \t|");

      // write the num i to the num column
      $write(" %0d \t|", i);

      // use the c function print_inst to print the instruction in the inst column
      print_inst(inst_buffer[i].inst, inst_buffer[i].PC, 1);

      // display t and t_old in the T and Told columns
      $write("\t| ");
      print_preg(inst_buffer[i].T);
      $write(" \t| ");
      print_preg(inst_buffer[i].Told);
      $display("");
    end
  endfunction


  always_ff @(negedge clock) begin
    print_reorder_buffer();
  end

`endif

endmodule
