// `include "verilog/sys_defs.svh"

module reorder_buffer(
    input clock,
    input reset,

    /* New instruction from fetch/dispatch */
    input INST inst,
    /* Signal from free list, indicating which register is ready (PREG) */
    // TODO switch to is_empty rather than free_reg = ZERO_REG
    input PREG tag,

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


    /* Signal to indicate we should update free list (TODO connect enqueue_en in free list) */
    output logic update_free_list,
    /* Index of reg in the free list, indicating it's now free (TODO connect to enqueue_pr) */
    output logic [`PHYS_REG_IDX_SZ:0] free_index,

    /* Signal to indicate we should update arch map and send packet( reg and corresponding old tag) to arch table */
    // Same as the MAP table 
    output logic update_arch_map,
    output logic [`REG_IDX_SZ:0] update_arch_told,
    /* signal to update arch_map*/
    output logic [`REG_IDX_SZ:0] arch_told,
    input logic [`PHYS_REG_IDX_SZ:0] phys_told,

    /* Instruction corresponds to entry in ROB */
    // Pass thru pipeline so during branch mispredict, we can roll back to inst_index
    // Index of currently written instruction 
    output logic [$clog2(`ROB_SZ)-1:0] inst_index,


    /* Signal to indicate the ROB is full */
    // TODO pass this to instruction fetch stall
    output logic full,

    /* passing the head's index to the retire stage */
    output logic [$clog2(`ROB_SZ)-1:0] head
);

    ROB_ENTRY [`ROB_SZ-1:0] inst_buffer;
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

    
    assign inst_index = tail;

    always_comb begin
        //update ROB & map table
        if (undo) begin
            next_tail = undo_index + 1;
        end else begin
            if (write) begin
                inst_buffer[tail].inst = inst;
                if (inst.r.rd != `ZERO_REG) begin
                    inst_buffer[tail].T = tag; // input from Free list
                    inst_buffer[tail].Told = phys_told;
                end else begin
                    inst_buffer[tail].T = `ZERO_REG;
                    inst_buffer[tail].Told = `ZERO_REG;
                end
                if (!full) begin
                    next_tail = tail + 1;
                end else begin
                    next_tail = tail;
                end
            end else begin
                inst_buffer[tail].T = `ZERO_REG;
                inst_buffer[tail].Told = `ZERO_REG;
                next_tail = tail;
            end
        end

        // if (finish) begin
        //     inst_buffer[finish_index].done = 1;
        // end 

        //update arch map & free list

        // if (inst_buffer[head].done) begin
        if (move_head) begin
            update_free_list = 1;
            free_index = inst_buffer[head].Told.reg_num;
            update_arch_map = 1;
            update_arch_told = inst_buffer[head].inst.r.rd;
            // update_phys_told = inst_buffer[head].T;
            if (head != tail) begin
                next_head = head + 1;
            end else begin
                next_head = head;
            end
        end else begin
            next_head = head;
            update_free_list = 0;
            free_index = inst_buffer[head].Told.reg_num;
            update_arch_map = 0;
            update_arch_told = inst_buffer[head].inst.r.rd;
            // update_phys_told = inst_buffer[head].T;
        end    
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            head <= 0;
            tail <= 0;
            full <= 0;
        end else if (write) begin
            full <= next_full;
            tail <= next_tail;
            head <= next_head;
        end
    end

    always_ff @(negedge clock) begin
        $display("head:%0d tail:%0d", head, tail);
    end

endmodule