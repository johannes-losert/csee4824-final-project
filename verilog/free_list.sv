module free_list (
    input logic clk,
    input logic reset,

    // READ from head operation
    input logic dequeue_en;
    output logic [`PHYS_REG_IDX_SZ:0] dequeue_pr;
    output logic was_dequeued;

    // WRITE to tail operation
    input logic enqueue_en;
    input logic [`PHYS_REG_IDX_SZ:0] enqueue_pr;
    output logic was_enqueued;
);

    /*
    Free list is array of PR numbers. The length of free list is number of registers
    plus 1. 
    */
    logic [`PHYS_REG_IDX_SZ+1:0] [`PHYS_REG_IDX_SZ:0] free_list;

    // Head pointer points to first free slot
    logic [`PHYS_REG_IDX_SZ:0] head_ptr;
    // Tail pointer points to last filled slot
    logic [`PHYS_REG_IDX_SZ:0] tail_ptr;

    logic is_full, is_empty, do_enqueue, do_dequeue;

    assign is_empty = (head_ptr == tail_ptr);
    assign is_full = ( (head_ptr + 1) % (PHYS_REG_SZ + 1) == tail_ptr);


/* Ex. 3 physical registers, requires 4 slots (one always empty)
    [ [x], <- head_ptr, tail_ptr
      [x],
      [x],
      [x] ]
    enqueue 1
    [ [1], <- tail_ptr
      [x], <-head_ptr
      [x],
      [x] ] 
    enqueue 2
    [ [1], <- tail
      [2], 
      [x], <-head
      [x] ]
    enqueue 3
    [ [1], <- tail
      [2], 
      [3], 
      [x] <-head ]
    dequeue 1
    [ [x], 
      [2], <-tail
      [3],
      [x], <- head]
    enqueue 4
    [ [x], <- head
      [2], <- tail
      [3],
      [4] ]
    enqueue 5, dequeue 2
    [ [5], 
      [x], <- head
      [3], <- tail
      [4] ] 
*/

    always @(posedge clk) begin
        if (reset) begin
            head_ptr <= 0;
            tail_ptr <= 0;
            size <= 0;
        end else begin
            // Directly forward, do not update pointers
            if (dequeue_en && enqueue_en && is_empty) begin 
                dequeue_pr <= enqueue_pr;
                was_dequeued <= 1;
                was_enqueued <= 1;
            end else begin
                if (dequeue_en) begin
                    if (is_empty) begin
                        was_dequeued <= 0;
                    end else begin
                        dequeue_pr <= free_list[tail_ptr];
                        tail_ptr <= (tail_ptr + 1) % (PHYS_REG_SZ + 1);
                        was_dequeued <= 1;
                    end        
                end else begin
                    was_dequeued <= 0;
                end

                if (enqueue_en) begin
                    if (is_full && !dequeue_en) begin
                        was_enqueued <= 0;
                    end else begin
                        free_list[head_ptr] <= enqueue_pr;
                        head_ptr <= (head_ptr + 1) % (PHYS_REG_SZ + 1);
                        was_enqueued <= 1;
                    end
                end else begin
                    was_enqueued <= 0;
                end
            end

        /*

if (dequeue_en && enqueue_en && is_empty)
        // forward enqueued->dequed
else if (dequeue_en && enqueue_en)
        // dequeue first then enqueue
else if (dequeue)
    // just dequeue
else if (enqueue)
    // just enquque

        */
        end
    end



    // One PREG per arch reg, always ready
    PREG [`REG_IDX_SZ:1] preg_entries;
    PREG temp_old_dest_pr;

    // Read port 1
    always_comb begin 
        if (arch_reg1_idx == `ZERO_REG) begin
            preg1_out.ready = 1;
            preg1_out.reg_num = `ZERO_REG;
        end else begin 
             // Forwarding; if CDB broadcasting to the PR we're reading from, use its value for the ready bit
             // Note: still need to update the ready bit in the PR
            if (set_ready_enable && (ready_phys_idx == preg_entries[arch_reg1_idx].reg_num)) begin
                preg1_out.ready = 1;
            end else begin
                preg1_out.ready = preg_entries[arch_reg1_idx].ready;
            end 

            preg1_out.reg_num = preg_entries[arch_reg1_idx].reg_num;
        end
    end 

    // Read port 2
    always_comb begin 
        if (arch_reg2_idx == `ZERO_REG) begin
            preg2_out.ready = 1;
            preg2_out.reg_num = `ZERO_REG;
        end else begin 
            if (set_ready_enable && (ready_phys_idx == preg_entries[arch_reg2_idx].reg_num)) begin
                preg2_out.ready = 1;
            end else begin
                preg2_out.ready = preg_entries[arch_reg2_idx].ready;
            end 

            preg2_out.reg_num = preg_entries[arch_reg2_idx].reg_num;
        end
    end

    // GET old dest (TODO test that new dest doesn't corrupt this)
    always_comb begin 
        if (arch_dest_idx == `ZERO_REG) begin
            old_dest_pr.ready = 1;
            old_dest_pr.reg_num = `ZERO_REG;
        end else begin 
            if (set_ready_enable && (ready_phys_idx == temp_old_dest_pr.reg_num)) begin
                old_dest_pr.ready = 1;
            end else begin
                old_dest_pr.ready = temp_old_dest_pr.ready;
            end 

            old_dest_pr.reg_num = temp_old_dest_pr.reg_num;
        end
    end 

endmodule