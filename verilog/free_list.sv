// TODO switch head and tail names
module free_list (
    input logic clk,
    input logic reset,

    // READ from head operation
    output logic [`PHYS_REG_IDX_SZ:0] front_head_pr, /* Always points to the next element to be dequeued, unless is_empty */
    output logic is_empty, /* IF is_empty, front_head_pr is undefined */

    input logic dequeue_en,
    output logic was_dequeued, /* Probably don't need this */
    output logic [`PHYS_REG_IDX_SZ:0] dequeue_pr, /* Probably don't need this either */

    // WRITE to tail operation
    input logic enqueue_en,
    input logic [`PHYS_REG_IDX_SZ:0] enqueue_pr,
    output logic was_enqueued,
    output logic is_full,

    output logic [`PHYS_REG_IDX_SZ+1:0] back_tail_ptr,
    output logic [`PHYS_REG_IDX_SZ+1:0] front_head_ptr,

    output logic [`PHYS_REG_IDX_SZ:0] free_list[`FREE_LIST_SIZE]
);

    /*
    Free list is array of PR numbers. The length of free list is number of registers
    plus 1. 
    */
    // logic [`PHYS_REG_IDX_SZ+1:0] [`PHYS_REG_IDX_SZ:0] free_list;
// WRONG, swithc head and tail names
    // // Tail pointer points to first free slot
    // logic [`PHYS_REG_SZ+1:0] head_ptr;
    // // Tail pointer points to last filled slot
    // logic [`PHYS_REG_SZ+1:0] tail_ptr;

    // logic is_full, is_empty, do_enqueue, do_dequeue;

    assign is_empty = (back_tail_ptr == front_head_ptr);
    assign is_full = ( (back_tail_ptr + 1) % (`FREE_LIST_SIZE) == front_head_ptr);
    assign front_head_pr = free_list[front_head_ptr];


/* Ex. 3 physical registers, requires 4 slots (one always empty)
    [ [x], <- head_ptr, tail_ptr
      [x],
      [x],
      [x] ]
    enqueue 1
    [ [1], <- head_ptr
      [x], <-tail_ptr
      [x],
      [x] ] 
    enqueue 2
    [ [1], <- head
      [2], 
      [x], <-tail
      [x] ]
    enqueue 3
    [ [1], <- head
      [2], 
      [3], 
      [x] <- tail ]
    dequeue 1
    [ [x], 
      [2], <-head
      [3],
      [x], <- tail]
    enqueue 4
    [ [x], <- tail
      [2], <- head
      [3],
      [4] ]
    enqueue 5, dequeue 2
    [ [5], 
      [x], <- tail
      [3], <- head
      [4] ] 
*/

    always @(posedge clk) begin
        if (reset) begin
            back_tail_ptr <= 0;
            front_head_ptr <= 1;
            was_dequeued <= 0;
            was_enqueued <= 0;
            dequeue_pr <= 0;
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
                        dequeue_pr <= 0;
                    end else begin
                        dequeue_pr <= free_list[front_head_ptr];
                        front_head_ptr <= (front_head_ptr + 1) % (`FREE_LIST_SIZE);
                        was_dequeued <= 1;
                    end        
                end else begin
                    was_dequeued <= 0;
                    dequeue_pr <= 0;
                end

                if (enqueue_en) begin
                    if (is_full && !dequeue_en) begin
                        was_enqueued <= 0;
                    end else begin
                        free_list[back_tail_ptr] <= enqueue_pr;
                        back_tail_ptr <= (back_tail_ptr + 1) % (`FREE_LIST_SIZE);
                        was_enqueued <= 1;
                    end
                end else begin
                    was_enqueued <= 0;
                end
            end
        end
    end
endmodule