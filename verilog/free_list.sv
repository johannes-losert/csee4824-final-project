// TODO switch head and tail names
module free_list (
    input logic clk,
    input logic reset,

    output logic is_empty, /* IF is_empty, front_head_pr is undefined */

    input logic dequeue_en,
    output logic [`PHYS_REG_IDX_SZ:0] dequeue_pr, /* Probably don't need this either */

    // WRITE to tail operation
    input logic enqueue_en,
    input logic [`PHYS_REG_IDX_SZ:0] enqueue_pr,

    // Rollback mask includes things from ROB to free (TODO could pack this to be more efficient if actually synthesizing)
    input logic rollback,
    input logic [`PHYS_REG_SZ-1:0] rollback_mask,

    // DEBUG 
    output logic [`PHYS_REG_SZ-1:0] free_list_bitmap

);

   // logic [`PHYS_REG_SZ-1:0] free_list_bitmap;

    /* True if current dequeue_pr comes via enqueue_pr, not the bitmap */
    logic passthrough;

    assign passthrough = is_empty && enqueue_en;

    /* Find the first free PR, assign as dequeue_pr */
    always_comb begin
        /* if empty, but about to enqueue something, just pass through to dequeue_pr */ 
        if (passthrough)
            dequeue_pr = enqueue_pr;
        else begin 
            for (int i = `PHYS_REG_SZ-1; i > 0; i--) begin
                if (free_list_bitmap[i]) begin
                    dequeue_pr = i;
                end
            end
        end 
    end


    /* Calculating is_empty, this feels inefficient, but probably gets compiled/synthesized to efficient bit operation */
    always_comb begin 
        is_empty = 1;
        for (int i = 0; i < `PHYS_REG_SZ; i++) begin
            if (free_list_bitmap[i]) begin
                is_empty = 0;
            end
        end
    end 


    always_ff @(posedge clk) begin
        if (reset) begin
            /* Everything starts free */
            for (int i = 0; i < `PHYS_REG_SZ; i++) begin
                free_list_bitmap[i] <= 1;
            end
        end else begin
            /* If rolling back, mark all bits in rollback_mask as free */
            if (rollback) begin
                // TODO could do this much cleaner if this was actually a bitmap (but it probably compiles/synthesizes to something not too bad)
                `ifdef DEBUG_PRINT
                $display("[FL] Rolling back, mask: %b", rollback_mask);
                `endif
                for (int i = 0; i < `PHYS_REG_SZ; i++) begin
                    if (rollback_mask[i])
                        free_list_bitmap[i] <= 1;
                end

                // TODO is there ever a case where we rollback AND dequeue/enqueue?

            end else begin 
                    /* If dequeue_en, then mark dequeued pr as not free (on pass-through, this may already be 0) */
                if (dequeue_en) begin
                    free_list_bitmap[dequeue_pr] <= 0; 
                end 

                /* If enqueue_en and not pass-through, then mark enqueued pr as free */
                if (enqueue_en && !passthrough) begin
                    free_list_bitmap[enqueue_pr] <= 1;
                end            
            end
        end
    end

    always_ff @(negedge clk) begin
        `ifdef DEBUG_PRINT
        $display("FREE LIST:");
        $display("Bitmap: %b, rollback: %b, rollback mask: %b", free_list_bitmap, rollback, rollback_mask);
        //$dipslay("is_empty: %b");
        `endif
    end 


endmodule
