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
    output logic was_enqueued
);

    logic [`PHYS_REG_SZ-1:0]  	 	free_list_bitmap;
    logic [`PHYS_REG_IDX_SZ:0] 		free_reg_num;
    logic 				need_reset;
    logic [`PHYS_REG_IDX_SZ:0]		next_enqueue_pr;
    logic [`PHYS_REG_IDX_SZ:0]		next_dequeue_pr;
    assign is_empty = (dequeue_pr == 1);

    //enqueue & dequeue
    assign free_list_bitmap[0] = 1; // this is the zero reg (map table is empty)
    always_comb begin
        for (int i = 1; i < `PHYS_REG_SZ; i++) begin
	    if (need_reset | (i == next_enqueue_pr))
	    	free_list_bitmap[i] <= 1;
	    else if (i == next_dequeue_pr)
		free_list_bitmap[i] <= 0;    
	end
    end

    always_comb begin
	for (int i = 0; i < `PHYS_REG_SZ; i++) begin
	    if (free_list_bitmap[i])
		dequeue_pr = i;
	end
    end

    always_ff @(posedge clk) begin
	if (reset) begin
	    next_enqueue_pr <= 0;
	    next_dequeue_pr <= 0;
	    need_reset      <= 1;
	end else begin
	    need_reset	    <= 0;
	    if (enqueue_en)
		next_enqueue_pr <= enqueue_pr;
	    else
		next_enqueue_pr <= 0;
	    if (dequeue_en)
		next_dequeue_pr <= dequeue_pr;
	    else
		next_dequeue_pr <= 0;
	end
    end

endmodule
