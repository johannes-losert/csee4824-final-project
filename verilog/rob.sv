

`include "verilog/sys_defs.svh"

module reorder_buffer(
    input clock,
    input reset,

    /* New instruction from fetch */
    input INST inst,

    /* Enable to write to ROB */
    input write,

    /* Complete stage, indicates instruction has finished (is in complete stage) */
    input finish,
    input logic [$clog2(`ROB_SZ)-1:0] finish_index,

    /* Signal from free list, indicating which register is ready (PREG) */
    // TODO switch to is_empty rather than free_reg = ZERO_REG
    input logic [`PHYS_REG_IDX_SZ:0] free_reg,

    /* If undo, rollback to former index */
    input undo,
    input [$clog2(`ROB_SZ)-1:0] undo_index,

    /* Signal to indicate we should stop fetching */
    output logic stall,

    /* Tell free list we need a reg (TODO connect to dequeue_en) */
    output logic used_free_reg,

    /* Signal to indicate we should update free list (TODO connect enqueue_en in free list) */
    output logic update_free_list,
    /* Index of reg in the free list, indicating it's now free (TODO connect to enqueue_pr) */
    output PREG free_index,

    /* Signal to indicate we should update map table and send packet( reg and corresponding tag) to map table */
    output logic update_map_table,
    output ROB_PACKET rob_mt_packet, /* Unpack ROB packet and pass to GET/SET operation of map table */

    // TODO add input/output from arch table; send an arch register num, and get a preg num back
    // TODO this will be used to set Told

    // TODO add input/output for each FU including 
    // input 'alu_do_dispatch' signal to indicate we will put the instruction in an RS
    // output 'alu_inst' INST/packet of some kind to be passed to RS 

    /* Instruction corresponds to entry in ROB */
    // Pass thru pipeline so during branch mispredict, we can roll back to inst_index
    // Index of currently written instruction 
    output logic [$clog2(`ROB_SZ)-1:0] inst_index,

    /* Signal to indicate we should update arch map and send packet( reg and corresponding old tag) to arch table */
    // Same as the MAP table 
    output logic update_arch_map,
    output ROB_PACKET rob_am_packet,

    /* Signal to indicate the ROB is full */
    // TODO pass this to instruction fetch stall
    output logic full
);

    ROB_ENTRY [`ROB_SZ-1:0] inst_buffer;
    logic [$clog2(`ROB_SZ)-1:0] head;
    logic [$clog2(`ROB_SZ)-1:0] tail;
    logic [$clog2(`ROB_SZ)-1:0] tail_h;
    logic [$clog2(`ROB_SZ)-1:0] next_head;
    logic [$clog2(`ROB_SZ)-1:0] next_tail;
    logic next_full;

    
    assign tail_h = tail + 1;

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
            next_tail = undo_index;
        end else begin
            if (write) begin
                inst_buffer[tail].inst = inst;
                //count_up = 1;
                if (inst.r.rd != `ZERO_REG) begin
                    if (free_reg != `ZERO_REG) begin
                        stall = 0;
                        inst_buffer[tail].T = free_reg; // input from Free list
                        // inst_buffer[tail].Told = arch_map[]  input from arch table
                        inst_buffer[tail].done = 0;
                        used_free_reg = 1;
                        rob_mt_packet.rd  = inst.r.rd;
                        rob_mt_packet.T.reg_num = free_reg;
                        rob_mt_packet.T.ready = 0;
                        update_map_table = 1;
                        if (!full) begin
                            next_tail = tail + 1;
                        end else begin
                            next_tail = tail;
                        end
                    end else begin
                        stall = 1;
                        inst_buffer[tail].T = `ZERO_REG;
                        inst_buffer[tail].Told = `ZERO_REG;
                        inst_buffer[tail].done = 0;
                        rob_mt_packet.rd  = inst.r.rd;
                        rob_mt_packet.T.reg_num = free_reg;
                        rob_mt_packet.T.ready = 0;
                        update_map_table = 0;
                        used_free_reg = 0;
                        next_tail = tail;
                    end
                end else begin
                    stall = 0;
                    inst_buffer[tail].T = `ZERO_REG;
                    inst_buffer[tail].Told = `ZERO_REG;
                    inst_buffer[tail].done = 0;
                    rob_mt_packet.rd  = inst.r.rd;
                    rob_mt_packet.T.reg_num = free_reg;
                    rob_mt_packet.T.ready = 0;
                    update_map_table = 0;
                    used_free_reg = 0;
                    if (!full) begin
                        next_tail = tail + 1;
                    end else begin
                        next_tail = tail;
                    end
                end
            end else begin
                //count_up = 0;
                stall = 0;
                inst_buffer[tail].T = `ZERO_REG;
                inst_buffer[tail].Told = `ZERO_REG;
                inst_buffer[tail].done = 0;
                rob_mt_packet.rd  = inst.r.rd;
                rob_mt_packet.T.reg_num = free_reg;
                rob_mt_packet.T.ready = 0;
                update_map_table = 0;
                used_free_reg = 0;
                next_tail = tail;
            end
        end

        if (finish) begin
            inst_buffer[finish_index].done = 1;
        end 


        //update arch map & free list
        if (inst_buffer[head].done) begin
            //count_down = 1;
            update_free_list = 1;
            free_index = inst_buffer[head].Told;
            update_arch_map = 1;
            rob_am_packet.rd = inst_buffer[head].inst.r.rd;
            rob_am_packet.T = inst_buffer[head].T;
            if (head != tail) begin
                next_head = head + 1;
            end else begin
                next_head = head;
            end
        end else begin
            //count_down = 0;
            next_head = head;
            update_free_list = 0;
            free_index = inst_buffer[head].Told;
            update_arch_map = 0;
            rob_am_packet.rd = inst_buffer[head].inst.r.rd;
            rob_am_packet.T = inst_buffer[head].T;
        end    
    end

    // always_comb begin
        
    // end
    // assign next_counter = counter + count_up -count_down;

    always_ff @(posedge clock) begin
        if (reset) begin
            head <= 0;
            tail <= 0;
            //counter <=0;
        end else if (write) begin
            full <= next_full;
            tail <= next_tail;
            head <= next_head;
            //counter <= next_counter;
            
        end
    end

    always_ff @(negedge clock) begin
        $display("head:%0d tail:%0d", head, tail);
    end


    

endmodule