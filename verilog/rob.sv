

`include "verilog/sys_defs.svh"

module reorder_buffer(
    input clock,
    input reset,
    input INST inst,
    input write,
    input finish,
    input logic [$clog2(`ROB_SZ)-1:0] finish_index,
    input logic [4:0] free_reg,
    input undo,
    input [$clog2(`ROB_SZ)-1:0] undo_index,
    output logic stall,
    output logic used_free_reg,
    output logic update_free_list,
    output logic update_map_table,
    output ROB_PACKET rob_mt_packet,
    output logic [$clog2(`ROB_SZ)-1:0] inst_index,
    output logic update_arch_map,
    output ROB_PACKET rob_am_packet,
    output PREG free_index,
    output logic full
);

    ROB_ENTRY [`ROB_SZ-1:0] inst_buffer;
    logic [$clog2(`ROB_SZ)-1:0] head;
    logic [$clog2(`ROB_SZ)-1:0] tail;
    logic [$clog2(`ROB_SZ)-1:0] tail_h;
    logic [$clog2(`ROB_SZ)-1:0] next_head;
    logic [$clog2(`ROB_SZ)-1:0] next_tail;
    logic [$clog2(`ROB_SZ)-1:0] next_inst_index;
    logic next_full;
    logic [$clog2(`ROB_SZ):0] counter;
    logic [$clog2(`ROB_SZ):0] next_counter;
    logic count_down, count_up;
    

    // always_comb begin
    //     // the addition will truncate to last 3 bits if tail is 7(111), which will generate a 000
    //     if (tail_h == head) begin
    //         next_full = 1;
    //     end else begin
    //         next_full = 0;
    //     end
    // end

    always_comb begin
        if (counter == `ROB_SZ + 1) begin
            full = 1;
        end else begin
            full = 0;
        end
    end

    

    assign next_inst_index = tail;

    always_comb begin
        //update ROB & map table
        if (undo) begin
            next_tail = undo_index;
        end else if (!full) begin
            if (write) begin
                inst_buffer[tail].inst = inst;
                count_up = 1;
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
                        next_tail = tail + 1;
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
                    next_tail = tail + 1;
                end
            end else begin
                count_up = 0;
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
        end else begin
            count_up = 0;
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

        if (finish) begin
            inst_buffer[finish_index].done = 1;
        end 


        //update arch map & free list
        if (inst_buffer[head].done) begin
           count_down = 1;
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
            count_down = 0;
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
    assign next_counter = counter + count_up -count_down;

    always_ff @(posedge clock) begin
        if (reset) begin
            head <= 0;
            tail <= 0;
            inst_index <= 0;
            counter <=0;
        end else if (write) begin
            // full <= next_full;
            tail <= next_tail;
            head <= next_head;
            inst_index <= next_inst_index;
            counter <= next_counter;
        end
    end


    

endmodule