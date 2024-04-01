
`include "verilog/sys_defs.svh"


module testbench;

    logic clock, reset;
    INST inst;
    logic write;
    logic finish;
    logic [$clog2(`ROB_SZ)-1:0] finish_index;
    logic [4:0] free_reg;
    logic undo;
    logic [$clog2(`ROB_SZ)-1:0] undo_index;
    logic stall;
    logic used_free_reg;
    logic update_free_list;
    logic update_map_table;
    ROB_PACKET rob_mt_packet;
    logic [$clog2(`ROB_SZ)-1:0] inst_index;
    logic update_arch_map;
    ROB_PACKET rob_am_packet;
    PREG free_index;
    logic full;


    reorder_buffer dut(
        .clock(clock),
        .reset(reset),
        .inst(inst),
        .write(write),
        .finish(finish),
        .finish_index(finish_index),
        .free_reg(free_reg),
        .undo(undo),
        .undo_index(undo_index),
        .stall(stall),
        .used_free_reg(used_free_reg),
        .update_free_list(update_free_list),
        .update_map_table(update_map_table),
        .rob_mt_packet(rob_mt_packet),
        .inst_index(inst_index),
        .update_arch_map(update_arch_map),
        .rob_am_packet(rob_am_packet),
        .free_index(free_index),
        .full(full)
 
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin
        $monitor("clock=%0d reset=%0d inst=%0d write=%0d finish=%0d finish_index=%0d free_reg=%0d undo=%0d undo_index=%0d stall=%0d used_free_reg=%0d update_free_list=%0d \
        update_map_table=%0d rob_mt_packet=%0d inst_index=%0d update_arch_map=%0d rob_am_packet=%0d free_index=%0d full=%0d", 
        clock, reset, inst, write, finish, finish_index, free_reg, undo, undo_index, stall, used_free_reg, update_free_list, 
        update_map_table, rob_mt_packet, inst_index, update_arch_map, rob_am_packet, free_index, full);


        clock     = 0;
        reset     = 0;
        inst      = 0;
        write     = 0;
        finish = 0;
        finish_index = 0;
        free_reg = 0;
        undo = 0;
        undo_index = 0;

        

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        $display("Test case 1");
        inst = 54;
        inst.r.rd = 1;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 5;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 0) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 1) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 5) else exit_on_error;
        assert(used_free_reg == 1);

        // Test case 2
        $display("Test case 2");
        inst = 64;
        inst.r.rd = 2;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 6;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 1) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 2) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 6) else exit_on_error;
        assert(used_free_reg == 1);


        // Test case 3
        $display("Test case 3");
        inst = 74;
        inst.r.rd = `ZERO_REG;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 7;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 2) else exit_on_error;
        assert(update_map_table == 0) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == `ZERO_REG) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 7) else exit_on_error;
        assert(used_free_reg == 0);
        

        // Test case 4
        $display("Test case 4");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 3) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);


        // below are all same test cases just to fill up the reorder buffer
        // Test case 5
        $display("Test case 5");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 4) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);
       
        // Test case 6
        $display("Test case 6\n");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 5) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);
        
        
        // Test case 7
        $display("Test case 7");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 6) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);
        
        
        // Test case 8
        $display("Test case 8");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(inst_index == 7) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 0) else exit_on_error;
        assert(update_free_list == 0) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);
        


        $display("Test case 9");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 0;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(full) else exit_on_error;
        
        
        // Test case 10
        $display("Test case 10");
        inst = 84;
        inst.r.rd = 3;
        write = 1;
        finish = 1;
        finish_index = 0;
        free_reg = 8;
        undo = 0;
        undo_index = 0;

        @(negedge clock)
        assert(!full) else exit_on_error;
        assert(free_index.reg_num == 5) else exit_on_error;
        assert(inst_index == 0) else exit_on_error;
        assert(update_map_table == 1) else exit_on_error;
        assert(update_arch_map == 1) else exit_on_error;
        assert(update_free_list == 1) else exit_on_error;
        assert(rob_mt_packet.rd == 3) else exit_on_error;
        assert(rob_mt_packet.T.reg_num == 8) else exit_on_error;
        assert(used_free_reg == 1);

        
        $display("@@@Passed");
        $finish;

        
    end 
endmodule