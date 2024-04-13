
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;
    // inputs
    logic clock, reset;

    IF_ID_PACKET if_id_packet;

    logic cdb_broadcast_en;
    logic [`PHYS_REG_IDX_SZ:0] cdb_ready_reg;

    logic rollback;

    logic retire_move_head;

    logic [`NUM_FU_ALU-1:0] free_alu;            // (new) free ALU entry
    logic [`NUM_FU_MULT-1:0] free_mult;             // (new) free MULT entry
    logic [`NUM_FU_LOAD-1:0] free_load;             // (new) free LOAD entry
    logic [`NUM_FU_STORE-1:0] free_store;           // (new) free STORE entry
    logic [`NUM_FU_BRANCH-1:0] free_branch;           // (new) free BRANCH entry

    // outputs
    logic stall;
    ID_IS_PACKET id_packet;

    dispatch dut(
        .clock(clock),
        .reset(reset),

        .if_id_packet(if_id_packet),

        .cdb_broadcast_en(cdb_broadcast_en),
        .cdb_ready_reg(cdb_ready_reg),
        
        .rollback(rollback),
        
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load),
        .free_store(free_store),
        .free_branch(free_branch),

        .retire_move_head(retire_move_head),
        // outputs
        .stall(stall),
        .id_packet(id_packet)
        
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

        $monitor("-------clock=%0d reset=%0d----\ninput_instr=%0h input_pc = %0d \
input_npc = %0d input_valid = %0b cdb_broadcast_en=%0b cdb_ready_reg=%0d \
rollback=%0b retire_move_head=%0b \ \nstall=%0b output_instr=%0h output_pc=%0d output_npc=%0d \
output_dest_reg_idx=%0d output_src1_reg_idx=%0d output_src2_reg_idx=%0d \
output_func_type=%0d output_valid=%0b\n-------\n", clock, reset, if_id_packet.inst, 
                 if_id_packet.PC, if_id_packet.NPC, if_id_packet.valid, cdb_broadcast_en,
                 cdb_ready_reg, rollback, retire_move_head, stall, id_packet.inst,
                 id_packet.PC, id_packet.NPC, id_packet.dest_reg.reg_num,
                 id_packet.src1_reg.reg_num, id_packet.src2_reg.reg_num, id_packet.function_type,
                 id_packet.valid);

        clock     = 0;
        reset     = 0;

        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;


        if_id_packet.inst = `NOP;
        if_id_packet.PC = 0;
        if_id_packet.NPC = 0;
        if_id_packet.valid = 0;
        
        cdb_broadcast_en = 0;
        cdb_ready_reg = 0;
        rollback = 0;
        retire_move_head = 0;

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        $display("Test case 1");

        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;

        if_id_packet.inst.r.funct7 = 7'b101;
        if_id_packet.inst.r.rd = 5'h1;
        if_id_packet.inst.r.rs1 = 5'h2;
        if_id_packet.PC = 1;
        if_id_packet.NPC = 2;
        if_id_packet.valid = 0;
        
        cdb_broadcast_en = 0;
        cdb_ready_reg = 0;
        rollback = 0;
        retire_move_head = 0;
      
        @(negedge clock)
        assert(!id_packet.valid) else exit_on_error;
      
        $display("Fetch Mul instruction");       

        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;

        if_id_packet.inst = `RV32_MUL;
        if_id_packet.inst.r.rd = 5'h1;
        if_id_packet.inst.r.rs1 = 5'h2;
        if_id_packet.inst.r.rs2 = 5'h3;
        if_id_packet.PC = 2;
        if_id_packet.NPC = 3;
        if_id_packet.valid = 1;
        
        cdb_broadcast_en = 0;
        cdb_ready_reg = 0;
        rollback = 0;
        retire_move_head = 0;

        @(negedge clock)
        assert(!id_packet.valid) else exit_on_error;

              
        $display("Fetch Add Instruction");       

       
        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;

        if_id_packet.inst = `RV32_ADD;
        if_id_packet.inst.r.rd = 5'h1;
        if_id_packet.inst.r.rs1 = 5'h2;
        if_id_packet.inst.r.rs2 = 5'h3;
        if_id_packet.PC = 3;
        if_id_packet.NPC = 4;
        if_id_packet.valid = 1;
        
        cdb_broadcast_en = 0;
        cdb_ready_reg = 0;
        rollback = 0;
        retire_move_head = 0;

        @(negedge clock)
        assert(id_packet.valid) else exit_on_error;
        assert(id_packet.function_type == MULT) else exit_on_error;
        $display("Test case 4");       

        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;

        if_id_packet.inst.r.funct7 = 7'b101;
        if_id_packet.inst.r.rd = 5'h1;
        if_id_packet.inst.r.rs1 = 5'h2;
        if_id_packet.inst.r.rs2 = 5'h3;
        if_id_packet.PC = 5;
        if_id_packet.NPC = 6;
        if_id_packet.valid = 0;
        
        cdb_broadcast_en = 1;
        cdb_ready_reg = 2;
        rollback = 0;
        retire_move_head = 0;

        @(negedge clock)
        assert(id_packet.valid) else exit_on_error;
        assert(id_packet.function_type == ALU) else exit_on_error;
        $display("Test case 5");       

        free_alu = 0;
        free_mult = 0;
        free_load = 0;
        free_store = 0;
        free_branch = 0;

        if_id_packet.inst.r.funct7 = 7'b101;
        if_id_packet.inst.r.rd = 5'h1;
        if_id_packet.inst.r.rs1 = 5'h2;
        if_id_packet.inst.r.rs2 = 5'h3;
        if_id_packet.PC = 7;
        if_id_packet.NPC = 8;
        if_id_packet.valid = 0;
        
        cdb_broadcast_en = 1;
        cdb_ready_reg = 3;
        rollback = 0;
        retire_move_head = 0;

        @(negedge clock)
       // assert(id_packet.valid) else exit_on_error;

        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        @(negedge clock)
        $display("@@@Passed");
        $finish;

        
    end 
endmodule