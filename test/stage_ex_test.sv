
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;

    logic [`XLEN-1:0] mult_result, branch_result;
    logic clock, reset;
    logic [`NUM_FU_ALU-1:0] free_alu;
    logic [`NUM_FU_MULT-1:0] free_mult;
    logic [`NUM_FU_LOAD-1:0] free_load;
    logic [`NUM_FU_STORE-1:0] free_store;
    logic [`NUM_FU_BRANCH-1:0] free_branch;
    logic [`MAX_FU_INDEX-1:0]     issue_fu_index;
    integer i;
    IS_EX_PACKET is_ex_reg, alu_packet, mult_packet, branch_packet;
    EX_CO_PACKET ex_packet;


    stage_ex dut(
        .clock(clock),
        .reset(reset),
        .is_ex_reg(is_ex_reg),
        .issue_fu_index(issue_fu_index),
        .ex_packet(ex_packet),
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load), 
        .free_store(free_store),
        .free_branch(free_branch),
        .tmp_alu_packet(alu_packet),
        .tmp_mult_packet(mult_packet),
        .tmp_branch_packet(branch_packet)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done_mult;
        forever begin : wait_loop
            @(posedge free_mult[0]);
            @(negedge clock);
            if (free_mult[0]) begin
                disable wait_until_done_mult;
            end
        end
    endtask

    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done_alu;
        forever begin : wait_loop
            @(posedge free_alu[0]);
            @(negedge clock);
            if (free_alu[0]) begin
                disable wait_until_done_alu;
            end
        end
    endtask

    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done_branch;
        forever begin : wait_loop
            @(posedge free_branch[0]);
            @(negedge clock);
            if (free_branch[0]) begin
                disable wait_until_done_branch;
            end
        end
    endtask

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin
        // NOTE: monitor starts using 5-digit decimal values for printing
        $monitor("Time:%4.0f opa:%d opb:%d alu_func:%d func:%b result:%d funit:%2h free_alu:%b free_mult:%b free_branch:%b take_branch:%b",
                 $time, $signed(is_ex_reg.rs1_value), $signed(is_ex_reg.rs2_value), is_ex_reg.alu_func, is_ex_reg.inst.b.funct3, $signed(ex_packet.result), is_ex_reg.function_type, free_alu, free_mult, free_branch, ex_packet.take_branch);
        $display("\nBeginning edge-case testing:");

        is_ex_reg <= '{
            `NOP, // we can't simply assign 0 because NOP is non-zero
            {`XLEN{1'b0}}, // PC
            {`XLEN{1'b0}}, // NPC
            {`XLEN{1'b0}}, // rs1 select
            {`XLEN{1'b0}}, // rs2 select
            OPA_IS_RS1,
            OPB_IS_RS2,
            `ZERO_REG,
            ALU_ADD,
            1'b0, // rd_mem
            1'b0, // wr_mem
            1'b0, // cond
            1'b0, // uncond
            1'b0, // halt
            1'b0, // illegal
            1'b0, // csr_op
            3'b000,     // function type
            1'b0,  // valid
            0
        };
        clock = 0;

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        // Test that basic multiplication works
        is_ex_reg.rs1_value = 2;
        is_ex_reg.rs2_value = 3;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        //@(negedge free_mult[0]);
        wait_until_done_mult();
        assert(ex_packet.result == 6) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;

        // Test that basic alu operations work
        is_ex_reg.rs1_value = 6;
        is_ex_reg.rs2_value = 3;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_SUB;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_alu();
        assert(ex_packet.result == 3) else exit_on_error;
        assert(free_alu[0] == 1) else exit_on_error;

        @(negedge clock);

        // Test that multiplication with negative numbers works (MUL)
        is_ex_reg.rs1_value = -5;
        is_ex_reg.rs2_value = 2;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 32'b11111111111111111111111111110110) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test multiplication with upper bits (MULH)
        is_ex_reg.rs1_value = 123456789;
        is_ex_reg.rs2_value = 123456789;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MULH;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 32'b00000000001101100010011000100010) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test MULHSU (mixed sign and unsigned) ASK
        is_ex_reg.rs1_value = -34343434;
        is_ex_reg.rs2_value = 45454545;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MULHSU;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        // assert(mult_result == 32'b11111111111110100111010000111000) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test MULHU (unsigned)
        is_ex_reg.rs1_value = 34343434;
        is_ex_reg.rs2_value = 56565656;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MULHU;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 32'b00000000000001101110011011010110) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test MULHU on very very large values
        is_ex_reg.rs1_value = 32'hffff_ffff;
        is_ex_reg.rs2_value = 32'hffff_ffff;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MULHSU;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 32'b11111111111111111111111111111110) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test MUL on where inputs are the same binary as ^ but should give different product
        is_ex_reg.rs1_value = -1;
        is_ex_reg.rs2_value = -1;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 1) else exit_on_error;
        assert(free_mult[0] == 1) else exit_on_error;

        // Test that doing mult -> alu will still allow the output of mult to be present (+packet)
        // also test that the correct packet is outputted
        is_ex_reg.rs1_value = 6;
        is_ex_reg.rs2_value = 3;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;

        is_ex_reg.rs1_value = 7;
        is_ex_reg.rs2_value = 6;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_alu();
        assert(ex_packet.result == 13) else exit_on_error;
        assert(free_alu[0] == 1) else exit_on_error;
        assert(alu_packet.function_type == ALU) else exit_on_error;

        //@(negedge free_mult[0]);
        wait_until_done_mult();
        assert(ex_packet.result == 18) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;
        assert(mult_packet.function_type == MULT) else exit_on_error;
        assert(mult_packet.rs1_value == 6) else exit_on_error;

        // Maybe test whether fus can end simultaneously by starting a mult and then keeping the alu on until the mult ends? 
        is_ex_reg.rs1_value = 10;
        is_ex_reg.rs2_value = 11;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        is_ex_reg.rs1_value = 1;
        is_ex_reg.rs2_value = 10;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 110) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;
        assert(mult_packet.rs1_value == 10) else exit_on_error;
        is_ex_reg.valid = 0;
        @(negedge clock) 
        assert(ex_packet.result == 11) else exit_on_error;
        assert(free_alu[0] == 1'b1) else exit_on_error;
        assert(alu_packet.rs1_value == 1) else exit_on_error;

        // Add tests for branch... first, just checking the alu part works as expected (just like alu)
        is_ex_reg.rs1_value = 8;
        is_ex_reg.rs2_value = 10;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = BRANCH;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 18) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        
        // Test uncond branch
        is_ex_reg.rs1_value = 11;
        is_ex_reg.rs2_value = 3;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = BRANCH;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.uncond_branch = 1;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 14) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 1) else exit_on_error;
        assert(branch_packet.rs1_value == 11) else exit_on_error;

        // Test conditional branch
        is_ex_reg.rs1_value = 7;
        is_ex_reg.rs2_value = 5;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = BRANCH;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.inst.b.funct3 = 3'b001;
        is_ex_reg.uncond_branch = 0;
        is_ex_reg.cond_branch = 1;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 12) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 1) else exit_on_error;
        assert(branch_packet.rs1_value == 7) else exit_on_error;
        
        // Test conditional branch
        is_ex_reg.rs1_value = 6;
        is_ex_reg.rs2_value = 8;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = BRANCH;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.inst.b.funct3 = 3'b000;
        is_ex_reg.uncond_branch = 0;
        is_ex_reg.cond_branch = 1;
        is_ex_reg.valid = 1;
        issue_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 14) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 0) else exit_on_error;
        assert(branch_packet.rs1_value == 6) else exit_on_error;

        /*start = 1;
        a = 5;
        b = 50;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 0;
        b = 257;
        @(negedge clock);
        start = 0;
        wait_until_done();

        // change the monitor to hex for these values
        $monitor("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                 $time, done, a, b, result, cres);

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 64'hFFFF_FFFF_FFFF_FFFF;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 3;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'h5555_5555_5555_5555;
        b = 64'hCCCC_CCCC_CCCC_CCCC;
        @(negedge clock);
        start = 0;
        wait_until_done();

        $monitor(); // turn off monitor for the for-loop
        $display("\nBeginning random testing:");

        for (i = 0; i <= 15; i = i+1) begin
            start = 1;
            a = {$random, $random}; // multiply random 64-bit numbers
            b = {$random, $random};
            @(negedge clock);
            start = 0;
            wait_until_done();
            $display("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                     $time, done, a, b, result, cres);
        end*/

        $display("@@@ Passed\n");
        $finish;
    end

endmodule