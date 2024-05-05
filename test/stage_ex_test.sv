
`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

module testbench;

    logic [`XLEN-1:0] mult_result, branch_result, Dmem2proc_data, proc2Dmem_addr, proc2Dmem_data, load2Dcache_addr;
    logic [63:0] Dcache_data_out;
    logic clock, reset, Dcache_valid_out, load_en, rollback;
    logic [`NUM_FU_ALU-1:0] free_alu;
    logic [`NUM_FU_MULT-1:0] free_mult;
    logic [`NUM_FU_LOAD-1:0] free_load;
    logic [`NUM_FU_STORE-1:0] free_store;
    logic [`NUM_FU_BRANCH-1:0] free_branch;
    integer i;
    IS_EX_PACKET is_ex_reg;
    IS_EX_PACKET [`NUM_FU_ALU-1:0] alu_packet;
    IS_EX_PACKET [`NUM_FU_MULT-1:0] mult_packet;
    IS_EX_PACKET [`NUM_FU_BRANCH-1:0] branch_packet;
    IS_EX_PACKET [`NUM_FU_LOAD-1:0] load_packet;
    IS_EX_PACKET [`NUM_FU_STORE-1:0] store_packet;
    EX_CO_PACKET ex_packet;
    logic [3:0] Dmem2proc_response;
    logic [1:0] proc2Dmem_command;
    MEM_SIZE proc2Dmem_size;

    stage_ex dut(
        .clock(clock),
        .reset(reset),
        .is_ex_reg(is_ex_reg),
        .ex_packet(ex_packet),
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load), 
        .free_store(free_store),
        .free_branch(free_branch),
        .Dcache_data_out(Dcache_data_out),
        .Dcache_valid_out(Dcache_valid_out),
        .ex_load_en(load_en),
        .ex_load2Dcache_addr(load2Dcache_addr),
        .rollback(rollback),
        .alu_packet(alu_packet),
        .mult_packet(mult_packet),
        .branch_packet(branch_packet),
        .load_packet(load_packet),
        .store_packet(store_packet)
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

    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done_load;
        forever begin : wait_loop
            $display("Dmem2proc_data: %h, proc2Dmem_addr: %h, proc2Dmem_data: %h, Dmem2proc_response: %h, proc2Dmem_command: %h, proc2Dmem_size: %h", 
                    Dmem2proc_data, proc2Dmem_addr, proc2Dmem_data, Dmem2proc_response, proc2Dmem_command, proc2Dmem_size);
            @(posedge free_load[0]);
            @(negedge clock);
            if (free_load[0]) begin
                disable wait_until_done_load;
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
        $monitor("Time:%4.0f opa:%d opb:%d alu_func:%d func:%b result:%d funit:%2h free_alu:%b free_mult:%b free_branch:%b take_branch:%b free_load:%b",
                 $time, $signed(is_ex_reg.rs1_value), $signed(is_ex_reg.rs2_value), is_ex_reg.alu_func, is_ex_reg.inst.b.funct3, $signed(ex_packet.result), is_ex_reg.function_type, free_alu, free_mult, free_branch, ex_packet.take_branch, free_load);
        $display("\nBeginning edge-case testing:");

        is_ex_reg <= '{
            `NOP, // we can't simply assign 0 because NOP is non-zero
            {`XLEN{1'b0}}, // PC
            {`XLEN{1'b0}}, // NPC
            {`XLEN{1'b0}}, // opa select
            {`XLEN{1'b0}}, // opb select
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
            0, //rob index
            0, //has dest
            0,   //issued_fu_index
            0 //arch_dest_reg_num
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
        is_ex_reg.issued_fu_index = 0;
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
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_alu();
        $display(ex_packet.rs1_value);
        $display(alu_packet[0].rs1_value);
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
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;

        is_ex_reg.rs1_value = 7;
        is_ex_reg.rs2_value = 6;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_alu();
        assert(ex_packet.result == 13) else exit_on_error;
        assert(free_alu[0] == 1) else exit_on_error;
        assert(alu_packet[0].function_type == ALU) else exit_on_error;

        //@(negedge free_mult[0]);
        wait_until_done_mult();
        assert(ex_packet.result == 18) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;
        assert(mult_packet[0].function_type == MULT) else exit_on_error;
        assert(mult_packet[0].rs1_value == 6) else exit_on_error;

        // Maybe test whether fus can end simultaneously by starting a mult and then keeping the alu on until the mult ends? 
        is_ex_reg.rs1_value = 10;
        is_ex_reg.rs2_value = 11;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        is_ex_reg.rs1_value = 1;
        is_ex_reg.rs2_value = 10;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
        wait_until_done_mult();
        assert(ex_packet.result == 110) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;
        assert(mult_packet[0].rs1_value == 10) else exit_on_error;
        is_ex_reg.valid = 0;
        wait_until_done_alu();
        assert(ex_packet.result == 11) else exit_on_error;
        assert(free_alu[0] == 1'b1) else exit_on_error;
        assert(alu_packet[0].rs1_value == 1) else exit_on_error;

        // Add tests for branch... first, just checking the alu part works as expected (just like alu)
        is_ex_reg.rs1_value = 8;
        is_ex_reg.rs2_value = 10;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = BRANCH;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
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
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 14) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 1) else exit_on_error;
        assert(branch_packet[0].rs1_value == 11) else exit_on_error;

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
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 12) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 1) else exit_on_error;
        assert(branch_packet[0].rs1_value == 7) else exit_on_error;
        
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
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        is_ex_reg.valid = 0;
        wait_until_done_branch();
        assert(ex_packet.result == 14) else exit_on_error;
        assert(free_branch[0] == 1) else exit_on_error;
        assert(ex_packet.take_branch == 0) else exit_on_error;
        assert(branch_packet[0].rs1_value == 6) else exit_on_error;

        // Test that the second fu of each type works (multiplication)
        is_ex_reg.rs1_value = 25;
        is_ex_reg.rs2_value = 2;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        //@(negedge free_mult[0]);
        @(posedge free_mult[1]);
        //wait_until_done_mult();
        assert(ex_packet.result == 50) else exit_on_error;
        assert(free_mult[1] == 1'b1) else exit_on_error;

        // Test that I can call 2 mults on different fus and things will still work
        is_ex_reg.rs1_value = 9;
        is_ex_reg.rs2_value = 6;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        @(negedge clock);
        is_ex_reg.rs1_value = 7;
        is_ex_reg.rs2_value = 4;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = MULT;
        is_ex_reg.alu_func = ALU_MUL;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        @(posedge free_mult[0]);
        assert(ex_packet.result == 54) else exit_on_error;
        assert(free_mult[0] == 1'b1) else exit_on_error;
        @(posedge free_mult[1]); 
        $display(ex_packet.result);
        assert(ex_packet.result == 28) else exit_on_error;
        assert(free_mult[1] == 1'b1) else exit_on_error;

        // Do the same thing for alu (does it still require 2 clock cycles to work?) 
        is_ex_reg.rs1_value = 2;
        is_ex_reg.rs2_value = 4;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_ADD;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 0;
        @(negedge clock);
        @(negedge clock);
        is_ex_reg.rs1_value = 12;
        is_ex_reg.rs2_value = 7;
        is_ex_reg.opa_select = OPA_IS_RS1;
        is_ex_reg.opb_select = OPB_IS_RS2;
        is_ex_reg.function_type = ALU;
        is_ex_reg.alu_func = ALU_SUB;
        is_ex_reg.valid = 1;
        is_ex_reg.issued_fu_index = 1;
        @(negedge clock);
        is_ex_reg.valid = 0;
        assert(ex_packet.result == 6) else exit_on_error;
        assert(free_alu[0] == 1'b1) else exit_on_error;
        @(negedge clock);
        assert(ex_packet.result == 5) else exit_on_error;
        assert(free_alu[1] == 1'b1) else exit_on_error;

        $display("@@@ Passed\n");
        $finish;
    end

endmodule