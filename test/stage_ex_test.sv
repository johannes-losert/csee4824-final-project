
`include "verilog/sys_defs.svh"

module testbench;

    logic [63:0] mult_result;
    logic clock, reset, free_alu, free_mult, free_load, free_store, alu_en, mult_en;
    integer i;
    ID_EX_PACKET id_ex_reg;
    FUNIT funit;
    EX_MEM_PACKET ex_packet;


    stage_ex dut(
        .clock(clock),
        .reset(reset),
        .id_ex_reg(id_ex_reg),
        .funit(funit),
        .alu_en(alu_en),
        .mult_en(mult_en),
        .ex_packet(ex_packet),
        .mult_result(mult_result),
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load), 
        .free_store(free_store)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done;
        forever begin : wait_loop
            @(posedge free_mult);
            @(negedge clock);
            if (free_mult) begin
                disable wait_until_done;
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
        $monitor("Time:%4.0f opa:%5d opb:%5d alu_result:%5d mult_result:%5d funit:%2h free_alu:%h free_mult:%h",
                 $time, id_ex_reg.rs1_value, id_ex_reg.rs2_value, ex_packet.alu_result, mult_result, funit, free_alu, free_mult);

        $display("\nBeginning edge-case testing:");

        id_ex_reg <= '{
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
            1'b0  // valid
        };
        funit = 0;
        clock = 0;
        alu_en = 0;
        mult_en = 0;

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        // Test that basic multiplication works
        id_ex_reg.rs1_value = 2;
        id_ex_reg.rs2_value = 3;
        id_ex_reg.opa_select = OPA_IS_RS1;
        id_ex_reg.opb_select = OPB_IS_RS2;
        funit = MULT;
        mult_en = 1;
        @(negedge clock);
        mult_en = 0;
        wait_until_done();
        assert(mult_result == 6) else exit_on_error;
        assert(free_mult == 1) else exit_on_error;

        // Test that basic alu operations work
        id_ex_reg.rs1_value = 6;
        id_ex_reg.rs2_value = 3;
        id_ex_reg.opa_select = OPA_IS_RS1;
        id_ex_reg.opb_select = OPB_IS_RS2;
        funit = ALU;
        id_ex_reg.alu_func = ALU_SUB;
        alu_en = 1;
        @(negedge clock);
        alu_en = 0;
        assert(ex_packet.alu_result == 3) else exit_on_error;
        assert(free_alu == 1) else exit_on_error;

        /*// Test that multiplication with negative numbers works
        id_ex_reg.rs1_value = -5;
        id_ex_reg.rs2_value = 2;
        id_ex_reg.opa_select = OPA_IS_RS1;
        id_ex_reg.opb_select = OPB_IS_RS2;
        funit = MULT;
        mult_en = 1;
        @(negedge clock);
        mult_en = 0;
        wait_until_done();
        assert(mult_result == -10) else exit_on_error;
        assert(free_mult == 1) else exit_on_error;*/

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
