`include "verilog/sys_defs.svh"

module testbench; 

    logic clock, reset; 

    logic [`XLEN-1:0] if_pc;

    logic [`XLEN-1:0] ex_pc;
    logic ex_is_branch_taken, ex_is_branch_not_taken;

    // outputs
    logic predict_branch_taken, hit;

    branch_predictor dut (
        .clock(clock), 
        .reset(reset), 

        .if_pc(if_pc),

        .ex_pc(ex_pc),
        .ex_is_branch_taken(ex_is_branch_taken),
        .ex_is_branch_not_taken(ex_is_branch_not_taken),
        
        //outputs
        .predict_branch_taken(predict_branch_taken),
        .hit(hit)
    );

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
        // $monitor();

        clock     = 0;
        reset = 1;
        @(negedge clock)
        reset = 0;

        $display("Test 1: All branches predictions are no hit in beginning.");
            for (int i = 0; i < `BP_ENTRIES; i = i + 1) begin
                if_pc = i;
                @(negedge clock)
                #2;
                assert(predict_branch_taken == 0) else exit_on_error();
                assert(hit == 0) else exit_on_error();
            end
        $display("Test 1 Passed.");

        $display("Test 2: Some random branches, tracking multiple branches.");

        for (int i = 0; i < `BP_ENTRIES; i = i + 1) begin
            ex_pc = i;
            ex_is_branch_taken = 1;
            ex_is_branch_not_taken = 0;
            if_pc = i;
            
            @(negedge clock)
            #2;
            assert(predict_branch_taken == 0) else exit_on_error();
            assert(hit == 1) else exit_on_error();

            @(negedge clock)
            #2;
            assert(predict_branch_taken == 1) else exit_on_error();
            assert(hit == 1) else exit_on_error();

            ex_pc = i;
            ex_is_branch_taken = 0;
            ex_is_branch_not_taken = 1;
            
            @(negedge clock)
            #2;
            assert(predict_branch_taken == 0) else exit_on_error();
            assert(hit == 1) else exit_on_error();

            ex_pc = i;
            ex_is_branch_taken = 1;
            ex_is_branch_not_taken = 0;
            @(negedge clock)
            @(negedge clock)
            @(negedge clock)
            #2;
            assert(predict_branch_taken == 1) else exit_on_error();
            assert(hit == 1) else exit_on_error();

            ex_pc = i;
            ex_is_branch_taken = 0;
            ex_is_branch_not_taken = 1;
            #2;
            assert(predict_branch_taken == 1) else exit_on_error();
            assert(hit == 1) else exit_on_error();

        end

        $display("Test 2 Passed.");


        $display("@@@Passed");
        $finish;
    end 


endmodule