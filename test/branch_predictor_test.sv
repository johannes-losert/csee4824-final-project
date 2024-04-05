`include "verilog/sys_defs.svh"

module testbench; 

    logic clock, reset; 

    logic [`XLEN-1:0] if_pc;

    logic [`XLEN-1:0] ex_pc;
    logic ex_branch_taken;

    logic predict_branch_taken; 

    branch_predictor dut (
        .clock(clock), 
        .reset(reset), 

        .if_pc(if_pc),

        .ex_pc(ex_pc),
        .ex_branch_taken(ex_branch_taken),

        .predict_branch_taken(predict_branch_taken)
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

    end 


endmodule