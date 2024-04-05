// this file is for testing the branch target buffer

`include "verilog/sys_defs.svh"

module testbench; 
    logic clock; 
    logic reset; 

    logic write_enable;
    logic [`XLEN-1:0] write_source_pc;
    logic [`XLEN-1:0] write_dest_pc;

    logic [`XLEN-1:0] query_pc;
    
    logic hit;
    logic [`XLEN-1:0] target_pc;

    btb dut (
        .clock(clock), 
        .reset(reset), 

        .write_enable(write_enable),
        .write_source_pc(write_source_pc),
        .write_dest_pc(write_dest_pc),

        .query_pc(query_pc), // the pc at which the current branch is 
    
        .hit(hit), // tells us if the target pc is valid
        .target_pc(target_pc) 
    );

    // logic clk;


    
    always begin       
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
        // counter_edges = counter_edges + 1; 

        // if (counter_edges == mem_latency_edges) begin
        //     clk = ~clk;
        //     counter_edges = 0; 
        // end

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
        @(negedge clock)
        // cycle 1
        $display("Test 1: At restart all entries should be invalid.");

        for (int i = 0; i < `BTB_ENTRIES; i = i + 1) begin
            query_pc = i; 
            @(negedge clock)
            $display("hit = %b", hit);  
            assert(hit == 0) else exit_on_error;;    
        end
        

        $display("@@@Passed");
        $finish;
    end


endmodule


