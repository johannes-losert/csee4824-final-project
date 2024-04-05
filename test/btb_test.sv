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
        // cycle 1
        $display("Test 1: At restart all entries should be invalid.");

        for (int i = 0; i < `BTB_ENTRIES; i = i + 1) begin
            query_pc = i; 
            @(negedge clock)
            assert(hit == 0) else exit_on_error;    
        end

        $display("Test 1 Passed.");

        @(negedge clock)
        
        
        $display("Test 2: Write to all entries and check if they are valid.");
        write_enable = 1;
        for (int i = 0; i < `BTB_ENTRIES; i = i + 1) begin
            @(negedge clock)
            write_source_pc = i;
            write_dest_pc = 420 + i;
        end
        #2; // give the last write time to pass through
        write_enable = 0;

        for (int i = 0; i < `BTB_ENTRIES; i = i + 1) begin
            @(negedge clock)
            query_pc = i;
            #2; // give enough time for value to stabilize
            $display("hit = %b, target_pc = %d", hit, target_pc);  
            assert(hit) else exit_on_error;
            assert(target_pc == 420 + i) else exit_on_error;
        end

        $display("Test 2 Passed.");

        $display("@@@Passed");
        $finish;
    end


endmodule


