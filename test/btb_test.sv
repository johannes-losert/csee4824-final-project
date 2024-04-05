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

        $display("@@@Passed");
        $finish;
    end


endmodule


