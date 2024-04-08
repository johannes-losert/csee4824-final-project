`include "verilog/sys_defs.svh"


module testbench; 

    logic clock, reset; 

    // icache request
    logic [1:0]       icache_command;
    logic [`XLEN-1:0] icache_addr;
    logic icache_is_miss;
    
    // dcache request
    logic [1:0] dcache_command;
    logic [`XLEN-1:0] dcache_addr;

    // To memory
    logic [1:0]       proc2mem_command;
    logic [63:0] proc2mem_addr;
    
    // From memory
    logic [3:0]  mem2proc_response; // Should be zero unless there is a response
    logic [63:0] mem2proc_data;
    logic [3:0]  mem2proc_tag;

    // to caches (both are connected)
    logic [3:0]  control2cache_response; // Should be zero unless there is a response
    logic [3:0] control2cache_response_which; // always either ICACHE, or DCACHE
    logic [63:0] control2cache_data; 
    logic [3:0]  control2cache_tag;
    logic [1:0] control2cache_tag_which; 

    mem_controller dut (
        .clock(clock),
        .reset(reset),
        .icache_command(icache_command),
        .icache_addr(icache_addr),
        .icache_is_miss(icache_is_miss),
        .dcache_command(dcache_command),
        .dcache_addr(dcache_addr),
        .proc2mem_command(proc2mem_command),
        .proc2mem_addr(proc2mem_addr),
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag),
        .control2cache_response(control2cache_response),
        .control2cache_response_which(control2cache_response_which),
        .control2cache_data(control2cache_data),
        .control2cache_tag(control2cache_tag),
        .control2cache_tag_which(control2cache_tag_which)
    );

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


        $display("@@@Passed");
        $finish;
    end

endmodule