`include "verilog/sys_defs.svh"


module testbench; 

    logic clock, reset; 

    // icache request
    BUS_COMMAND       icache_command;
    logic [`XLEN-1:0] icache_addr;
    
    // dcache request
    BUS_COMMAND dcache_command;
    logic [`XLEN-1:0] dcache_addr;

    // To memory
    BUS_COMMAND proc2mem_command;
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

    logic clk; // Memory clock

    logic [`XLEN-1:0] proc2mem_addr; // address for current command // support for memory model with byte level addressing
    logic [63:0] proc2mem_data; // address for current command
    // MEM_SIZE proc2mem_size; // BYTE, HALF, WORD or DOUBLE
    logic[1:0] proc2mem_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE

    logic [3:0]  mem2proc_response; // 0 = can't accept, other=tag of transaction
    logic [63:0] mem2proc_data;     // data resulting from a load
    logic [3:0]  mem2proc_tag;       // 0 = no value, other=tag of transaction

    mem mem_0 (
        .clk(clk),

        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data),
        // .proc2mem_size(proc2mem_size),
        .proc2mem_command(proc2mem_command),
        
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag)
    );


    // The latency of memory accesses in clock edges 
    integer mem_latency_edges = `MEM_LATENCY_IN_CYCLES * 2;
    // the number of elapsed clock edges
    integer counter_edges = 0; 

    always begin       
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
        counter_edges = counter_edges + 1; 

        if (counter_edges == mem_latency_edges) begin
            clk = ~clk;
            counter_edges = 0; 
        end
    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin 
        // $monitor();

        clock = 0;
        reset = 1;
        @(negedge clock)
        reset = 0;

        $display("Test case 1: Controller gets tag from mem.")
        icache_command = BUS_LOAD;
        icache_addr = 420;

        dcache_command = BUS_LOAD;
        dcache_addr = 69;
        
        #2; 
        assert(proc2mem_addr == dcache_addr) else exit_on_error;
        assert(mem2proc_response == 4'h1) else exit_on_error; 
        
        while (control2cache_response == 0) begin 
            @(negedge clock)
        end
        control2cache_response == 1

        $display("@@@Passed");
        $finish;
    end

endmodule