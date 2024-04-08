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

endmodule