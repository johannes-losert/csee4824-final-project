`include "verilog/sys_defs.svh"

module mem_controller (
    input clock, 
    input reset, 

    // icache request
    input logic [1:0]       icache_command,
    input logic [`XLEN-1:0] icache_addr,
    input logic icache_is_miss,
    
    // dcache request
    input logic [1:0]       dcache_command,
    input logic [`XLEN-1:0] dcache_addr,

    // To memory
    output logic [1:0]       proc2mem_command,
    output logic [63:0] proc2mem_addr,
    
    // From memory
    input logic [3:0]  mem2proc_response, // Should be zero unless there is a response
    input logic [63:0] mem2proc_data,
    input logic [3:0]  mem2proc_tag,

    // to caches (both are connected)
    output logic [3:0]  control2cache_response, // Should be zero unless there is a response
    output DEST_CACHE control2cache_response_which, // always either ICACHE, or DCACHE
    output logic [63:0] control2cache_data, 
    output logic [3:0]  control2cache_tag,
    output DEST_CACHE control2cache_tag_which
); 

    // we index by tracking number
    logic [3:0] [1:0] controller_table; 
    logic [3:0] [1:0] n_controller_table; 
    
    /* Memory Request Response (Tag Acquisition) */ 
    logic no_grant, dcache_req_granted, icache_req_granted;

    assign no_grant = ~&mem2proc_response; // all zeros, no tag granted
    assign dcache_req_granted = (dcache_command != BUS_NONE) && !no_grant; 
    assign icache_req_granted = (icache_command == BUS_LOAD) && (dcache_command == BUS_NONE) && !no_grant;

    /* Memory Data Response (Data Acquisition, Tag Freed) */ 
    // has the memory responded with data? 
    logic mem_has_data;
    assign mem_has_data = |mem2proc_tag; 

    // in each cycle we get AT MOST one response from memory 
    always_comb begin
        if (reset) begin
            for (int i = 0; i < 16; i = i + 1) begin
                n_controller_table[i] = NONE;
            end
        end
        
        if (icache_req_granted) begin
            n_controller_table[mem2proc_response] = ICACHE;
            proc2mem_command = icache_command;
            proc2mem_addr = icache_addr;
            control2cache_response_which = ICACHE; 
        end else if (dcache_req_granted) begin
            n_controller_table[mem2proc_response] = DCACHE;
            proc2mem_command = dcache_command;
            proc2mem_addr = dcache_addr;
            control2cache_response_which = DCACHE; 
        end 

        if (mem_has_data) begin
            control2cache_tag = mem2proc_tag;
            control2cache_data = mem2proc_data;
            control2cache_tag_which = controller_table[mem2proc_tag];
        end else begin 
            control2cache_tag = 0;
            control2cache_data = 0;
            control2cache_tag_which = NONE;
        end
    end

    always_ff @(posedge clock) begin
        controller_table <= n_controller_table;
    end
endmodule 