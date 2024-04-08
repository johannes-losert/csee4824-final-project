`include "verilog/sys_defs.svh"

/*
    Memory Controller by Johannes Losert (johannes.losert@columbia.edu)

    This module is responsible for deliberating between memory requests from caches
    and forwarding satisfied requests to the correct location. The purpose of this 
    module is to remove complexity from our final pipeline by providing a more natural 
    interface for the caches to interact with memory. 

    The controller is responsible for the following:
        - Ensuring that dcache requests get priority over icache requests
        - Forwarding requests to memory
        - Forwarding responses to the correct cache
        - Managing the state of memory tags

    IMPORTANT: The controller does not currently output whether memory can accept more requests
    I want to implement it soon.

*/ 

module mem_controller (
    input clock, 
    input reset, 

    // icache request
    input BUS_COMMAND       icache_command,
    input logic [`XLEN-1:0] icache_addr,
    
    // dcache request
    input BUS_COMMAND       dcache_command,
    input logic [`XLEN-1:0] dcache_addr,

    // To memory
    output BUS_COMMAND       proc2mem_command,
    output logic [`XLEN-1:0] proc2mem_addr,
    
    // From memory
    input logic [3:0]  mem2proc_response, // Should be zero unless there is a response
    input logic [63:0] mem2proc_data,
    input logic [3:0]  mem2proc_tag,

    output REQ_STATUS current_request_status,

    // to caches (both are connected)
    output logic [3:0]  control2cache_response, // Should be zero unless there is a response
    output DEST_CACHE control2cache_response_which, // always either ICACHE, or DCACHE
    output logic [63:0] control2cache_data, 
    output logic [3:0]  control2cache_tag,
    output DEST_CACHE control2cache_tag_which
); 

    // we index by tracking number
    DEST_CACHE controller_table [3:0]; 
    DEST_CACHE n_controller_table [3:0]; 

    REQ_STATUS next_request_status;
    
    /* Memory Request Response (Tag Acquisition) */ 
    logic no_grant, dcache_req_granted, icache_req_granted;

    /* logic no_grant_possible: This is a future feature */ 

    assign dcache_req_granted = (dcache_command != BUS_NONE); 
    assign icache_req_granted = (icache_command == BUS_LOAD) && (dcache_command == BUS_NONE);
    assign no_grant = !dcache_req_granted || !icache_req_granted; // all zeros, no tag granted
    

    /* Memory Data Response (Data Acquisition, Tag Freed) */ 
    // has the memory responded with data? 
    logic mem_has_data;
    assign mem_has_data = |mem2proc_tag; 

    logic mem_has_tag;
    assign mem_has_tag = |mem2proc_response;

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

        if (dcache_req_granted) begin
            next_request_status = AWAIT_TAG_DCACHE;
        end else if (icache_req_granted) begin 
            next_request_status = AWAIT_TAG_ICACHE;
        end else if (mem_has_tag) begin
            next_request_status = TAG_AVAILABLE;
        end else begin
            next_request_status = NO_REQUEST;
        end
    end

    always_ff @(posedge clock) begin
        controller_table <= n_controller_table;
        current_request_status <= next_request_status;
    end
endmodule 