// This is the branch target buffer (BTB) module. It is a 
// cache that stores the target addresses of branches. 
// The module has inputs for the clock, reset signal, and various signals 
// from other modules such as the memory and fetch stage. 
// It also has outputs for commands to the memory, 
// data from the cache, and validity of the cache data.

`include "verilog/sys_defs.svh"

module btb (
    input logic clock, 
    input logic reset, 

    input logic write_enable,
    input logic [`XLEN-1:0] write_source_pc,
    input logic [`XLEN-1:0] write_dest_pc,

    input logic [`XLEN-1:0] query_pc, // the pc at which the current branch is 
    
    output logic hit, // tells us if the target pc is valid
    output logic [`XLEN-1:0] target_pc,

    output BTB_ENTRY debug_buffer [`BTB_ENTRIES-1:0]
);



    BTB_ENTRY buffer [`BTB_ENTRIES-1:0];
    BTB_ENTRY n_buffer [`BTB_ENTRIES-1:0];

    assign debug_buffer = buffer;

    // the last 4 bits of the query
    logic [`BTB_TAG_LEN-1:0] query_tag;
    assign query_tag = query_pc[`BTB_TAG_LEN:0];

    // we get the results of our query immediately
    assign target_pc = buffer[query_tag].target_pc;
    assign hit = buffer[query_tag].valid;

    // the last four bits of the write source pc
    logic [`BTB_TAG_LEN-1:0] write_tag;
    assign write_tag = write_source_pc[`BTB_TAG_LEN:0];


    always_comb begin
        if (reset) begin
            for (int i = 0; i < `BTB_ENTRIES; i = i + 1) begin
                n_buffer[i].target_pc = 0;
                n_buffer[i].valid = 0;
            end
        end else if (write_enable) begin
            n_buffer[write_tag].target_pc = write_dest_pc;
            n_buffer[write_tag].valid = 1;
        end 
    end

    always_ff @(posedge clock) begin
        buffer <= n_buffer;
    end


    



endmodule