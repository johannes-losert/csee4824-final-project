/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  icache.sv                                           //
//                                                                     //
//  Description :  The instruction cache module that reroutes memory   //
//                 accesses to decrease misses.                        //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// Internal macros, no other file should need these
`define CACHE_LINES 32
`define CACHE_LINE_BITS $clog2(`CACHE_LINES)

typedef struct packed {
    logic [63:0]                  data;
    // (13 bits) since only need 16 bits to access all memory and 3 are the offset
    logic [12-`CACHE_LINE_BITS:0] tags;
    logic                         valid;
} DCACHE_ENTRY;

/**
 * A quick overview of the cache and memory:
 *
 * We've increased the memory latency from 1 cycle to 100ns. which will be
 * multiple cycles for any reasonable processor. Thus, memory can have multiple
 * transactions pending and coordinates them via memory tags (different meaning
 * than cache tags) which represent a transaction it's working on. Memory tags
 * are 4 bits long since 15 mem accesses can be live at one time, and only one
 * access happens per cycle.
 *
 * On a request, memory *responds* with the tag it will use for that request
 * then ceiling(100ns/clock period) cycles later, it will return the data with
 * the corresponding tag. The 0 tag is a sentinel value and unused. It would be
 * very difficult to push your clock period past 100ns/15=6.66ns, so 15 tags is
 * sufficient.
 *
 * This cache coordinates those memory tags to speed up fetching reused data.
 *
 * Note that this cache is blocking, and will wait on one memory request before
 * sending another (unless the input address changes, in which case it abandons
 * that request). Implementing a non-blocking cache can count towards simple
 * feature points, but will require careful management of memory tags.
 */

module dcache (
    input clock,
    input reset,

    // From memory
input [3:0]  Dmem2proc_response, // Should be zero unless there is a response
    input [63:0] Dmem2proc_data,
    input [3:0]  Dmem2proc_tag,

    // Store command addr (from retire stage)
    input logic store_en,
    input [`XLEN-1:0] store2Dcache_addr,
    input [`XLEN-1:0] store2Dcache_data,

    // Load command addr (from execute stage)
    input logic load_en,
    input [`XLEN-1:0] load2Dcache_addr,

    // To memory
    output [1:0]       proc2Dmem_command,
    output [`XLEN-1:0] proc2Dmem_addr,
    output [`XLEN-1:0] proc2Dmem_data,

    // To load (stage ex)
    output [63:0] Dcache_data_out, // Data is mem[proc2Dcache_addr]
    output Dcache_valid_out // When valid is high
);

    // ---- Cache data ---- //

    DCACHE_ENTRY [`CACHE_LINES-1:0] dcache_data;


    /* Calculate 'real' input to data cache, choosing between load and store.
    Data Cache can (for now) only take either a load or a store at the same time,
    so select whichever has raised enable  
    */

    logic [`XLEN-1:0] proc2Dcache_addr;
    logic [`XLEN-1:0] proc2Dcache_data; /* Only defined for stores */  
    BUS_COMMAND proc2Dcache_command;

    always_comb begin
        assert(!(store_en && load_en));

        if (store_en) begin 
            proc2Dcache_command = BUS_STORE;
            proc2Dcache_data = store2Dcache_data;
            proc2Dcache_addr = store2Dcache_addr;
        end else if (load_en) begin
            proc2Dcache_command = BUS_LOAD;
            proc2Dcache_data = 32'hdeadbeef;
            proc2Dcache_addr = load2Dcache_addr;
        end else begin
            proc2Dcache_command = BUS_NONE;
            proc2Dcache_data = 32'hdeadbeef;
            proc2Dcache_addr = 32'hdeadbeef;
        end
    end


    /* STORE */
    /* Update dcache with new value */
    /* Make request to mem to store new value */

    /* LOAD */
    /* If valid, return data */
    /* If not valid, make request to memory */

    // ---- Addresses and final outputs ---- //

    // Note: cache tags, not memory tags
    logic [12-`CACHE_LINE_BITS:0] current_tag, last_tag;
    logic [`CACHE_LINE_BITS - 1:0] current_index, last_index;

    assign {current_tag, current_index} = proc2Dcache_addr[15:3];

    assign Dcache_data_out = dcache_data[current_index].data;
    assign Dcache_valid_out = load_en && !store_en && dcache_data[current_index].valid &&
                              (dcache_data[current_index].tags == current_tag);


    // If load_en and invalid, send BUS_LOAD to memory
    // If store_en, send BUS_STORE to memory
    // Otherwise, send BUS_NONE
    assign proc2Dmem_command = (load_en && !Dcache_valid_out) ? BUS_LOAD
                             : (store_en) ? BUS_STORE
                             : BUS_NONE;

    assign proc2Dmem_data = (store_en) ? proc2Dcache_data : 32'hdeadbeef;


    // ---- Main cache logic ---- //

    logic [3:0] current_mem_tag; // The current memory tag we might be waiting on
    logic miss_outstanding; // Whether a miss has received its response tag to wait on

    wire got_mem_data = (current_mem_tag == Dmem2proc_tag) && (current_mem_tag != 0);

    wire changed_addr = (current_index != last_index) || (current_tag != last_tag);

    // Set mem tag to zero if we changed_addr, and keep resetting while there is
    // a miss_outstanding. Then set to zero when we got_mem_data.
    // (this relies on Imem2proc_response being zero when there is no request)
    wire update_mem_tag = changed_addr || miss_outstanding || got_mem_data;

    // If we have a new miss or still waiting for the response tag, we might
    // need to wait for the response tag because dcache has priority over icache
    wire unanswered_miss = changed_addr ? !Dcache_valid_out
                                        : miss_outstanding && (Dmem2proc_response == 0);

    // Keep sending memory requests until we receive a response tag or change addresses
    //assign proc2Dmem_command = (miss_outstanding && !changed_addr) ? BUS_LOAD : BUS_NONE;
    assign proc2Dmem_addr    = {proc2Dcache_addr[31:3],3'b0};

    // ---- Cache state registers ---- //

    always_ff @(posedge clock) begin
        if (reset) begin
            last_index       <= -1; // These are -1 to get ball rolling when
            last_tag         <= -1; // reset goes low because addr "changes"
            current_mem_tag  <= 0;
            miss_outstanding <= 0;
            dcache_data      <= 0; // Set all cache data to 0 (including valid bits)
        end else begin
            last_index       <= current_index;
            last_tag         <= current_tag;
            miss_outstanding <= unanswered_miss;
            if (update_mem_tag) begin
                current_mem_tag <= Dmem2proc_response;
            end

            assert(!(got_mem_data && store_en));

            if (store_en) begin
                dcache_data[current_index].data  <= proc2Dcache_data;
                dcache_data[current_index].tags  <= current_tag;
                dcache_data[current_index].valid <= 1;
            end else if (got_mem_data) begin // If data came from memory, meaning tag matches
                dcache_data[current_index].data  <= Dmem2proc_data;
                dcache_data[current_index].tags  <= current_tag;
                dcache_data[current_index].valid <= 1;
                // assert(load_en);
            end
            // end else begin
            //     dcache_data[current_index].valid <= dcache_data[current_index].valid;
            //     dcache_data[current_index].tags  <= 0;
            //     dcache_data[current_index].data  <= 0;
            //     assert(0); // Should never get here
            // end 
        end
    end

endmodule // icache
