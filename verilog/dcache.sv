/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  dcache.sv                                           //
//                                                                     //
//  Description :  The data cache module that reroutes memory          //
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

/* Data cache handles loads (from stage_ex) and stores (from retire) */

/* STORE */
// Update dcache with new value
// Make request to mem to store new value

/* LOAD */
// If valid, return data
// If not valid, make request to memory

module dcache (
    input clock,
    input reset,

    // From memory
    input [ 3:0] Dmem2proc_response,  // Should be zero unless there is a response
    input [63:0] Dmem2proc_data,
    input [ 3:0] Dmem2proc_tag,

    // Store command addr (from retire stage)
    input logic store_en,
    input [`XLEN-1:0] store2Dcache_addr,
    input [63:0] store2Dcache_data,

    // Load command addr (from execute stage)
    input logic load_en,
    input [`XLEN-1:0] load2Dcache_addr,

    // To memory
    output logic [      1:0] proc2Dmem_command,
    output logic [`XLEN-1:0] proc2Dmem_addr,
    output logic [     63:0] proc2Dmem_data,

    // To load (stage ex)
    output [63:0] Dcache_data_out,  // Data is mem[proc2Dcache_addr]
    output Dcache_valid_out,  // When valid is high

    // DEBUG 
    output logic last_load_en,
    output logic last_store_en
);

  // ---- Cache data ---- //

  DCACHE_ENTRY [`CACHE_LINES-1:0] dcache_data;


  /* Calculate 'real' input to data cache, choosing between load and store.
    Data Cache can (for now) only take either a load or a store at the same time,
    so select whichever has raised enable */

  logic [`XLEN-1:0] proc2Dcache_addr;
  logic [63:0] proc2Dcache_data;  /* Only defined for stores */
  BUS_COMMAND proc2Dcache_command;

  always_comb begin
    assert (!(store_en && load_en));

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
  assign proc2Dmem_command = ((load_en != last_load_en) && load_en && !Dcache_valid_out) ? BUS_LOAD
                             : ((store_en != last_store_en) && store_en) ? BUS_STORE
                             : BUS_NONE;

  /* Calculate 64-bit data to send to memory */
  assign proc2Dmem_data = store_en ? proc2Dcache_data : 64'hdeadbeef;

  // ---- Main cache logic ---- //


  logic [3:0] current_mem_tag;  // The current memory tag we might be waiting on

  wire got_mem_data = (current_mem_tag == Dmem2proc_tag) && (current_mem_tag != 0);

  wire changed_addr = (current_index != last_index) || (current_tag != last_tag);

  // Set mem tag to zero if we changed_addr, then set to zero when we got_mem_data.
  wire update_mem_tag = changed_addr || got_mem_data;

  // TODO i think we are doing this twice, could remove
  assign proc2Dmem_addr = {proc2Dcache_addr[31:3], 3'b0};

  // ---- Cache state registers ---- //

  always_ff @(posedge clock) begin
    if (reset) begin
      last_index      <= -1;  // These are -1 to get ball rolling when
      last_tag        <= -1;  // reset goes low because addr "changes"
      current_mem_tag <= 0;
      dcache_data     <= 0;  // Set all cache data to 0 (including valid bits)
    end else begin
      last_index    <= current_index;
      last_tag      <= current_tag;


      last_load_en  <= load_en;
      last_store_en <= store_en;

      if (update_mem_tag) begin
        current_mem_tag <= Dmem2proc_response;
      end

      assert (!(got_mem_data && store_en));

      if (store_en) begin
        dcache_data[current_index].data  <= proc2Dcache_data;
        dcache_data[current_index].tags  <= current_tag;
        dcache_data[current_index].valid <= 1;
      end else if (got_mem_data) begin  // If data came from memory, meaning tag matches
        dcache_data[current_index].data  <= Dmem2proc_data;
        dcache_data[current_index].tags  <= current_tag;
        dcache_data[current_index].valid <= 1;
      end
    end
  end

endmodule  // icache
