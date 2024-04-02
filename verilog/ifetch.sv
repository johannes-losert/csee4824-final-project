`include "verilog/sys_defs.svh"

module ifetch (
    // GLOBAL VARIABLES
    input             clock,          // system clock
    input             reset,          // system reset
    
    // USED FOR STALLING
    input             if_valid,       // only go to next PC when true
    
    // BRANCH PREDICTOR INTERACTS HERE
    input [`XLEN-1:0] branch_target,  // target pc: use if take_branch is TRUE
    
    // FROM ICACHE
    input [63:0]      Imem2proc_data, // data coming back from Instruction memory
 
    output IF_ID_PACKET      if_packet,    
    output logic [`XLEN-1:0] proc2Icache_addr,  
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching




