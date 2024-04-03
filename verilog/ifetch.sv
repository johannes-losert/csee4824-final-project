`include "verilog/sys_defs.svh"

module ifetch (
    input             clock,          // system clock
    input             reset,          // system reset
    input             if_valid,       // only go to next PC when true
    
    // BRANCH PREDICTOR INTERACTS HERE
    input [`XLEN-1:0] branch_target,  // target pc: use if take_branch is TRUE
    input             take_branch,    // taken-branch signal

    
    input rob_target_pc,
    input rob_target_valid,

    input branch_predictor_pc,
    input branch_predictor_valid,
    
    // FROM ICACHE
    input [63:0]      Icache2proc_data, // data coming back from Instruction memory
    input logic Icache2proc_data_valid, 

    //OUTPUTS 
    // To decode
    output IF_ID_PACKET      if_packet,    
    // To cache for next instruction
    output logic [`XLEN-1:0] proc2Icache_addr,  
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching

    logic 

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else if (take_branch) begin
            PC_reg <= branch_target; // update to a taken branch (does not depend on valid bit)
        end else if (if_valid) begin
            PC_reg <= PC_reg + 4;    // or transition to next PC if valid
        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};

    // this mux is because the Imem gives us 64 bits not 32 bits
    assign if_packet.inst = (~if_valid) ? `NOP :
                            PC_reg[2] ? Imem2proc_data[63:32] : Imem2proc_data[31:0];

    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction

    assign if_packet.valid = if_valid;

endmodule




