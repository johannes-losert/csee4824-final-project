`include "verilog/sys_defs.svh"

// ROB_Target + Branch_Predictor + (PC+4) + EX_Branch = 4
`define REQ 4 

module ifetch (
    input             clock,          // system clock
    input             reset,          // system reset
    input             if_valid,       // only go to next PC when true
    
    input [`XLEN-1:0] certain_branch_pc,  // target pc: use if take_branch is TRUE
    input             certain_branch_req,    // taken-branch signal ONLY FROM EX

    // Question for Tanvir: take branch forces the change of the program counter and should only be high
    // when we are absolutely confident that the branch will be taken - i.e. in the commit stage. 

    input rob_target_pc,
    input rob_target_req,
    input rob_stall, 

    input branch_pred_pc,
    input branch_pred_req,
    
    // FROM ICACHE
    input [63:0]      Icache2proc_data, // data coming back from Instruction memory
    input logic Icache2proc_data_valid, 

    //OUTPUTS 
    // To decode
    output IF_ID_PACKET      if_packet,    
    output logic [`XLEN-1:0] proc2Icache_addr,  
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching


    logic ['REQ-1:0] req;
    logic ['REQ-1:0] gnt; 

    psel_gen #(
        .REQS(1),     // This is internal
        .WIDTH(REQ)    // Custom width
    ) psel_gen_instance (
        .req(req),       
        
        .gnt(gnt),       
        .gnt_bus(), 
        .empty()  
    );

    always_comb begin
        unique case (gnt) :
        
        // TODO fill in the selector logic

        default: n_PC_reg = PC_reg + 4 
    end
        

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;             // initial PC value is 0 (the memory address where our program starts)
        end else begin
            PC_reg <= n_PC_reg;
        end
    end

    // Below is copied from P3

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




