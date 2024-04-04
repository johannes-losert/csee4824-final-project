`include "verilog/sys_defs.svh"
`include "verilog/psel_gen.sv"

// 1. EX_Branch 2. ROB_Target 3. Branch_Predictor 3. (PC+4)  = 4

module ifetch (
    input             clock,          // system clock
    input             reset,          // system reset
    input             if_valid,       // only go to next PC when true
    
    input [`XLEN-1:0] certain_branch_pc, 
    input certain_branch_req,

    input [`XLEN-1:0] rob_target_pc,
    input rob_target_req,
    input rob_stall, 

    input [`XLEN-1:0] branch_pred_pc,
    input branch_pred_req,
    
    // FROM ICACHE
    input [63:0]      Icache2proc_data, // data coming back from Instruction memory
    input Icache2proc_data_valid, 

    //OUTPUTS 
    // To decode
    output IF_ID_PACKET      if_packet,    
    output [`XLEN-1:0] proc2Icache_addr
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching

    logic [3:0] req;
    logic [3:0] gnt; 

    psel_gen #(.WIDTH(4), .REQS(1)) (
        .req(req),       
        .gnt(gnt),       
        .gnt_bus(), 
        .empty()  
    );

    always_comb begin
        unique case (gnt)
            4'b1000 : n_PC_reg = certain_branch_pc;
            4'b0100 : n_PC_reg = rob_target_pc;
            4'b0010 : n_PC_reg = branch_pred_pc;
            4'b0001 : n_PC_reg = PC_reg + 4;
            default: n_PC_reg = PC_reg + 4; 
        endcase
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
                            PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];

    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction

    assign if_packet.valid = if_valid;

endmodule




