`include "verilog/sys_defs.svh"
`include "verilog/psel_gen.sv"

// 1. EX_Branch 2. ROB_Target 3. Branch_Predictor 4. (PC+4)  = 4

// the size of our FIFO instruction buffer
`define INSN_BUF_SIZE 8

module ifetch (
    input             clock,          // system clock
    input             reset,          // system reset
    
    //          ***INPUTS***

    input             if_valid,       // only go to next PC when true
    
    // Program Counter Inputs   
    input [`XLEN-1:0] certain_branch_pc, 
    input certain_branch_req,

    input [`XLEN-1:0] rob_target_pc,
    input rob_target_req,
    input rob_stall, 

    input [`XLEN-1:0] branch_pred_pc,
    input branch_pred_req,

    // Icache Request Response
    input [63:0] Icache2proc_data, // data coming back from Instruction memory
    input Icache2proc_data_valid, // data coming back from Instruction memory

    //          *** OUTPUTS ***

    // To cache to get insn from memory
    output [`XLEN-1:0] proc2Icache_addr,
    // To decode
    output IF_ID_PACKET if_packet,    
    
    //          *** DEBUG ***
    output [3:0] req_debug,
    output [3:0] gnt_debug
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic fetch_busy; // if we are currently fetching an instruction in current cycle
    logic n_fetch_busy; // if we will be fetching an instruction next cycle
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching

    logic insn_buffer [`INSN_BUF_SIZE-1:0][63:0];
    logic n_insn_buffer [`INSN_BUF_SIZE-1:0][63:0];

    logic [`XLEN-1:0] if_packet_inst; 
    logic [`XLEN-1:0] n_if_packet_inst; // This should be a copy of the buffer head  
    assign  n_if_packet_inst = (~if_valid) ? `NOP :
                            PC_reg[2] ? insn_buffer[`INSN_BUF_SIZE-1][63:32] : insn_buffer[`INSN_BUF_SIZE-1][31:0];

    logic [3:0] req;
    assign req = {certain_branch_req, rob_target_req, branch_pred_req, 1'b1};
    logic [3:0] gnt; 

    // DEBUG SIGNALS
    assign req_debug = req;
    assign gnt_debug = gnt;

    psel_gen #(.WIDTH(4), .REQS(1)) 
    if_psel (
        .req(req),       
        .gnt(gnt),       
        .gnt_bus(), 
        .empty()  
    );

    always_comb begin

        // Select the next PC based four possible sources if we are not busy
        case ({gnt, fetch_busy})
            4'b10000 : n_PC_reg = certain_branch_pc;
            4'b01000 : n_PC_reg = rob_target_pc;
            4'b00100 : n_PC_reg = branch_pred_pc;
            4'b00010 : n_PC_reg = PC_reg + 4;
            4'bxxxx1 : n_PC_reg = PC_reg;
            default: n_PC_reg = PC_reg + 4; 
        endcase
    
        // Perform the FIFO shift and load the next fetched instruction into the buffer 
        if (Icache2proc_data_valid) begin
            n_insn_buffer[0] = Icache2proc_data;
        end else begin
            n_insn_buffer[0] = 0;
        end

        for (int i = 0; i < `INSN_BUF_SIZE; i++) begin
            n_insn_buffer[i+1] = insn_buffer[i];
        end

        // Update the fetch busy signal
        n_fetch_busy = !Icache2proc_data_valid; // we will be busy until this signal is valid again 
    end
        

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;
            insn_buffer <= 0;
            if_packet_inst <= `NOP; 
            fetch_busy <= 0; // the first cycle should be a fetch
        end else begin
            PC_reg <= n_PC_reg;
            insn_buffer <= n_insn_buffer;
            if_packet_inst <= n_if_packet_inst;
            fetch_busy <= n_fetch_busy;
        end
    end

    // Below is copied from P3

    // // address of the instruction we're fetching (64 bit memory lines)
    // // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    // assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};

    assign if_packet.inst = if_packet_inst;   
    assign if_packet.PC  = PC_reg;
    assign if_packet.NPC = PC_reg + 4; // pass PC+4 down pipeline w/instruction
    assign if_packet.valid = if_valid;

endmodule




