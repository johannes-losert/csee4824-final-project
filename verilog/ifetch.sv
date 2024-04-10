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
    output [3:0] gnt_debug,
    output [`XLEN-1:0] PC_reg_debug,
    output fetch_available_debug,
    output IF_ID_PACKET inst_buffer_debug [`INSN_BUF_SIZE-1:0]
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching
    
    logic fetch_available; // if we are currently fetching an instruction in current cycle
    logic n_fetch_available; // if we will be fetching an instruction next cycle
    assign fetch_available_debug = fetch_available;

    IF_ID_PACKET inst_buffer [`INSN_BUF_SIZE-1:0];
    IF_ID_PACKET n_inst_buffer [`INSN_BUF_SIZE-1:0];
    IF_ID_PACKET n_if_packet;
    assign inst_buffer_debug = inst_buffer;

    logic [3:0] req;
    assign req = {certain_branch_req, rob_target_req, branch_pred_req, 1'b1};
    logic [3:0] gnt; 

    // DEBUG SIGNALS
    assign req_debug = req;
    assign gnt_debug = gnt;
    assign PC_reg_debug = PC_reg;
    
    psel_gen #(.WIDTH(4), .REQS(1)) 
    if_psel (
        .req(req),       
        .gnt(gnt),       
        .gnt_bus(), 
        .empty()  
    );

    always_comb begin

        // Select the next PC based four possible sources if we are not busy

        if (if_valid && fetch_available) begin 
            unique case (gnt)
                4'b1000 : n_PC_reg = certain_branch_pc;
                4'b0100 : n_PC_reg = rob_target_pc;
                4'b0010 : n_PC_reg = branch_pred_pc;
                4'b0001 : n_PC_reg = PC_reg + 4;
                default : n_PC_reg = 32'hdeadbeef;
            endcase
        end else begin 
            n_PC_reg = PC_reg;
        end
    
        // Perform the FIFO shift and load the next fetched instruction into the buffer 
        
        n_if_packet = inst_buffer[`INSN_BUF_SIZE-1];
        
        if (Icache2proc_data_valid) begin
            n_inst_buffer[0].inst = PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];
            n_inst_buffer[0].PC = PC_reg;
            n_inst_buffer[0].NPC = PC_reg + 4;
            n_inst_buffer[0].valid = 1;
            n_fetch_available = 1;
        end else begin
            n_inst_buffer[0].inst = `NOP;
            n_inst_buffer[0].PC = PC_reg;
            n_inst_buffer[0].NPC = PC_reg + 4;
            n_inst_buffer[0].valid = 0;
            n_fetch_available = 0;
        end

        for (int i = 0; i < `INSN_BUF_SIZE - 1; i++) begin
            n_inst_buffer[i+1] = inst_buffer[i];
        end
    end

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;
            fetch_available <= 1;
            for (int i = 0; i < `INSN_BUF_SIZE; i++) begin
                inst_buffer[i].inst <= `NOP;
                inst_buffer[i].PC <= 0;
                inst_buffer[i].NPC <= 0;
                inst_buffer[i].valid <= 0;
            end
        end else begin
            PC_reg <= n_PC_reg;
            inst_buffer <= n_inst_buffer;
            fetch_available <= n_fetch_available;
            if_packet <= n_if_packet;
        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};
endmodule




