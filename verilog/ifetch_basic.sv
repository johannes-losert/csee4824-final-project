// `include "verilog/sys_defs.svh"
// `include "verilog/psel_gen.sv"

// 1. EX_Branch 2. ROB_Target 3. Branch_Predictor 4. (PC+4)  = 4

// the size of our FIFO instruction buffer
`define INST_BUF_SIZE 8
`define INST_INDEX_SIZE $clog2(`INST_BUF_SIZE)

module ifetch_basic (
    input             clock,          // system clock
    input             reset,          // system reset
    
    //          ***INPUTS***

    input             if_valid,       // only pop the the instruction buffer head if valid
    
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

    // To cache to get inst from memory
    output [`XLEN-1:0] proc2Icache_addr,
    // To decode
    output IF_ID_PACKET if_packet,
    
    //          *** DEBUG ***
    output [3:0] req_debug,
    output [3:0] gnt_debug,
    output [`XLEN-1:0] PC_reg_debug,
    output waiting_on_inst_debug,
    output [`INST_INDEX_SIZE-1:0] inst_buffer_tail_debug
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching
    
    logic waiting_on_inst;
    logic n_waiting_on_inst;  
    assign waiting_on_inst_debug = waiting_on_inst;
    
    logic [3:0] req;
    assign req = {certain_branch_req, rob_target_req, branch_pred_req, 1'b1};
    logic [3:0] gnt; 

    IF_ID_PACKET n_if_packet;

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

        if (if_valid) begin         
            if (!waiting_on_inst) begin 
                unique case (gnt)
                    4'b1000 : n_PC_reg = certain_branch_pc;
                    4'b0100 : n_PC_reg = rob_target_pc;
                    4'b0010 : n_PC_reg = branch_pred_pc;
                    4'b0001 : n_PC_reg = PC_reg + 4;
                    default : n_PC_reg = 32'hdeadbeef;
                endcase
                n_waiting_on_inst = 1;

                n_if_packet.inst = `NOP;
                n_if_packet.PC = PC_reg;
                n_if_packet.NPC = PC_reg + 4;
                n_if_packet.valid = 0;
            end else if (waiting_on_inst) begin 
                n_PC_reg = PC_reg;
                
                if (Icache2proc_data_valid) begin
                    // pushing to the tail of inst buffer
                    n_waiting_on_inst = 0; 
                    n_if_packet.inst = PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];
                    n_if_packet.PC = PC_reg;
                    n_if_packet.NPC = PC_reg + 4;
                    n_if_packet.valid = 1;
                end else if (!Icache2proc_data_valid) begin
                    n_waiting_on_inst = 1; 
                    n_if_packet.inst = `NOP;
                    n_if_packet.PC = PC_reg;
                    n_if_packet.NPC = PC_reg + 4;
                    n_if_packet.valid = 0;
                end
            end
        end else begin
            n_PC_reg = PC_reg;
            n_waiting_on_inst = waiting_on_inst;
            n_if_packet = if_packet;
        end
    end

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            waiting_on_inst <= 1;
            PC_reg <= 0;
            if_packet <= 0;
        end else begin
            PC_reg <= n_PC_reg;
            waiting_on_inst <= n_waiting_on_inst;
            if_packet <= n_if_packet;
        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Icache_addr = {PC_reg[`XLEN-1:2], 2'b0};
endmodule
