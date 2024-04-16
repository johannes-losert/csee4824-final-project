`include "verilog/sys_defs.svh"
`include "verilog/psel_gen.sv"

// 1. EX_Branch 2. ROB_Target 3. Branch_Predictor 4. (PC+4)  = 4

// the size of our FIFO instruction buffer
`define INST_BUF_SIZE 8
`define INST_INDEX_SIZE $clog2(`INST_BUF_SIZE)

module ifetch (
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
    output fetch_available_debug,
    output IF_ID_PACKET inst_buffer_debug [`INST_BUF_SIZE-1:0]
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching
    
    logic fetch_available; // if we are currently fetching an instruction in current cycle
    logic n_fetch_available; // if we will be fetching an instruction next cycle
    assign fetch_available_debug = fetch_available;

    
    IF_ID_PACKET inst_buffer [`INST_BUF_SIZE-1:0];
    IF_ID_PACKET n_inst_buffer [`INST_BUF_SIZE-1:0];
    assign inst_buffer_debug = inst_buffer;

    logic[`INST_INDEX_SIZE-1:0] inst_buffer_tail;
    logic[`INST_INDEX_SIZE-1:0] n_inst_buffer_tail;

    IF_ID_PACKET n_if_packet;
    assign n_if_packet = inst_buffer[0];

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

        // if cache can receive new request and the buffer not full, request the next pc
        if (fetch_available && inst_buffer_tail < `INSN_BUF_SIZE) begin 
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

        // if we get a response from cache and there is room in the buffer, add the instruction to the tail of buffer
        if (Icache2proc_data_valid) begin
            // pushing to the inst buffer
            n_inst_buffer[inst_buffer_tail].inst = PC_reg[2] ? Icache2proc_data[63:32] : Icache2proc_data[31:0];
            n_inst_buffer[inst_buffer_tail].PC = PC_reg;
            n_inst_buffer[inst_buffer_tail].NPC = PC_reg + 4;
            n_inst_buffer[inst_buffer_tail].valid = 1;
            
            // if we output, head and input a new instruction the tail is same
            if (if_valid) begin
                n_inst_buffer_tail = inst_buffer_tail; // buffer does not grow because releasing and fetching at same time
            // if we don't output a new instruction, tail grows
            end else begin 
                n_inst_buffer_tail = inst_buffer_tail + 1;
            end
        end else if (!Icache2proc_data_valid && if_valid) begin
            // if we don't get a response from memory but we output the instruction
            // we need to push a NOP to the tail of buffer

            n_inst_buffer[inst_buffer_tail].inst = `NOP;
            n_inst_buffer[inst_buffer_tail].PC = PC_reg;
            n_inst_buffer[inst_buffer_tail].NPC = PC_reg + 4;
            n_inst_buffer[inst_buffer_tail].valid = 0;
            
            // keep the tail above zero while decrementing 
            if (inst_buffer_tail > 0) begin
                n_inst_buffer_tail = inst_buffer_tail - 1;
            end else begin 
                n_inst_buffer_tail = 0;
            end
        end 

        // if we filled the buffer, we can't fetch anymore
        if (n_inst_buffer_tail == `INST_BUF_SIZE - 1) begin
            n_fetch_available = 0;
        end else begin
            n_fetch_available = 1;
        end

        if (if_valid && (inst_buffer_tail > 0)) begin 
            for (int i = 0; i < inst_buffer_tail; i++) begin
                n_inst_buffer[i] = inst_buffer[i+1];
            end
        end else begin
            n_inst_buffer = inst_buffer;
        end
    end

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            PC_reg <= 0;
            fetch_available <= 1;
            for (int i = 0; i < `INST_BUF_SIZE; i++) begin
                inst_buffer[i].inst <= `NOP;
                inst_buffer[i].PC <= 0;
                inst_buffer[i].NPC <= 0;
                inst_buffer[i].valid <= 0;
            end
            inst_buffer_tail <= 0;
        end else begin
            PC_reg <= n_PC_reg;
            inst_buffer <= n_inst_buffer;
            fetch_available <= n_fetch_available;
            inst_buffer_tail <= n_inst_buffer_tail;
            if_packet <= n_if_packet;
        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};
endmodule




