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
    output [1:0] req_debug,
    output [1:0] gnt_debug,
    output [`XLEN-1:0] PC_reg_debug,
    output waiting_on_inst_debug,
    output [`INST_INDEX_SIZE-1:0] inst_buffer_tail_debug
);

    logic [`XLEN-1:0] PC_reg; // PC we are currently fetching
    logic [`XLEN-1:0] n_PC_reg; // PC we are currently fetching 
    logic received_certain_branch;
    logic n_received_certain_branch;
    logic [`XLEN-1:0] certain_branch_pc_temp;
    logic [`XLEN-1:0] n_certain_branch_pc_temp;
    
    logic waiting_on_inst;
    logic n_waiting_on_inst;  
    assign waiting_on_inst_debug = waiting_on_inst;
    
    logic [1:0] req;
    assign req = {received_certain_branch,branch_pred_req, 1'b1};
    logic [1:0] gnt; 

    IF_ID_PACKET n_if_packet;

    // DEBUG SIGNALS
    assign req_debug = req;
    assign gnt_debug = gnt;
    assign PC_reg_debug = PC_reg;
    
    psel_gen #(.WIDTH(3), .REQS(1)) 
    if_psel (
        .req(req),       
        .gnt(gnt),       
        .gnt_bus(), 
        .empty()  
    );

    always_comb begin

        if (certain_branch_req) begin 
            n_received_certain_branch = 1;
            n_certain_branch_pc_temp = certain_branch_pc;
        end

        if (if_valid) begin       
            if (!waiting_on_inst) begin 
                unique case (gnt)
                    3'b100 : n_PC_reg = certain_branch_pc_temp;
                    3'b010 : n_PC_reg = branch_pred_pc;
                    3'b001 : n_PC_reg = PC_reg + 4;
                    default : n_PC_reg = 32'hdeadbeef;
                endcase
                if (!received_certain_branch) begin
                    n_received_certain_branch = 0;
                end

                n_waiting_on_inst = 1;

                n_if_packet.inst = `NOP;
                n_if_packet.PC = PC_reg;
                n_if_packet.NPC = PC_reg + 4;
                n_if_packet.valid = 0;
            end else if (waiting_on_inst) begin 
                n_PC_reg = PC_reg;
                if (!received_certain_branch) begin
                    n_received_certain_branch = received_certain_branch;
                end 

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
            if (!received_certain_branch) begin 
                n_received_certain_branch = received_certain_branch;
            end

            n_waiting_on_inst = 1;
            if (certain_branch_req) begin 
                n_PC_reg = certain_branch_pc;
            end else begin 
                n_PC_reg = PC_reg;
            end

            n_if_packet.inst = `NOP;
            n_if_packet.PC = PC_reg;
            n_if_packet.NPC = PC_reg+4;
            n_if_packet.valid = 0;
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
            received_certain_branch <= n_received_certain_branch;
            certain_branch_pc_temp <= n_certain_branch_pc_temp;
            `ifdef DEBUG_PRINT
            if (if_packet.valid) begin
                $display("[IF] fetched valid instruction, PC %p", if_packet.PC);
            end else begin 
                $display("[IF] no valid instruction fetched");
            end
            `endif

        end
    end

    // address of the instruction we're fetching (64 bit memory lines)
    // mem always gives us 8=2^3 bytes, so ignore the last 3 bits
    assign proc2Icache_addr = {PC_reg[`XLEN-1:3], 3'b0};
endmodule
