
`include "verilog/sys_defs.svh"
`include "test/mem.sv"
`include "verilog/icache.sv"

module testbench;
    logic clock, reset;

    // IF MODULE 

    logic if_valid; 

    IF_ID_PACKET if_packet;

    logic [`XLEN-1:0] certain_branch_pc;  // target pc: use if take_branch is TRUE
    logic certain_branch_req;    // taken-branch signal ONLY FROM EX

    logic [`XLEN-1:0] rob_target_pc;
    logic rob_target_req;

    logic [`XLEN-1:0] branch_pred_pc;
    logic branch_pred_req;
    
    logic [`XLEN-1:0] proc2Icache_addr;

    logic [63:0]      Icache2proc_data; // data coming back from Instruction memory
    logic Icache2proc_data_valid;

    logic [3:0] req_debug; 
    logic [3:0] gnt_debug; 
    logic [`XLEN-1:0] PC_reg_debug;

    // MEMORY MODULES

    logic [`XLEN-1:0] proc2mem_addr; // address for current command // support for memory model with byte level addressing
    logic [63:0] proc2mem_data; // address for current command
    logic[1:0] proc2mem_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE

    logic [3:0]  mem2proc_response; // 0 = can't accept, other=tag of transaction
    logic [63:0] mem2proc_data;     // data resulting from a load
    logic [3:0]  mem2proc_tag;       // 0 = no value, other=tag of transaction


    ifetch dut(
        .clock(clock),
        .reset(reset),
        
        .if_valid(if_valid),
        
        .certain_branch_pc(certain_branch_pc),
        .certain_branch_req(certain_branch_req),

        .rob_target_pc(rob_target_pc),
        .rob_target_req(rob_target_req),

        .branch_pred_pc(branch_pred_pc),
        .branch_pred_req(branch_pred_req),

        .Icache2proc_data(Icache2proc_data),
        .Icache2proc_data_valid(Icache2proc_data_valid),

        .if_packet(if_packet),
        .proc2Icache_addr(proc2Icache_addr),
        
        .req_debug(req_debug),
        .gnt_debug(gnt_debug),
        .PC_reg_debug(PC_reg_debug)
    );

    mem mem_0 (
        .clk(clock),

        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data),
        // .proc2mem_size(proc2mem_size),
        .proc2mem_command(proc2mem_command),
        
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag)
    );

    icache icache_0 (
        .clock(clock),
        .reset(reset),

        .Imem2proc_response(mem2proc_response),
        .Imem2proc_data(mem2proc_data),
        .Imem2proc_tag(mem2proc_tag),

        .proc2Icache_addr(proc2Icache_addr),

        .proc2Imem_command(proc2mem_command),
        .proc2Imem_addr(proc2mem_addr),

        .Icache_data_out(Icache2proc_data),
        .Icache_valid_out(Icache2proc_data_valid)
    );

    always begin       
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin  
        // $monitor(
        //     "certain_branch_pc = %h, certain_branch_req = %b,\
        //     rob_target_pc = %h, rob_target_req = %b, \
        //     branch_pred_pc = %h,\
        //     branch_pred_req = %b, Icache2proc_data = %h,\
        //     Icache2proc_data_valid = %b, PC_reg_debug = %h", 
        //     certain_branch_pc, certain_branch_req,
        //     rob_target_pc, rob_target_req,
        //     branch_pred_pc,
        //     branch_pred_req, 
        //     Icache2proc_data,
        //     Icache2proc_data_valid, PC_reg_debug);

        clock = 0;
        reset = 0;
        if_valid = 0;

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;
        
        
        certain_branch_req = 0;
        rob_target_req = 0;
        branch_pred_req = 0;
        
        for (int i = 0; i < 5; i++) begin
            @(posedge if_packet.valid)
            $display("if_packet.valid = %b", if_packet.valid);
            $display("if_packet.PC = %h", if_packet.PC);
            $display("desired_PC =%h", 4 * (i + 1));
            assert(if_packet.PC == 4 * (i + 1)) else exit_on_error;
            assert(if_packet.valid == 1) else exit_on_error;
        end

        if_valid = 1;
        for (int i = 4; i < 10; i++) begin
            @(posedge if_packet.valid)
            $display("if_packet.valid = %b", if_packet.valid);
            $display("if_packet.PC = %h", if_packet.PC);
            $display("desired_PC =%h", 4 * (i + 1));
            assert(if_packet.PC == 4 * (i + 1)) else exit_on_error;
            assert(if_packet.valid == 1) else exit_on_error;
        end

        $display("@@@Passed");
        $finish;
    end 
endmodule 
