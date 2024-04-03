
`include "verilog/sys_defs.svh"


module testbench;

    logic clock, reset;

    logic if_valid; 

    logic [`XLEN-1:0] certain_branch_pc,  // target pc: use if take_branch is TRUE
    logic             certain_branch_req,    // taken-branch signal ONLY FROM EX

    logic rob_target_pc,
    logic rob_target_req,
    logic rob_stall, 

    logic branch_pred_pc,
    logic branch_pred_req,
    
    // FROM ICACHE
    logic [63:0]      Icache2proc_data, // data coming back from Instruction memory
    logic Icache2proc_data_valid, 

    ifetch dut(
        .clock(clock),
        .reset(reset),
        
        .if_valid(if_valid)
        
        .certain_branch_pc(certain_branch_pc)
        .certain_branch_req(certain_branch_req)

        .branch_pred_pc(branch_pred_pc),
        .branch_pred_req(branch_pred_req),

        .Icache2proc_data(Icache2proc_data),
        .Icache2proc_data_valid(Icache2proc_data_valid),

        .if_packet(if_packet),
        .proc2Icache_addr(proc2Icache_addr)
    );

    // PC+4 Test

    // Certain Branch Test 

    // Priority test



