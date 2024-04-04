
`include "verilog/sys_defs.svh"


module testbench;

    logic clock, reset;

    logic if_valid; 

    logic [`XLEN-1:0] certain_branch_pc;  // target pc: use if take_branch is TRUE
    logic certain_branch_req;    // taken-branch signal ONLY FROM EX

    logic [`XLEN-1:0] rob_target_pc;
    logic rob_target_req;
    logic rob_stall;

    logic [`XLEN-1:0] branch_pred_pc;
    logic branch_pred_req;
    
    // FROM ICACHE
    logic [63:0]      Icache2proc_data; // data coming back from Instruction memory
    logic Icache2proc_data_valid;

    ifetch dut(
        .clock(clock),
        .reset(reset),
        
        .if_valid(if_valid),
        
        .certain_branch_pc(certain_branch_pc),
        .certain_branch_req(certain_branch_req),
        .branch_pred_pc(branch_pred_pc),
        .branch_pred_req(branch_pred_req),

        .Icache2proc_data(Icache2proc_data),
        .Icache2proc_data_valid(Icache2proc_data_valid),

        .if_packet(if_packet),
        .proc2Icache_addr(proc2Icache_addr)
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
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
        // TODO FIX THE MONITOR
        // $monitor(
        //     $display("if_valid = %b, certain_branch_pc = %h, certain_branch_req = %b, rob_target_pc = %h, rob_target_req = %b, rob_stall = %b, branch_pred_pc = %h, branch_pred_req = %b, Icache2proc_data = %h, Icache2proc_data_valid = %b", 
        //     if_valid, certain_branch_pc, certain_branch_req, rob_target_pc, rob_target_req, rob_stall, branch_pred_pc, branch_pred_req, Icache2proc_data, Icache2proc_data_valid)
        // );

        clock     = 0;
        reset     = 0;
        if_valid = 1;

        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;
        
        // cycle 1
        $display("Starting Certain Branch Test");
        
        certain_branch_pc = 32'h1111_1111;
        certain_branch_req = 1;
        rob_target_pc = 32'h2222_2222;
        rob_target_req = 1;
        branch_pred_pc = 32'h3333_3333;
        branch_pred_req = 1;
        
        assert(proc2Icache_addr == 32'h1111_1111) else exit_on_error;

        // Wait until Icache2proc_data_valid is high
        @(posedge Icache2proc_data_valid);
        $display("Icache2proc_data = %h", Icache2proc_data);
        
        $display("Certain Branch Test Completed");
    end 
endmodule 
