
`include "verilog/sys_defs.svh"
`include "test/mem.sv"

module testbench;
    logic clock, reset;

    logic if_valid; 

    IF_ID_PACKET if_packet;

    logic [`XLEN-1:0] certain_branch_pc;  // target pc: use if take_branch is TRUE
    logic certain_branch_req;    // taken-branch signal ONLY FROM EX

    logic [`XLEN-1:0] rob_target_pc;
    logic rob_target_req;
    logic rob_stall;

    logic [`XLEN-1:0] branch_pred_pc;
    logic branch_pred_req;
    
    logic [`XLEN-1:0] proc2Icache_addr;

    // FROM ICACHE
    logic [63:0]      Icache2proc_data; // data coming back from Instruction memory
    logic Icache2proc_data_valid;

    logic [3:0] req_debug; 
    logic [3:0] gnt_debug; 

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
        .proc2Icache_addr(proc2Icache_addr),
        .req_debug(req_debug),
        .gnt_debug(gnt_debug)
    );

    logic clk; // Memory clock

    logic [`XLEN-1:0] proc2mem_addr; // address for current command // support for memory model with byte level addressing
    logic [63:0] proc2mem_data; // address for current command
    // MEM_SIZE proc2mem_size; // BYTE, HALF, WORD or DOUBLE
    logic[1:0] proc2mem_command; // `BUS_NONE `BUS_LOAD or `BUS_STORE

    logic [3:0]  mem2proc_response; // 0 = can't accept, other=tag of transaction
    logic [63:0] mem2proc_data;     // data resulting from a load
    logic [3:0]  mem2proc_tag;       // 0 = no value, other=tag of transaction

    mem mem_0 (
        .clk(clk),

        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(proc2mem_data),
        // .proc2mem_size(proc2mem_size),
        .proc2mem_command(proc2mem_command),
        
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag)
    );

    
    // The latency of memory accesses in clock edges 
    integer mem_latency_edges = `MEM_LATENCY_IN_CYCLES * 2;
    // the number of elapsed clock edges
    integer counter_edges = 0; 
    
    always begin       

        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
        counter_edges = counter_edges + 1; 

        if (counter_edges == mem_latency_edges) begin
            clk = ~clk;
            counter_edges = 0; 
        end

    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin  
        $monitor(
            "certain_branch_pc = %h, certain_branch_req = %b,\
            rob_target_pc = %h, rob_target_req = %b, \
            rob_stall = %b, branch_pred_pc = %h,\
            branch_pred_req = %b, Icache2proc_data = %h,\
            Icache2proc_data_valid = %b", 
            certain_branch_pc, certain_branch_req,
            rob_target_pc, rob_target_req,
            rob_stall, branch_pred_pc,
            branch_pred_req, 
            Icache2proc_data,
            Icache2proc_data_valid);

        clock     = 0;
        reset     = 0;
        if_valid = 1;
        rob_stall = 0;

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
            
        @(negedge clock)
        $display("proc2Icache_addr = %h", proc2Icache_addr);  
        @(posedge Icache2proc_data_valid)
        $display("if_packet.inst = %h", if_packet.inst);  
        assert(if_packet.inst == 32'h1111_1110) else exit_on_error;
    end 
endmodule 
