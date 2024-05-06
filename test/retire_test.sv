
`include "verilog/sys_defs.svh"


module testbench;

    CO_RE_PACKET co_package;
    logic regfile_en;
    logic [4:0] regfile_idx;
    logic [`XLEN-1:0] regfile_data;
    logic [3:0]       mem2proc_response;
    logic [$clog2(`ROB_SZ)-1:0] rob_head;

    logic move_head;
    logic [3:0]       pipeline_completed_insts;
    EXCEPTION_CODE    pipeline_error_status;
    logic [4:0]       pipeline_commit_wr_idx;
    logic [`XLEN-1:0] pipeline_commit_wr_data;
    logic             pipeline_commit_wr_en;
    logic [`XLEN-1:0] pipeline_commit_NPC;

    
    retire dut(
       .co_package(co_package),
       .regfile_en(regfile_en),
       .regfile_idx(regfile_idx),
       .regfile_data(regfile_data),
       .mem2proc_response(mem2proc_response),
       .rob_head(rob_head),

       .move_head(move_head),
       .pipeline_completed_insts(pipeline_completed_insts),
       .pipeline_error_status(pipeline_error_status),
       .pipeline_commit_wr_idx(pipeline_commit_wr_idx),
       .pipeline_commit_wr_data(pipeline_commit_wr_data),
       .pipeline_commit_wr_en(pipeline_commit_wr_en),
       .pipeline_commit_NPC(pipeline_commit_NPC)

    );


    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin
        
        co_package.result = 1;
        co_package.NPC = 1;
        co_package.dest_reg_idx = 2;
        co_package.take_branch = 0;
        co_package.halt = 0;
        co_package.illegal = 0;
        co_package.valid = 1;
        co_package.rob_index = 3;



        regfile_en = 1;
        regfile_idx = 4;
        regfile_data = 5;
        mem2proc_response = 1;
        rob_head = 3;


        #10

        assert(move_head == 1) else exit_on_error;
        assert(pipeline_error_status == NO_ERROR) else exit_on_error;
        assert(pipeline_commit_wr_idx == 4) else exit_on_error;
        assert(pipeline_commit_wr_data == 5) else exit_on_error;
        assert(pipeline_commit_wr_en == 1) else exit_on_error;
        assert(pipeline_commit_NPC == 1) else exit_on_error;
            
        $display("@@@Passed");
        $finish;

    end 
endmodule