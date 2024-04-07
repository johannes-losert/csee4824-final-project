module retire(
    input CO_RE_PACKET      co_package, //need to add type
    input logic             regfile_en,  // register write enable
    input logic [4:0]       regfile_idx, // register write index
    input logic [`XLEN-1:0] regfile_data, // register write data 
    input logic [3:0]       mem2proc_response,
    input logic [$clog2(`ROB_SZ)-1:0] rob_head, //head printer in rob

    //signals to tell rob to move head
    output logic move_head,
    //output of the processor
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC
);

    RETIRE_ENTRY [`ROB_SZ-1:0] retire_buffer;


    always_comb begin
        if (co_package.valid) begin
            retire_buffer[co_package.rob_index].valid                 = 1;
            retire_buffer[co_package.rob_index].completed_insts       =  {3'b0, co_package.valid};
            retire_buffer[co_package.rob_index].NPC                   = co_package.NPC;
            retire_buffer[co_package.rob_index].error_status = co_package.illegal        ? ILLEGAL_INST :
                                                                       co_package.halt           ? HALTED_ON_WFI :
                                                                       (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT : NO_ERROR;
            retire_buffer[co_package.rob_index].regfile_en    = regfile_en;
            retire_buffer[co_package.rob_index].regfile_idx   = regfile_idx;
            retire_buffer[co_package.rob_index].regfile_data  = regfile_data;
        end 
        if (retire_buffer[rob_head].valid) begin
            move_head = 1;
            pipeline_completed_insts = retire_buffer[rob_head].completed_insts;
            pipeline_error_status    = retire_buffer[rob_head].error_status;
            pipeline_commit_wr_idx   = retire_buffer[rob_head].regfile_idx;
            pipeline_commit_wr_data  = retire_buffer[rob_head].regfile_data;
            pipeline_commit_wr_en    = retire_buffer[rob_head].regfile_en;
            pipeline_commit_NPC      = retire_buffer[rob_head].NPC;
            retire_buffer[rob_head].valid = 0;
        end else begin
            move_head = 0;
        end
    end


endmodule //retire