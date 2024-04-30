module retire(
    input logic clock,
    input logic reset,

    input CO_RE_PACKET      co_packet, //need to add type

    // input logic             regfile_en,  // register write enable
    // input logic [4:0]       regfile_idx, // register write index
    // input logic [`XLEN-1:0] regfile_data, // register write data 

    input logic [3:0]       mem2proc_response,
    input logic [$clog2(`ROB_SZ)-1:0] rob_head, //head printer in rob
    input logic             clear_retire_buffer, // TODO take branch from previous stage or current?

    //signals to tell rob to move head
    output logic move_head,

    //output of the processor
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC,
    output logic [1:0]       proc2Dmem_command, // The memory command
    output MEM_SIZE          proc2Dmem_size,    // Size of data to read or write
    output logic [`XLEN-1:0] proc2Dmem_addr,    // Address sent to Data memory
    output logic [`XLEN-1:0] proc2Dmem_data,     // Data sent to Data memory

    output RETIRE_ENTRY [`ROB_SZ-1:0] retire_buffer,

    output RETIRE_ENTRY incoming_entry, outgoing_entry

);

    // RETIRE_ENTRY [`ROB_SZ-1:0] retire_buffer;

    // RETIRE_ENTRY incoming_entry, outgoing_entry;

    logic do_forward;

    // If incoming packet is valid and also head of ROB, then forward 
    assign do_forward = co_packet.valid && (co_packet.rob_index == rob_head);

    // Populate incoming entry, from the pipeline
    always_comb begin 
        incoming_entry.valid = co_packet.valid;
	incoming_entry.function_type = co_packet.function_type;
	incoming_entry.mem_size = co_packet.mem_size;
	incoming_entry.result = co_packet.result;
	incoming_entry.opb_value = outgoing_entry.opb_value;
        incoming_entry.completed_insts = {3'b0, co_packet.valid};
        incoming_entry.NPC = co_packet.NPC;
        incoming_entry.error_status = co_packet.illegal ? ILLEGAL_INST :
                                      co_packet.halt    ? HALTED_ON_WFI :
                                      (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT :
                                      NO_ERROR;
        incoming_entry.regfile_en = co_packet.regfile_en;
        incoming_entry.regfile_idx = co_packet.regfile_idx;
        incoming_entry.regfile_data = co_packet.regfile_data;
    end 

    // Populate outgoing entry, from the buffer
    always_comb begin
        if (do_forward) begin
            outgoing_entry = incoming_entry;
        end else begin 
            outgoing_entry = retire_buffer[rob_head];
        end 
    end

   // Move head if outgoing entry is valid
    assign move_head = ~reset && ~clear_retire_buffer && outgoing_entry.valid;

    assign proc2Dmem_data = outgoing_entry.opb_value;
    assign proc2Dmem_addr = outgoing_entry.result;
    assign proc2Dmem_size = outgoing_entry.mem_size;

    always_comb begin
	if (outgoing_entry.function_type == STORE && outgoing_entry.valid && ~reset && ~clear_retire_buffer) begin
	    proc2Dmem_command = BUS_STORE;
	end else begin 
	    proc2Dmem_command = BUS_NONE;
	end
    end

    always @(posedge clock) begin
        if (reset || clear_retire_buffer) begin
            for (int i=0; i < `ROB_SZ; i++) begin
                retire_buffer[i].valid = 0;
            end
        end else begin 
            // Adding to retire buffer
            if (do_forward) begin 
                // If forwarding, we do not add to buffer 
                assert(retire_buffer[rob_head].valid == 1'b0); // Make sure we are not overwriting? Maybe is OK            
            end else if (incoming_entry.valid) begin 
                // Add completed packet to retire buffer
                assert(retire_buffer[co_packet.rob_index].valid == 1'b0); // Make sure we are not overwriting
                retire_buffer[co_packet.rob_index] <= incoming_entry;
            end

            if (outgoing_entry.valid) begin
                // Remove from retire buffer
                if (!do_forward) begin 
                    // If we are forwarding, do not remove head from buffer? TODO this is weird b/c move head will still be raised
                    retire_buffer[rob_head].valid <= 0;
                end

                // Update pipeline outputs with outgoing entry
                pipeline_completed_insts <= outgoing_entry.completed_insts;
                pipeline_error_status    <= outgoing_entry.error_status;
                pipeline_commit_wr_idx   <= outgoing_entry.regfile_idx;
                pipeline_commit_wr_data  <= outgoing_entry.regfile_data;
                pipeline_commit_wr_en    <= outgoing_entry.regfile_en;
                pipeline_commit_NPC      <= outgoing_entry.NPC;
            end 
        end  
    end

    // old

    // always_comb begin
    //     if (co_packet.valid) begin
    //         retire_buffer[co_packet.rob_index].valid                 = 1;
    //         retire_buffer[co_packet.rob_index].completed_insts       =  {3'b0, co_packet.valid};
    //         retire_buffer[co_packet.rob_index].NPC                   = co_packet.NPC;
    //         retire_buffer[co_packet.rob_index].error_status = co_packet.illegal        ? ILLEGAL_INST :
    //                                                                    co_packet.halt           ? HALTED_ON_WFI :
    //                                                                    (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT : NO_ERROR;
    //         retire_buffer[co_packet.rob_index].regfile_en    = co_packet.regfile_en;
    //         retire_buffer[co_packet.rob_index].regfile_idx   = co_packet.regfile_idx;
    //         retire_buffer[co_packet.rob_index].regfile_data  = co_packet.regfile_data;
    //     end 
    //     if (retire_buffer[rob_head].valid) begin
    //         move_head = 1;
    //         pipeline_completed_insts = retire_buffer[rob_head].completed_insts;
    //         pipeline_error_status    = retire_buffer[rob_head].error_status;
    //         pipeline_commit_wr_idx   = retire_buffer[rob_head].regfile_idx;
    //         pipeline_commit_wr_data  = retire_buffer[rob_head].regfile_data;
    //         pipeline_commit_wr_en    = retire_buffer[rob_head].regfile_en;
    //         pipeline_commit_NPC      = retire_buffer[rob_head].NPC;
    //         retire_buffer[rob_head].valid = 0;
    //     end else begin
    //         move_head = 0;
    //     end
    // end

    // always_comb begin
    //     if (clear_retire_buffer) begin
    //         for (int i=0; i < `ROB_SZ; i++) begin
    //             retire_buffer[i].valid = 0;
    //         end
    //     end   
    // end

endmodule //retire
