module retire(
    input logic clock,
    input logic reset,

    input CO_RE_PACKET      co_packet, //need to add type

    // input logic             regfile_en,  // register write enable
    // input logic [4:0]       regfile_idx, // register write index
    // input logic [`XLEN-1:0] regfile_data, // register write data 

   // input logic [3:0]       Dcache2store_response,
    input logic [$clog2(`ROB_SZ)-1:0] rob_head, //head printer in rob
    input logic             clear_retire_buffer, // TODO take branch from previous stage or current?

    //signals to tell rob to move head
    output logic move_head,
    output logic free_store,

    output logic [`REG_IDX_SZ:0] retire_reg_arch_idx,
    output PREG retire_reg_preg,

    //output of the processor
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC,

    /* Output of store to memory (full dword) */
    output logic store_en,
    output logic [`XLEN-1:0] store2Dcache_addr,
    output logic [63:0] store2Dcache_data,


    // output logic [1:0]       proc2Dmem_command, // The memory command
    // output MEM_SIZE          proc2Dmem_size,    // Size of data to read or write
    // output logic [`XLEN-1:0] proc2Dmem_addr,    // Address sent to Data memory
    // output logic [`XLEN-1:0] proc2Dmem_data,     // Data sent to Data memory

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
    incoming_entry.inst = co_packet.inst;
	incoming_entry.function_type = co_packet.function_type;
	incoming_entry.mem_size = co_packet.mem_size;
	incoming_entry.result = co_packet.result;
	incoming_entry.rs2_value = co_packet.rs2_value;
    incoming_entry.completed_insts = {3'b0, co_packet.valid};
    incoming_entry.PC = co_packet.PC;
    incoming_entry.NPC = co_packet.NPC;
    incoming_entry.prev_dword = co_packet.prev_dword;
    // TODO this is definitely wrong, in p3 this was exiting immediately not along with instruction \/
    incoming_entry.error_status = co_packet.illegal ? ILLEGAL_INST :
                                      co_packet.halt    ? HALTED_ON_WFI :
                                      /*(Dcache2store_response==4'h0) ? LOAD_ACCESS_FAULT : */
                                      NO_ERROR;
        incoming_entry.regfile_en = co_packet.regfile_en;
        incoming_entry.regfile_idx = co_packet.regfile_idx;
        incoming_entry.arch_dest_reg_num = co_packet.arch_dest_reg_num; 
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

    assign retire_reg_arch_idx = outgoing_entry.arch_dest_reg_num;
    assign retire_reg_preg.reg_num = outgoing_entry.regfile_idx;
    assign retire_reg_preg.ready = 1;

    /* Signal dcache if we need to do a store */
    assign store_en = outgoing_entry.function_type == STORE && outgoing_entry.valid && ~reset && ~clear_retire_buffer;
    assign free_store = store_en;

    /* Calculate dword to store based on mem size, previous word, and new value */
    // TODO should probably do this in dcache, and pass full address, previous, and mem_size
    // OR could have dcache itself remember prev_dword?
    EXAMPLE_CACHE_BLOCK c;
    always_comb begin 
        c.byte_level = outgoing_entry.prev_dword;
        c.half_level = outgoing_entry.prev_dword;
        c.word_level = outgoing_entry.prev_dword;
        if (store_en) begin 
            case (outgoing_entry.mem_size)
                BYTE: begin
                    c.byte_level[outgoing_entry.result[2:0]] = outgoing_entry.rs2_value[7:0];
                    store2Dcache_data = c.byte_level;
                end
                HALF: begin
                    assert(outgoing_entry.result[0] == 0);
                    c.half_level[outgoing_entry.result[2:1]] = outgoing_entry.rs2_value[15:0];
                    store2Dcache_data = c.half_level;
                end
                WORD: begin
                    assert(outgoing_entry.result[1:0] == 0);
                    c.word_level[outgoing_entry.result[2]] = outgoing_entry.rs2_value[31:0];
                    store2Dcache_data = c.word_level;
                end
                default: begin
                    assert(outgoing_entry.result[1:0] == 0);
                    c.byte_level[outgoing_entry.result[2]] = outgoing_entry.rs2_value[31:0];
                    store2Dcache_data = c.word_level;
                end
            endcase
        end else begin 
            store2Dcache_data = 64'hfacefeed;
        end
    end

    assign store2Dcache_addr = {outgoing_entry.result[31:3],3'b0};


    // ---- 


    // assign proc2Dmem_data = outgoing_entry.rs2_value;
    // assign proc2Dmem_addr = outgoing_entry.result;
    // assign proc2Dmem_size = outgoing_entry.mem_size;

    // always_comb begin
	// if (outgoing_entry.function_type == STORE && outgoing_entry.valid && ~reset && ~clear_retire_buffer) begin
	//     proc2Dmem_command = BUS_STORE;
	//     free_store = 1;
	// end else begin 
	//     proc2Dmem_command = BUS_NONE;
	//     free_store = 0;
	// end
    // end

// typedef struct packed {
//     INST              inst;
//     logic [3:0]       completed_insts;
//     logic [`XLEN-1:0] PC;
//     logic [`XLEN-1:0] NPC;
//     EXCEPTION_CODE    error_status;
//     logic             regfile_en;   // register write enable
//     logic [`PHYS_REG_IDX_SZ:0]  regfile_idx;  // register write index
//     logic [`XLEN-1:0] regfile_data; // register write data 
//     logic             valid;
//     FUNIT 	      function_type;
//     MEM_SIZE          mem_size;
//     logic [`XLEN-1:0] rs2_value;
//     logic [`XLEN-1:0] result;
// } RETIRE_ENTRY;


    function void print_retire_entry(int i, RETIRE_ENTRY ret_entry);
        $write(" %0d \t|", i);
        print_inst(ret_entry.inst , ret_entry.PC, ret_entry.valid);
        $write("\t| ");
        // Result value
        $write(" %0h \t|", ret_entry.result);
        // error status 
        $write(" %0h \t\t|", ret_entry.error_status);
    endfunction


    function void print_retire_buffer();
            $display("RETIRE BUFFER");
            $write("num \t| inst \t\t| result \t| error status\t|");
            $write("num \t| inst \t\t| result \t| error status\t|");
            $write("num \t| inst \t\t| result \t| error status\t|");
            $display("num \t| inst \t\t| result \t| error status\t|");
            for (int i = 0; i < `ROB_SZ/4; i++) begin
                print_retire_entry(i, retire_buffer[i]);
                print_retire_entry(i+`ROB_SZ/4, retire_buffer[i+`ROB_SZ/4]);
                print_retire_entry(i+`ROB_SZ/2, retire_buffer[i+`ROB_SZ/2]);
                print_retire_entry(i+3*`ROB_SZ/4, retire_buffer[i+3*`ROB_SZ/4]);
                $display("");
            end
        endfunction

    always @(negedge clock) begin 
        `ifdef DEBUG_PRINT
        print_retire_buffer();
        $write("Incoming Entry: ");
        print_retire_entry(-1, incoming_entry);
        $display("");
        $write("Outgoing Entry: ");
        print_retire_entry(-1, outgoing_entry);
        $display("");
        `endif

    end 


    /* Calculate pipeline output */
    always_comb begin
        if (outgoing_entry.valid) begin 
            pipeline_completed_insts = outgoing_entry.completed_insts;
            pipeline_error_status    = outgoing_entry.error_status;
            pipeline_commit_wr_idx   = outgoing_entry.arch_dest_reg_num;
            pipeline_commit_wr_data  = outgoing_entry.regfile_data;
            pipeline_commit_wr_en    = outgoing_entry.regfile_en;
            pipeline_commit_NPC      = outgoing_entry.NPC;
        end else begin 
            pipeline_completed_insts = 4'b0;
            pipeline_error_status    = NO_ERROR;
            pipeline_commit_wr_idx   = 5'b0;
            pipeline_commit_wr_data  = `XLEN'b0;
            pipeline_commit_wr_en    = 1'b0;
            pipeline_commit_NPC      = `XLEN'b0;
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
                `ifdef DEBUG_PRINT
                $display("[RT] forwarding packet, not added to buffer");
                `endif
                assert(retire_buffer[rob_head].valid == 1'b0); // Make sure we are not overwriting? Maybe is OK            
            end else if (incoming_entry.valid) begin 
                // Add completed packet to retire buffer
                assert(retire_buffer[co_packet.rob_index].valid == 1'b0); // Make sure we are not overwriting
                retire_buffer[co_packet.rob_index] <= incoming_entry;
            end

            if (outgoing_entry.valid) begin
                `ifdef DEBUG_PRINT
                $write("[RT] retiring packet: ");
                print_inst(outgoing_entry.inst, outgoing_entry.PC, outgoing_entry.valid);
                $display("");
                `endif
                // Remove from retire buffer
                if (!do_forward) begin 
                    // If we are forwarding, do not remove head from buffer? TODO this is weird b/c move head will still be raised
                    retire_buffer[rob_head].valid <= 0;
                end
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
