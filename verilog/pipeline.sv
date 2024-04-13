/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  pipeline.sv                                         //
//                                                                     //
//  Description :  Top-level module of the verisimple pipeline;        //
//                 This instantiates and connects the 5 stages of the  //
//                 Verisimple pipeline together.                       //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

module pipeline (
    input        clock,             // System clock
    input        reset,             // System reset
    input [3:0]  mem2proc_response, // Tag from memory about current request
    input [63:0] mem2proc_data,     // Data coming back from memory
    input [3:0]  mem2proc_tag,      // Tag from memory about current reply

    output logic [1:0]       proc2mem_command, // Command sent to memory
    output logic [`XLEN-1:0] proc2mem_addr,    // Address sent to memory
    output logic [63:0]      proc2mem_data,    // Data sent to memory
`ifndef CACHE_MODE // no longer sending size to memory
    output MEM_SIZE          proc2mem_size,    // Data size sent to memory
`endif

    // Note: these are assigned at the very bottom of the module
    output logic [3:0]       pipeline_completed_insts,
    output EXCEPTION_CODE    pipeline_error_status,
    output logic [4:0]       pipeline_commit_wr_idx,
    output logic [`XLEN-1:0] pipeline_commit_wr_data,
    output logic             pipeline_commit_wr_en,
    output logic [`XLEN-1:0] pipeline_commit_NPC,

    // Debug outputs: these signals are solely used for debugging in testbenches
    // Do not change for project 3
    // You should definitely change these for project 4
    // output logic [`XLEN-1:0] if_NPC_dbg,
    // output logic [31:0]      if_inst_dbg,
    // output logic             if_valid_dbg,
    // output logic [`XLEN-1:0] if_id_NPC_dbg,
    // output logic [31:0]      if_id_inst_dbg,
    // output logic             if_id_valid_dbg,
    // output logic [`XLEN-1:0] id_ex_NPC_dbg,
    // output logic [31:0]      id_ex_inst_dbg,
    // output logic             id_ex_valid_dbg,
    // output logic [`XLEN-1:0] ex_mem_NPC_dbg,
    // output logic [31:0]      ex_mem_inst_dbg,
    // output logic             ex_mem_valid_dbg,
    // output logic [`XLEN-1:0] mem_wb_NPC_dbg,
    // output logic [31:0]      mem_wb_inst_dbg,
    // output logic             mem_wb_valid_dbg
);

    //////////////////////////////////////////////////
    //                                              //
    //                Pipeline Wires                //
    //                                              //
    //////////////////////////////////////////////////

    // Pipeline register enables
    logic if_id_enable, id_ex_enable, ex_mem_enable, mem_wb_enable;

    // Outputs from IF-Stage and IF/ID Pipeline Register
    logic [`XLEN-1:0] proc2Imem_addr;
    IF_ID_PACKET if_packet, if_id_reg;

    // Outputs from ID stage and ID/EX Pipeline Register
    ID_EX_PACKET id_packet, id_ex_reg;

    // Outputs from EX-Stage and EX/MEM Pipeline Register
    EX_MEM_PACKET ex_packet, ex_mem_reg;

    // Outputs from MEM-Stage and MEM/WB Pipeline Register
    MEM_WB_PACKET mem_packet, mem_wb_reg;

    // Outputs from MEM-Stage to memory
    logic [`XLEN-1:0] proc2Dmem_addr;
    logic [`XLEN-1:0] proc2Dmem_data;
    logic [1:0]       proc2Dmem_command;
    MEM_SIZE          proc2Dmem_size;

    // Outputs from WB-Stage (These loop back to the register file in ID)
    logic             wb_regfile_en;
    logic [4:0]       wb_regfile_idx;
    logic [`XLEN-1:0] wb_regfile_data;

    //////////////////////////////////////////////////
    //                                              //
    //                  Valid Bit                   //
    //                                              //
    //////////////////////////////////////////////////

    // This state controls the stall signal that artificially forces IF
    // to stall until the previous instruction has completed.
    // For project 3, start by setting this to always be 1

    logic next_if_valid;

    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            // start valid, other stages (ID,EX,MEM,WB) start as invalid
            next_if_valid <= 1;
        end else begin
            // valid bit will cycle through the pipeline and come back from the wb stage
            next_if_valid <= mem_wb_reg.valid;
        end
    end


    //////////////////////////////////////////////////
    //                                              //
    //                Instruction Fetch             //
    //                                              //
    //////////////////////////////////////////////////

    logic [`XLEN-1:0] proc2Icache_addr;
    logic [63:0] Icache_data_out;
    logic Icache_valid_out; 

    icache icache_0 (
        .clock(clock)
        .reset(reset)

        // MEMORY RESPONSE
        .Imem2proc_response(mem2proc_response), // Should be zero unless there is a response
        .Imem2proc_data(mem2proc_data), 
        .Imem2proc_tag(mem2proc_tag),

        // This wire comes from the ifetch module.  
        // The main challenge here 
        .proc2Icache_addr(proc2Icache_addr),
        .proc2Imem_command(proc2mem_command),
        

        // OUTPUTS
        .proc2Imem_addr(proc2Imem_addr), // Cache Fetching Address From Mem

        .Icache_data_out(Icache_data_out), // Data From Mem to Cache to Fetch
        .Icache_valid_out(Icache_valid_out) 
    )   


    ifetch ifetch_0 (
        .clock(clock)
        .reset(reset)
        .if_valid(next_if_valid),       // only go to next PC when true

        
        .branch_target(),  // target pc: use if take_branch is TRUE
    
        .Icache2proc_data(Icache_data_out), // data coming back from Instruction memory
        .Icache2proc_data_valid(Icache_valid_out)
 
        if_packet(if_id_reg),    
        proc2Icache_addr(proc2Icache_addr),  
    )


    //////////////////////////////////////////////////
    //                                              //
    //                Memory Outputs                //
    //                                              //
    //////////////////////////////////////////////////

    // these signals go to and from the processor and memory
    // we give precedence to the mem stage over instruction fetch
    // note that there is no latency in project 3
    // but there will be a 100ns latency in project 4

    always_comb begin
        if (proc2Dmem_command != BUS_NONE) begin // read or write DATA from memory
            proc2mem_command = proc2Dmem_command;
            proc2mem_addr    = proc2Dmem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = proc2Dmem_size;  // size is never DOUBLE in project 3
`endif
        end else begin                          // read an INSTRUCTION from memory
            proc2mem_command = BUS_LOAD;
            proc2mem_addr    = proc2Imem_addr;
`ifndef CACHE_MODE
            proc2mem_size    = DOUBLE;          // instructions load a full memory line (64 bits)
`endif
        end
        proc2mem_data = {32'b0, proc2Dmem_data};
    end

    //////////////////////////////////////////////////
    //                                              //
    //               Pipeline Outputs               //
    //                                              //
    //////////////////////////////////////////////////

    assign pipeline_completed_insts = {3'b0, mem_wb_reg.valid}; // commit one valid instruction
    assign pipeline_error_status = mem_wb_reg.illegal        ? ILLEGAL_INST :
                                   mem_wb_reg.halt           ? HALTED_ON_WFI :
                                   (mem2proc_response==4'h0) ? LOAD_ACCESS_FAULT : NO_ERROR;

    assign pipeline_commit_wr_en   = wb_regfile_en;
    assign pipeline_commit_wr_idx  = wb_regfile_idx;
    assign pipeline_commit_wr_data = wb_regfile_data;
    assign pipeline_commit_NPC     = mem_wb_reg.NPC;

// ABOVE TTHIS LINE IS OLD STUFF 
// 
//
//
//
//
//
// BELOW THIS LINE IS NEW STUFF 

    /* Big integration todos */
    // - figure out how ROB/RS/Free list communicate stalls to ROB
    // - figure out where there are intermediate registers
    // - define each module: reservation station, rob, free list, map table, stage_if, stage_id, stage_is, ??



    //////////////////////////////////////////////////
    //                                              //
    //               CDB Broadcast                  //
    //                                              //
    //////////////////////////////////////////////////

    logic cdb_broadcast_en; // whether or not the cdb is currently broadcasting
    logic [`PHYS_REG_IDX_SZ:0] cdb_phys_idx; // the physical register index to broadcast
    logic [`XLEN-1:0] cdb_data; // the data to write to the register


    //////////////////////////////////////////////////
    //                                              //
    //               Regfile Stuff                  //
    //                                              //
    //////////////////////////////////////////////////

    /* regifle signals */

    logic rf_read_idx_1;
    logic rf_read_idx_2;

    logic [`XLEN-1:0] rf_read_data_1;
    logic [`XLEN-1:0] rf_read_data_2;

    logic rf_write_en;
    logic [`PHYS_REG_IDX_SZ:0] rf_write_idx;
    logic [`XLEN-1:0] rf_write_data;

    // Writes come from the CDB

    assign rf_write_en = cdb_broadcast_en;
    assign rf_write_idx = cdb_phys_idx;
    assign rf_write_data = cdb_data;


    // Instantiate the register file
    regfile regfile_0 (
        .clock      (clock),
        .read_idx_1 (rf_read_idx_1),
        .read_idx_2 (rf_read_idx_2),

        .write_idx  (regfile_idx),
        .write_en   (regfile_en),
        .write_data (regfile_data),

        .read_out_1 (rf_read_data_1),
        .read_out_2 (rf_read_data_2)
    );

    // Read from head operation
    logic fl_dequeue_en;
    logic [`PHYS_REG_IDX_SZ:0] fl_head_pr;
    logic fl_is_empty;

    // write to tail operation
    logic fl_enqueue_en;
    logic [`PHYS_REG_IDX_SZ:0] fl_enqueue_pr;
    logic fl_is_full;

    // debug 
    logic fl_was_dequeued;
    logic [`PHYS_REG_IDX_SZ:0] fl_dequeue_pr;
    logic fl_was_enqueued;
    logic [`PHYS_REG_IDX_SZ+1:0] fl_back_tail_ptr, fl_front_head_ptr;
    logic [`PHYS_REG_IDX_SZ:0] fl_free_list[`FREE_LIST_SIZE];
 
    free_list free_list_0 (
        .clk(clock),
        .reset(reset),

        /* READ from head operation */

        // inputs 
        .dequeue_en(fl_dequeue_en), // If raised, will actually deuqueue from the head
        
        // outputs
        .front_head_pr(fl_head_pr), // Always points to the next element to be dequeued, unless is_empty
        .is_empty(fl_is_empty), // IF is_empty, front_head_pr is undefined

        // debug outputs
        .was_dequeued(fl_was_dequeued),
        .dequeue_pr(fl_dequeue_pr),


        /* WRITE to tail operation */
        // inputs
        .enqueue_en(fl_enqueue_en), // If raised, will enqueue fl_enqueue_pr at the tail
        .enqueue_pr(fl_enqueue_pr), // PR to enqueue if fl_enqueue_en raised
        
        // outputs
        .is_full(fl_is_full), // raised if free list is currently full (TODO not sure why this would happen, currently max is greater than num PRs)

        // debug outputs
        .was_enqueued(fl_was_enqueued),

        /* misc debug outputs */
        .back_tail_ptr(fl_back_tail_ptr),
        .front_head_ptr(fl_front_head_ptr),
        .free_list(fl_free_list)
    );


    // GET operation 1 (opa)
    logic [`REG_IDX_SZ:0] mt_arch_reg1_idx;
    PREG mt_preg1_out;

    // GET operation 2 (opb)
    logic [`REG_IDX_SZ:0] mt_arch_reg2_idx;
    PREG mt_preg2_out;

    // GET/SET operation (get old dest, replace with new dest)
    logic mt_set_dest_enable; /* Enable to overwrite dest with new pr */
    logic [`REG_IDX_SZ:0] mt_arch_dest_idx;
    logic [`PHYS_REG_IDX_SZ:0] mt_new_dest_pr;

    PREG mt_old_dest_pr;

    // retire operation 
    logic mt_retire_enable;
    logic [`REG_IDX_SZ:0] mt_retire_arch_reg;

    // restore operation
    logic mt_restore_enable;

    // set ready operation (CDB)
    logic mt_set_ready_enable;
    logic [`PHYS_REG_IDX_SZ:0] mt_ready_phys_idx;


    map_table map_table_0 (
        .clk(clk),
        .reset(reset),
        
        /* GET operation 1 (opa) */
        // input
        .arch_reg1_idx(mt_arch_reg1_idx),
        //output
        .preg1_out(mt_preg1_out),

        /* GET operation 2 (opb) */
        // input
        .arch_reg2_idx(mt_arch_reg2_idx),
        // output
        .preg2_out(mt_preg2_out),

        /* GET/SET operation (get old dest, replace with new dest) */
        // inputs
        .arch_dest_idx(mt_arch_dest_idx),
        .set_dest_enable(mt_set_dest_enable),
        .new_dest_pr(mt_new_dest_pr),
        // output
        .old_dest_pr(mt_old_dest_pr),

        /* Retire operation */
        // inputs
        .retire_arch_reg(mt_retire_arch_reg),
        .retire_enable(mt_retire_enable),stall

        /* Restore operation */
        // inputs
        .restore_enable(mt_restore_enable),

        /* SET READY operation (CDB) */
        // inputs
        .set_ready_enable(mt_set_ready_enable),
        .ready_phys_idx(mt_ready_phys_idx)
    );

    //////////////////////////////////////////////////
    //                                              //
    //               Instruction Fetch (F)           //
    //                                              //
    //////////////////////////////////////////////////


    /* SIGNALS */
    logic next_if_valid; // stall signal for IF stage
    
    logic certain_branch_req, rob_target_req, branch_pred_req; // TODO what are these?
    logic [63:0] certain_branch_pc, rob_target_pc, branch_pred_pc; // TODO program counters outputed at different stages, for branch prediction

    // data output from icache/memory TODO move closer to memory definition
    logic [63:0] Icache2proc_data;  // data coming back from Instruction memory
    logic Icache2proc_data_valid; // data coming back from Instruction memory
    
    logic [`XLEN-1:0] proc2Icache_addr; // address to fetch from memory

    IF_ID_PACKET if_packet; // instruction packet to pass to decode

    //DEBUG
    logic [3:0] req_debug, gnt_debug;
    logic [`XLEN-1:0] PC_reg_debug;
    logic fetch_available_debug;
    IF_ID_PACKET inst_buffer_debug [`INSN_BUF_SIZE-1:0];



    // TODO Instruction fetch stall logic 
    always_ff @(posedge clock) begin
        next_if_valid <= ~dispatch_stall;
        if (/*something*/) begin
            next_if_valid <= 1'b0;
        end
    end



    icache icache_0 (
        .clock(clk),
        .reset(reset),
        
        // From memory (input to pipeline module)
        .Imem2proc_response(Imem2proc_response), // Should be zero unless there is a response
        .Imem2proc_data(Imem2proc_data),
        .Imem2proc_tag(Imem2proc_tag),

        // From fetch stage
        .proc2Icache_addr(proc2Icache_addr),

        // To memory
        .proc2Imem_command(proc2Imem_command),
        .proc2Imem_addr(proc2Imem_addr),

        // To fetch stage
        .Icache_data_out(Icache_data_out), // Data is mem[proc2Icache_addr]
        .Icache_valid_out(Icache_valid_out) // When valid is high
        
    );

    ifetch ifetch_0 (
        .clock(clk),
        .reset(reset),

        //          ***INPUTS***
        .if_valid(next_if_valid),       // only go to next PC when true

        // Program Counter Inputs
        .certain_branch_pc(certain_branch_pc),
        .certain_branch_req(certain_branch_req),

        .rob_target_pc(rob_target_pc),
        .rob_target_req(rob_target_req),

        .branch_pred_pc(branch_pred_pc),
        .branch_pred_req(branch_pred_req),

        // Icache Request Response
        .Icache2proc_data(Icache2proc_data), // data coming back from Instruction memory
        .Icache2proc_data_valid(Icache2proc_data), // data coming back from Instruction memory
        
        //         *** OUTPUTS ***
        .proc2Icache_addr(proc2Icache_addr),
        .if_packet(if_packet), // to decode 


        //          *** DEBUG ***
        .req_debug(req_debug),
        .gnt_debug(gnt_debug),
        .PC_reg_debug(PC_reg_debug),
        .fetch_available_debug(fetch_available_debug),
        .inst_buffer_debug(inst_buffer_debug)
    
    );

    //////////////////////////////////////////////////
    //                                              //
    //            IF/ID Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////

    // register takes output from ifetch and gives input to decode
    // if if_id_enable is not 1, just retains its old value (for stalling)

        IF_ID_PACKET if_id_reg;

        assign if_id_enable = 1'b1; // always enabled (TODO change for stall logic?)
        // synopsys sync_set_reset "reset"
        always_ff @(posedge clock) begin
            if (reset) begin // TODO make sure these are correct
                if_id_reg.inst  <= `NOP;
                if_id_reg.valid <= `FALSE;
                if_id_reg.NPC   <= 0;
                if_id_reg.PC    <= 0;
            end else if (if_id_enable) begin
                if_id_reg <= if_packet;
            end
        end

        // debug outputs
        assign if_id_NPC_dbg   = if_id_reg.NPC;
        assign if_id_inst_dbg  = if_id_reg.inst;
        assign if_id_valid_dbg = if_id_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //               Instruction Dispatch (D)       //
    //                                              //
    //////////////////////////////////////////////////

    // TODO figure out whether this all happens in a single clock cycle 

    /* 1. Take instruction in if_id register and decode it
        - Generate architectural opa, opb, dest, and valid bits for each
       2. If decoded instruction destination is valid, dqueue a PR from the free list
        - If free list is empty, stall fetching
       3. Try to allocate new ROB entry 
        a. Provide instruction (from fetch), free_reg (from free list), and which RS entries are free 
        b. If ROB full, then stall fetching
        c. If ANY single ROB entry ready to be dispatched has an FU type that has a free RS entry,
        then issue the instruction in that ROB entry to the RS (and from the RS to the map table)
       4. Try to allocate new RS entry
        a. If ROB issues an instruction to RS, then allocate an RS entry (we better have a slot for it)
            - Get operands from Map Table, update destination in map table
        b. If ROB does not issue (stall?) then do not allocate RS entry
    */
    logic rob_stall;
    logic rob_full;
    logic head;
    logic [$clog2(`ROB_SZ)-1:0] inst_index;// need to pass to retire stage
    logic dispatch_stall;
    logic alu_entries_full;
    logic mult_entries_full;
    logic load_entries_full;
    logic store_entries_full;
    logic branch_entries_full;
    ID_IS_PACKET id_packet;
    


    //define signals for reorder buffer
    logic move_head;
    logic undo_rob;
    logic [$clog2(`ROB_SZ)-1:0] undo_index_rob;
    // logic rob_stall;
    logic update_free_list;
    PREG free_index;
    logic update_map_table;
    logic [`PHYS_REG_SZ-1:0] mt_update_tag;
    logic [4:0] mt_target_reg;
    logic update_arch_map;
    logic [`REG_IDX_SZ:0] update_arch_told;
    logic [`PHYS_REG_SZ-1:0] update_phys_told;
    logic [`REG_IDX_SZ:0] arch_told;
    logic [`PHYS_REG_SZ-1:0] phys_told;
    // logic rob_full;
    // logic head;


    //define signals for reservation station
    RS_PACKET rs_input_packet;
    logic alloc_done;
    logic issue_enable;
    logic issue_ready;
    PREG  rs_ready_reg;
    
    //TODO: assign these to functional units' outputs
    logic free_alu;
    logic free_mult;
    logic free_load;
    logic free_store;
    logic free_branch;


    RS_PACKET rs_issued_packet;
    logic [`MAX_FU_INDEX-1:0] issue_fu_index;
    
    logic [`PHYS_REG_IDX_SZ:0] rob_free_reg_pr;

    assign rob_free_reg_pr = (fl_is_empty) ? `ZERO_REG : fl_head_pr;
    // TODO add reset?
    // TODO: need to find the phys reg
    dispatch dispatch_0 (
        .if_id_packet(if_id_packet),
        .rob_stall(rob_stall), 
        .rob_full(rob_full), 
        .alu_entries_full(alu_entries_full),       
        .mult_entries_full(mult_entries_full),
        .load_entries_full(load_entries_full),
        .store_entries_full(store_entries_full),
        .branch_entries_full(ranch_entries_full),

        //dispatch's outputs
        .id_packet(id_packet),
        .stall(dispatch_stall)                
    );

    // TODO fixup inputs and outputs
    reorder_buffer reorder_buffer_0(
        .clock(clock),
        .reset(reset),
        .inst(id_packet.inst),
        .write(id_packet.valid),
        .move_head(move_head),
        .free_reg(rob_free_reg_pr),
        .undo(undo_rob),
        .undo_index(undo_index_rob),


        // rob's outputs
        .stall(rob_stall),

        //TODO: need to connect the actual interconnects
        .used_free_reg(fl_dequeue_en),
        .update_free_list(update_free_list),
        .free_index(free_index),
        .update_map_table(update_map_table),
        .mt_update_tag(mt_update_tag),
        .mt_target_reg(mt_target_reg),
        .update_arch_map(update_arch_map),
        .update_arch_told(update_arch_told),
        .update_phys_told(update_phys_told),
        .arch_told(arch_told),
        .phys_told(phys_told),
        
        // following wires are defined
        .inst_index(inst_index),
        .full(rob_full),
        .head(head)
    );
    
    assign rs_input_packet.funit = id_packet.function_type;
    assign rs_input_packet.inst = id_packet.inst;
    assign rs_input_packet.dest_reg = id_packet.dest_reg_idx;
    assign rs_input_packet.src1_reg = id_packet.opa_select;
    assign rs_input_packet.src2_reg = id_packet.opb_select;
    assign issue_enable = 1;
    assign rs_ready_reg.reg_num = cdb_phys_idx;
    assign rs_ready_reg.ready = 1;
    
    //Yet to be connected to complete stage
    logic update_from_complete;
    PREG ready_reg_from_complete;

    reservation_station reservation_station_0(
        .clock(clock),
        .reset(reset),
        
        /* Allocate */
        .allocate(id_packet.valid),
        .input_packet(rs_input_packet),

        .done(alloc_done),

        /* Update 
           TODO: need to define based on CDB*/
        .update(cdb_broadcast_en),
        .ready_reg(rs_ready_reg),

        /* Issue */
        .issue_enable(issue_enable),
        .ready(issue_ready),
        .issued_packet(rs_issued_packet),
        .issue_fu_index(issue_fu_index),

        // if func free
        .alu_entries_full(alu_entries_full),
        .mult_entries_full(mult_entries_full),
        .load_entries_full(load_entries_full),
        .store_entries_full(store_entries_full),
        .branch_entries_full(branch_entries_full),

        /* Free */
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load),
        .free_store(free_store),
        .free_branch(free_branch)
 
    );

    //////////////////////////////////////////////////
    //                                              //
    //            ID/IS Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////
    // TODO this probably needs a lot more thought

    ID_IS_PACKET id_is_reg;


    // TODO maybe use this for stalling?
    assign id_is_enable = 1'b1; // always enabled?
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin   
            // TODO this is still p3 id_ex packet, replace with id_is
            id_is_reg <= `INVALID_ID_IS_PACKET;
        end else if (id_is_enable) begin
            id_is_reg <= id_packet;
        end
    end

    // debug outputs TODO add more
    assign id_is_NPC_dbg   = id_is_reg.NPC;
    assign id_is_inst_dbg  = id_is_reg.inst;
    assign id_is_function_type = id_is_reg.function_type;
    assign id_is_valid_dbg = id_is_reg.valid;
 


    //////////////////////////////////////////////////
    //                                              //
    //               Issue (S)                      //
    //                                              //
    //////////////////////////////////////////////////


    // Waiting in an RS entry, and passing to FUs once both operands ready 
    /* 
        1. If reservation station is ready to issue an instruction, pass it to the FU
            - if more than one FU is ready, choose one somehow (done in reservation station)
            - Should be storing FUs indexed somehow so they can be passed directly (for loop?)
            a. Convert physical register number for each operand to its value by reading regfile
    */  

    /* SIGNALS */

    IS_EX_PACKET is_packet;
    RS_PACKET issued_packet;

    assign issued_packet = is_packet.rs_packet; // TODO add this component to is_ex_packet and integrate into stage_issue

    issue stage_issue (
        // Inputs
        .rs_packet(issued_packet), // TODO put issued packet in id_is_reg?
        .id_is_reg(id_is_reg),

        // Outputs
        .is_packet(is_packet)
    );



    //////////////////////////////////////////////////
    //                                              //
    //            IS/EX Pipeline Register           //
    //                                              //
    //////////////////////////////////////////////////
    // TODO this probably needs a lot more thought

    IS_EX_PACKET is_ex_reg;

    assign is_ex_enable = 1'b1; // always enabled?
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin   
            // TODO this is still p3 id_ex packet, replace with id_is
            is_ex_reg <= '{
                `NOP, // we can't simply assign 0 because NOP is non-zero
                {`XLEN{1'b0}}, // PC
                {`XLEN{1'b0}}, // NPC
                {`XLEN{1'b0}}, // rs1 select
                {`XLEN{1'b0}}, // rs2 select
                OPA_IS_RS1,
                OPB_IS_RS2,
                `ZERO_REG,
                ALU_ADD,
                1'b0, // rd_mem
                1'b0, // wr_mem
                1'b0, // cond
                1'b0, // uncond
                1'b0, // halt
                1'b0, // illegal
                1'b0, // csr_op
                1'b0  // valid
            };
        end else if (id_is_enable) begin
            is_ex_reg <= is_packet;
        end
    end

    // debug outputs
    //assign id_ex_NPC_dbg   = id_ex_reg.NPC;
    //assign id_ex_inst_dbg  = id_ex_reg.inst;
    //assign id_ex_valid_dbg = id_ex_reg.valid;


    //////////////////////////////////////////////////
    //                                              //
    //               Execute (E)                    //
    //                                              //
    //////////////////////////////////////////////////
    
    EX_CO_PACKET ex_packet;

    // Perform each operation 
    stage_ex stage_ex0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .is_ex_reg(is_ex_reg),
        .issue_fu_index(issue_fu_index), // TODO add this to packet at earlier stage of the pipeline

        // Outputs
        .ex_packet(ex_packet),
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load),
        .free_store(free_store),
        .free_branch(free_branch),

        // debug outputs
        .tmp_alu_packet(tmp_alu_packet),
        .tmp_mult_packet(tmp_mult_packet),
        .tmp_branch_packet(tmp_branch_packet)
    );



    //////////////////////////////////////////////////
    //                                              //
    //           EX/CO Pipeline Register             //
    //                                              //
    //////////////////////////////////////////////////
    // TODO this probably needs a lot more thought
    EX_CO_PACKET ex_co_reg;

    assign ex_co_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            ex_co_inst_dbg <= `NOP; // debug output
            ex_co_reg      <= 0;    // TODO can the defaults all be zero?
        end else if (ex_co_enable) begin
      //      ex_co_inst_dbg <= id_ex_inst_dbg; // debug output, just forwarded from ID
            ex_co_reg      <= ex_packet;
        end
    end

    // debug outputs
  //  assign ex_mem_NPC_dbg   = ex_mem_reg.NPC;
   // assign ex_mem_valid_dbg = ex_mem_reg.valid;

    //////////////////////////////////////////////////
    //                                              //
    //               Complete (C)                   //
    //                                              //
    //////////////////////////////////////////////////    
    
    complete complete_0 (
        // input packet 
        .ex_co_reg(ex_co_reg),

        // output packet 
        .co_package(co_package),

        // CDB output
        .regfile_en(cdb_broadcast_en),
        .regfile_idx(cdb_phys_idx),
        .regfile_data(cdb_data),
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_branch(free_branch),
        .free_load(free_load),
        .free_store(free_store)
    );

    //////////////////////////////////////////////////
    //                                              //
    //           CO/RE Pipeline Register             //
    //                                              //
    //////////////////////////////////////////////////
    // TODO this probably needs a lot more thought
    CO_RE_PACKET co_re_reg;
    logic co_re_enable;
    logic re_regfile_en;
    logic [`PHYS_REG_SZ-1:0] re_regfile_idx;
    logic [`XLEN-1:0] re_regfile_data;

    assign co_re_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            co_re_inst_dbg  <= `NOP; // debug output
            co_re_reg       <= 0;    // TODO can the defaults all be zero?
        end else if (co_re_enable) begin
      //      ex_co_inst_dbg <= id_ex_inst_dbg; // debug output, just forwarded from ID
            co_re_reg       <= ex_packet;
            re_regfile_en   <= cdb_broadcast_en;
            re_regfile_idx  <= cdb_phys_idx;
            re_regfile_data <= cdb_data;
        end
    end
    //////////////////////////////////////////////////
    //                                              //
    //               Retire (R)                     //
    //                                              //
    ////////////////////////////////////////////////// 
    // In program order, remove from ROB, update arch table, and free PR
    // Handle exceptions

    /* 
        On any given cycle, check if head of ROB is complete
        If it is, retire it by removing from ROB, updating Arch Map, and freeing PR
    */

    retire retire_0(
        .co_package(co_package), //need to add type
        .regfile_en(re_regfile_en),  // register write enable
        .regfile_idx(re_regfile_idx), // register write index
        .regfile_data(re_regfile_data), // register write data 
        .mem2proc_response(mem2proc_response), //TODO: need to update when memory operation is implemented
        .rob_head(rob_head), //head printer in rob
        .take_branch(take_branch),
        //signals to tell rob to move head
        .move_head(move_head),
        //output of the processor
        .pipeline_completed_insts(pipeline_completed_insts),
        .pipeline_error_status(pipeline_error_status),
        .pipeline_commit_wr_idx(pipeline_commit_wr_idx),
        .pipeline_commit_wr_data(pipeline_commit_wr_data),
        .pipeline_commit_wr_en(pipeline_commit_wr_en),
        .pipeline_commit_NPC(pipeline_commit_NPC)
);

   

endmodule // pipeline
