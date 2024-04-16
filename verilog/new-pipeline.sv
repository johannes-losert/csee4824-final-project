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
    //                Pipeline Wires                //
    //////////////////////////////////////////////////


    // pipeline register enables
    // TODO difference between these and stall signals?
    logic if_id_en, id_is_en, is_ex_en, ex_co_en, co_rt_en;

    // Outputs from IF stage and input to ID stage
    logic [`XLEN-1:0] if_proc2Icache_addr; 
    IF_ID_PACKET if_packet, if_id_reg;

    // Output s from ID stage and input to IS stage
    ID_IS_PACKET id_packet, id_is_reg;

    // Outputs from IS stage and input to EX stage
    IS_EX_PACKET is_packet, is_ex_reg;

    // Outputs from EX stage and input to CO stage
    EX_CO_PACKET ex_packet, ex_co_reg;

    // Outputs from CO stage and input to RT stage
    CO_RT_PACKET co_packet, co_rt_reg;

    //////////////////////////////////////////////////
    //                Stall Logic                   //
    //////////////////////////////////////////////////

    // Logic that controls stalling at each stage 
    // TODO some of these might not be necessary (probably only need if_stall)
    logic if_stall, id_stall, is_stall, ex_stall, co_stall, rt_stall;


    // TODO implement logic: if stalls if id stage can't accept a new instruction
    assign if_stall = 0;

    // TODO these can prbably all stay zero, id stage 'stalls' are handled by RS signals
    assign id_stall = 0;
    assign is_stall = 0;
    assign ex_stall = 0;
    assign co_stall = 0;
    assign rt_stall = 0;

    //////////////////////////////////////////////////
    //          Instruction Fetch Signals           //
    //////////////////////////////////////////////////
    // TODO move signals to top and replace this section with assigns
    // Icache input signals
    logic [3:0] Imem2proc_response, Imem2proc_tag;
    logic [63:0] Imem2proc_data;
    logic [`XLEN-1:0] proc2Icache_addr; // Address from fetch stage

    // Icache output signals
    logic [1:0] proc2Imem_command;
    logic [`XLEN-1:0] proc2Imem_addr;

    logic [63:0] Icache_data_out;
    logic Icache_valid_out;

    // Ifetch input signals
    logic if_valid;
    logic [`XLEN-1:0] certain_branch_pc, rob_target_pc, branch_pred_pc;
    logic certain_branch_req, rob_target_req, branch_pred_req;

    logic [63:0] Icache2porc_data;
    logic Icache2proc_data_valid;

    // Ifetch output signals
    logic [`XLEN-1:0] if_proc2Icache_addr;

    // TODO debug signals?

    //////////////////////////////////////////////////
    //          Instruction Fetch Modules           //
    //////////////////////////////////////////////////

    assign proc2Icache_addr = if_proc2Icache_addr;

    icache icache_0 (
        .clock(clock),
        .reset(reset),

        // From memory
        .Imem2proc_response(mem2proc_response),
        .Imem2proc_data(mem2proc_data),
        .Imem2proc_tag(mem2proc_tag),

        // From fetch stage
        .proc2Icache_addr(proc2Icache_addr),

        // To memory
        .proc2Imem_command(proc2Imem_command),
        .proc2Imem_addr(proc2Imem_addr),

        // To fetch stage
        .Icache_data_out(Icache_data_out),
        .Icache_valid_out(Icache_valid_out)
    );

    assign Icache2proc_data = Icache_data_out;
    assign Icache2proc_data_valid = Icache_valid_out;

    ifetch ifetch_0 (
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

        // Outputs
        .proc2Icache_addr(proc2Icache_addr),
        .if_packet(if_packet)       
    );

    //////////////////////////////////////////////////
    //          IF ID Pipeline Register             //
    //////////////////////////////////////////////////

    // TODO write if_id_enable logic 
    assign if_id_enable = 1;
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

    //////////////////////////////////////////////////
    //          Instruction Dispatch Signals        //
    //////////////////////////////////////////////////

    // CDB Signals 
    logic cdb_broadcast_en;
    logic [`PHYS_REG_IDX_SZ:0] cdb_ready_reg;
    logic [`XLEN-1:0] cdb_data;

    // Rollback signals 
    logic rollback; // TODO probably more

    // Retire stage signals 
    logic retire_move_head; // TODO incorperate retire entirely into 'dispatch'? Maybe move all this into the pipeline?

    // Signals from functional units (so, complete stage)
    // TODO probably don't need all of these/could compact
    logic [`NUM_FU_ALU-1:0] rs_free_alu;
    logic [`NUM_FU_MULT-1:0] rs_free_mult;
    logic [`NUM_FU_LOAD-1:0] rs_free_load;
    logic [`NUM_FU_STORE-1:0] rs_free_store;
    logic [`NUM_FU_BRANCH-1:0] rs_free_branch;

    // Stall output signal
    // TODO branch addresses?
    logic id_needs_stall;


    //////////////////////////////////////////////////
    //          Instruction Dispatch Modules        //
    //////////////////////////////////////////////////

    // TODO figure out which of these is correct--where does free come from
    // assign rs_free_alu = ex_free_alu;
    // assign rs_free_mult = ex_free_mult;
    // assign rs_free_load = ex_free_load;
    // assign rs_free_store = ex_free_store;
    // assign rs_free_branch = ex_free_branch;

    assign rs_free_alu = co_free_alu;
    assign rs_free_mult = co_free_mult;
    assign rs_free_load = co_free_load;
    assign rs_free_store = co_free_store;
    assign rs_free_branch = co_free_branch;


    dispatch dispatch_0 (
        // Inputs 
        .clock(clock),
        .reset(reset),

        .if_id_packet(if_id_packet), // From pipeline reg

        // from CDB
        .cdb_broadcast_en(cdb_broadcast_en),
        .cdb_ready_reg(cdb_ready_reg),

        .rollback(rollback), // from somewhere (TODO)

        .retire_move_head(retire_move_head), // from retire stage

        // from functional units
        .rs_free_alu(rs_free_alu),
        .rs_free_mult(rs_free_mult),
        .rs_free_load(rs_free_load),
        .rs_free_store(rs_free_store),
        .rs_free_branch(rs_free_branch),

        // Outputs
        .stall(id_needs_stall),
        .id_packet(id_packet)
    );


    //////////////////////////////////////////////////
    //          ID IS Pipeline Register             //
    //////////////////////////////////////////////////

    // TODO figure out this logic 
    assign id_is_enable = 1'b1; 
    always_ff @(posedge clock) begin
        if (reset) begin   
            id_is_reg <= `INVALID_ID_IS_PACKET;
        end else if (id_is_enable) begin
            id_is_reg <= id_packet;
        end
    end
    //////////////////////////////////////////////////
    //          Issue/Regfile Signals               //
    //////////////////////////////////////////////////
    
    // Regfile inputs 
    logic [`PHYS_REG_IDX_SZ:0] rf_read_idx1, rf_read_idx2, rf_write_idx;
    
    logic rf_write_en;
    logic [`XLEN-1:0] rf_write_data;

    // Regfile outputs
    logic [`XLEN-1:0] rf_read_data1, rf_read_data2;

    // Issue inputs
    logic [`XLEN-1:0] is_opa_data, is_opb_data;

    //////////////////////////////////////////////////
    //          Instruction Issue                   //
    //////////////////////////////////////////////////

    // TODO potentially eventually remove issue stage?
    // All it does is pass things forward and communicate
    // with registers

    assign rf_write_en = cdb_broadcast_en;
    assign rf_write_idx = cdb_ready_reg;
    assign rf_write_data = cdb_data;

    regfile regfile_0 (
        .clock(clock),
        .reset(reset),

        .read_idx_1(rf_read_idx1),
        .read_idx_2(rf_read_idx2),

        .write_idx(rf_write_idx),
        .write_en(rf_write_en),
        .write_data(rf_write_data),

        //outputs
        .read_out_1(rf_read_data1),
        .read_out_2(rf_read_data2)
    );

    assign is_opa_data = rf_read_data1;
    assign is_opb_data = rf_read_data2;

    issue issue_0 ( 
        .id_is_reg(id_is_reg),

        .opa_preg_data(is_opa_data),
        .opb_preg_data(is_opb_data),

        .opa_preg_idx(rf_read_idx1),
        .opb_preg_idx(rf_read_idx2),

        .is_packet(is_packet)
    );

    //////////////////////////////////////////////////
    //          IS EX Pipeline Register             //
    //////////////////////////////////////////////////
    // TODO figure out this logic 
    assign is_ex_enable = 1'b1; 
    always_ff @(posedge clock) begin
        if (reset) begin   
            is_ex_reg <= `INVALID_ID_IS_PACKET;
        end else if (id_is_enable) begin
            is_ex_reg <= is_packet;
        end
    end
    
    //////////////////////////////////////////////////
    //          Execution Signals                   //
    //////////////////////////////////////////////////
    // TODO move each FU to just be in the pipeline?
    // ex stage outputs 
    logic [`NUM_FU_ALU-1:0] ex_free_alu;
    logic [`NUM_FU_MULT-1:0] ex_free_mult;
    logic [`NUM_FU_LOAD-1:0] ex_free_load;
    logic [`NUM_FU_STORE-1:0] ex_free_store;
    logic [`NUM_FU_BRANCH-1:0] ex_free_branch;

    //////////////////////////////////////////////////
    //          Execution Stage                     //
    //////////////////////////////////////////////////
    stage_ex stage_ex_0 (
        .clock(clock),
        .reset(reset),

        .is_ex_reg(is_ex_reg),

        // outputs
        .ex_packet(ex_packet),

        .ex_free_alu(ex_free_alu),
        .ex_free_mult(ex_free_mult),
        .ex_free_load(ex_free_load),
        .ex_free_store(ex_free_store),
        .ex_free_branch(ex_free_branch)
    );

    //////////////////////////////////////////////////
    //          EX CO Pipeline Register             //
    //////////////////////////////////////////////////
    // TODO figure out this logic
    assign ex_co_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            ex_co_reg      <= `INVALID_EX_CO_PACKET;
        end else if (ex_co_enable) begin
      //      ex_co_inst_dbg <= id_ex_inst_dbg; // debug output, just forwarded from ID
            ex_co_reg      <= ex_packet;
        end
    end

    //////////////////////////////////////////////////
    //          Complete Signals                    //
    //////////////////////////////////////////////////

    // Output from Complete Stage to CDB
    logic co_output_en;
    logic [`PHYS_REG_IDX_SZ:0] co_output_idx; 
    logic [`XLEN-1:0] co_output_data;

    // Output from Complete Stage to Dispatch/RS
    logic [`NUM_FU_ALU-1:0] co_free_alu;
    logic [`NUM_FU_MULT-1:0] co_free_mult;
    logic [`NUM_FU_BRANCH-1:0] co_free_branch;
    logic [`NUM_FU_LOAD-1:0] co_free_load;
    logic [`NUM_FU_STORE-1:0] co_free_store;

    //////////////////////////////////////////////////
    //          Complete Stage                      //
    //////////////////////////////////////////////////
    // TODO could extract this into the pipeline?

    assign cdb_broadcast_en = co_output_en;
    assign cdb_ready_reg = co_output_idx;
    assign cdb_data = co_output_data;

    complete complete_0 (
        .ex_co_reg(ex_co_reg),
        .co_packet(co_packet),

        // CDB output
        .co_output_en(co_output_en),
        .co_output_idx(co_output_idx),
        .co_output_data(co_output_data),

        .co_free_alu(co_free_alu),
        .co_free_mult(co_free_mult),
        .co_free_branch(co_free_branch),
        .co_free_load(co_free_load),
        .co_free_store(co_free_store)
    );

    //////////////////////////////////////////////////
    //          CO RE Pipeline Register             //
    //////////////////////////////////////////////////
    // TODO figure out this logic
    assign co_re_enable = 1'b1; // always enabled
    // synopsys sync_set_reset "reset"
    always_ff @(posedge clock) begin
        if (reset) begin
            co_re_reg      <= `INVALID_CO_RE_PACKET;
        end else if (ex_co_enable) begin
      //      ex_co_inst_dbg <= id_ex_inst_dbg; // debug output, just forwarded from ID
            co_re_reg      <= co_packet;
        end
    end

    //////////////////////////////////////////////////
    //          Retire Signals                      //
    //////////////////////////////////////////////////

    logic [$clog2(`ROB_SZ)-1:0] re_rob_head;
    logic clear_retire_buffer;

    //////////////////////////////////////////////////
    //          Retire Stage                        //
    //////////////////////////////////////////////////
    // TODO could extract this into the pipeline?
    retire retire_0 (

    );








   

endmodule // pipeline
