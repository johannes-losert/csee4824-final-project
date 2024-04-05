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


    //////////////////////////////////////////////////
    //                                              //
    //               Instruction Fetch (F)           //
    //                                              //
    //////////////////////////////////////////////////

    // REASONS TO STALL IF
        // - free list empty (no PR to allocate)
        // - ROB full (no ROB entry to allocate)

    // Instruction fetch stall state
    always_ff @(posedge clock) begin
        next_if_valid <= 1'b1;
        if (/*something*/) begin
            next_if_valid <= 1'b0;
        end
    end

    // Instantiate instruction fetch module (PLACEHOLDER)
    instruction_fetch fetcher (
        .clock(clock),
        .reset(reset),
        
        .next_if_valid(next_if_valid), // input: if lowered, must stall (don't give us more) 

        /* Instruction memory interface */
        .proc2mem_addr(proc2Imem_addr), // output: address requested
        .mem2proc_data(mem2proc_data), // input: data recieved
        
        /* SOME INPUTS: (branch prediction stuff?) */

        /* OUTPUTS: packet retrieved, enable signal */
        .fetched_packet(if_packet),
        .fetch(if_id_enable)    // if lowered, must stall decode (is this needed?)
    );


    // TODO IF/ID register (sequential)

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


    //////////////////////////////////////////////////
    //                                              //
    //               Execute (E)                    //
    //                                              //
    //////////////////////////////////////////////////
    // Perform each operation 


    //////////////////////////////////////////////////
    //                                              //
    //               Complete (C)                   //
    //                                              //
    //////////////////////////////////////////////////    
    // Write register file 

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



   

endmodule // pipeline
