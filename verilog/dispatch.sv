`include "verilog/decoder.sv"
`include "verilog/map_table.sv"
`include "verilog/free_list.sv"
`include "verilog/rob.sv"
`include "verilog/reservation_station.sv"

module dispatch (
    input clock, reset, // TODO implement reset everywhere
    input IF_ID_PACKET if_id_packet,    // passed from ifetch stage (new)
   
    // Input from CDB (a Phys reg to mark ready)
    input logic cdb_broadcast_en,       // (new) broadcast from CDB
    input logic [`PHYS_REG_IDX_SZ:0] cdb_ready_reg, // (new) ready register from RS

    // Rollback signal (from ex stage?)
    input logic rollback,              // (new) will clear ROB and revert Map Table 

    // Input from retire stage
    input logic retire_move_head,       // (new) retire signal to move head

    // Input from complete stage, freeing reservation station (TODO make into enum)
    input logic [`NUM_FU_ALU-1:0]free_alu,              // (new) free ALU entry
    input logic [`NUM_FU_MULT-1:0]free_mult,             // (new) free MULT entry
    input logic [`NUM_FU_LOAD-1:0]free_load,             // (new) free LOAD entry
    input logic [`NUM_FU_STORE-1:0]free_store,            // (new) free STORE entry
    input logic [`NUM_FU_BRANCH-1:0]free_branch,           // (new) free BRANCH entry



    output ID_IS_PACKET id_packet,      // outputs (new) 
    // if rob or RS is not abvailable, stall will be high, keep all inputs same as pervious cycle.
    output logic stall,          // (new) stall signal to ifetch stage 


    // Output to retire stage
    output logic [$clog2(`ROB_SZ)-1:0] rob_head_idx,

    // Debug outputs
    output logic rob_full,
    output logic rs_full,
    output logic free_list_empty
);

    // if_id_packet comes in from fetch stage, mostly passed to decoder
    // decoded_packet is output mostly of decoder, passed to RS/ROB
    // id_packet is output of RS, which is actually issued

        
    // Basic structure:
    // Declare RS, ROB, Map Table (and arch map), and Free List
    // Calculate 'stall' signal based on ROB full, RS full, and Free List empty
    // Issue an instruction if RS is ready for it 
    // Update components based on CDB (RS, Map Table?)
    // On rollback, flush ROB, RS and revert Map Table to Arch Map
    // Retire stage stuff
        // Broadcast ROB head pointer (for retire stage)
        // Pass retire signal (move_head) to ROB, and pass ROB output to update arch map 
    // If stall is high:
        // Send stall signal to fetch stage
        // Keep all inputs same as pervious cycle
    // If stall is low:
        // Dequeue from free list (generating a physical dest)
        // Access map table (convert arch opa/opb to physical) 
        // Update map table with new dest (if applicable)
        // Add instruction to ROB,RS
        


    
    // Declare Decoder
    
    ID_IS_PACKET decoded_packet;

    decoder decoder1 (
        // inputs
        .inst(if_id_packet.inst), 
        .valid(if_id_packet.valid),
        // outputs
        .opa_select(decoded_packet.opa_select),
        .opb_select(decoded_packet.opb_select),
        .has_dest(decoded_packet.has_dest),
        .alu_func(decoded_packet.alu_func),
        .rd_mem(decoded_packet.rd_mem),
        .wr_mem(decoded_packet.wr_mem),
        .cond_branch(decoded_packet.cond_branch),
        .uncond_branch(decoded_packet.uncond_branch),
        .csr_op(decoded_packet.csr_op),
        .halt(decoded_packet.halt),
        .illegal(decoded_packet.illegal),
        .function_type(decoded_packet.function_type)
    );

    // Declare Map Table
    
    // Inputs to getting from map table
    logic [`REG_IDX_SZ:0] mt_arch_reg1_idx;
    logic [`REG_IDX_SZ:0] mt_arch_reg2_idx;

    // Outputs from getting from map table
    PREG mt_preg1_out;
    PREG mt_preg2_out;

    logic [`REG_IDX_SZ:0] mt_arch_map_arch_reg_idx;
    PREG mt_arch_map_phys_reg_out;


    logic [`REG_IDX_SZ:0] mt_arch_dest_idx;
    logic mt_set_dest_enable;
    logic [`PHYS_REG_IDX_SZ:0] mt_new_dest_pr_idx;

    PREG mt_old_dest_pr;

    logic [`REG_IDX_SZ:0] mt_retire_arch_reg;
    logic mt_retire_enable;


    map_table map_table_0 (
        .clk(clock),
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
        .new_dest_pr_idx(mt_new_dest_pr_idx),
        // output
        .old_dest_pr(mt_old_dest_pr),

        /* GET physical register from architecture map */
        .arch_map_arch_reg_idx(mt_arch_map_arch_reg_idx), // Input
        .arch_map_phys_reg_out(mt_arch_map_phys_reg_out), // Outpu


        /* Retire operation */
        // inputs
        .retire_arch_reg(mt_retire_arch_reg),
        .retire_enable(mt_retire_enable),

        /* Restore operation */
        // inputs
        .restore_enable(rollback),

        /* SET READY operation (CDB) */
        // inputs
        .set_ready_enable(cdb_broadcast_en),
        .ready_phys_idx(cdb_ready_reg)
    );


    // Declare Free List

    // many of these signals are debug signals
   // logic free_list_empty;

    logic fl_dequeue_en, fl_was_dequeued;
    logic [`PHYS_REG_IDX_SZ:0] fl_dequeue_pr, fl_head_pr;

    logic [`PHYS_REG_IDX_SZ:0] fl_enqueue_pr;

    logic fl_enqueue_en, fl_was_enqueued;

    logic fl_is_full;

    logic [`PHYS_REG_IDX_SZ+1:0] fl_back_tail_ptr;
    logic [`PHYS_REG_IDX_SZ+1:0] fl_front_head_ptr;
    logic [`PHYS_REG_IDX_SZ:0] fl_free_list[`FREE_LIST_SIZE];

    free_list free_list_0 (
        .clk(clock),
        .reset(reset),

        /* READ from head operation */

        // inputs 
        .dequeue_en(fl_dequeue_en), // If raised, will actually deuqueue from the head
        
        // outputs
        .front_head_pr(fl_head_pr), // Always points to the next element to be dequeued, unless is_empty
        .is_empty(free_list_empty), // IF is_empty, front_head_pr is undefined (new)

        // debug outputs
        .was_dequeued(fl_was_dequeued),
        .dequeue_pr(fl_dequeue_pr), // TODO combine with fl_head_pr? should always be the same


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

    //  Declare ROB
   // logic rob_full; // Is the ROB currently full (we should stall)
    
    logic [$clog2(`ROB_SZ)-1:0] head; // Index of the current head printer in the ROB
    logic [$clog2(`ROB_SZ)-1:0] rob_index; // Index of the current instruction in the ROB

    reorder_buffer reorder_buffer_0(
        .clock(clock),
        .reset(reset),
        
        // Instruction input to store in ROB (if valid)
        .write(decoded_packet.valid),
        .inst(decoded_packet.inst),
        .tag(decoded_packet.dest_reg),

        // input from retire stage
        .move_head(retire_move_head), 

        // Undo stuff TODO figure this out
        .undo(rollback), // If signal raised, will move next tail back to undo_index+1
        .undo_index(head), // always roll back all the way to head

        //TODO: need to connect the actual interconnects

        // Ouput to free physical register at index free_index
        .update_free_list(fl_enqueue_en),
        .free_index(fl_enqueue_pr),
        
        // Output to update arch map to mark arch_told->phys_told as retired/committed
        .update_arch_map(mt_retire_enable),
        .update_arch_told(mt_retire_arch_reg),
        //.update_phys_told(update_phys_told),

        // Retrieve told from arch map
        .arch_told(mt_arch_map_arch_reg_idx), // output
        .phys_told(mt_arch_map_phys_reg_out.reg_num), // input (from arch map)
        
        // output 
        .inst_index(rob_index),
        .full(rob_full), // Output if ROB is full (new)
        .head(head)
    );
    
    // Declare RS
    PREG rs_ready_reg;
    assign rs_ready_reg.reg_num = cdb_ready_reg;
    assign rs_ready_reg.ready = cdb_broadcast_en; // TODO this is duplicate

    /* TODO figure out if we ever need to not issue */
    logic issue_enable;
    assign issue_enable = 1;

    reservation_station reservation_station_0(
        .clock(clock),
        .reset(reset),
        
        /* Allocate */
        .allocate(decoded_packet.valid),
        .input_packet(decoded_packet),

        .done(alloc_done),

        /* Update */
        .update(cdb_broadcast_en),
        .ready_reg(rs_ready_reg),

        /* Issue */
        .issue_enable(issue_enable), // TODO figure out
       // .ready(id_packet.valid),
        .issued_packet(id_packet),
        .issue_fu_index(issue_fu_index),

        // Output whether or not each type of FU is full (new)
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

    /* Just forward some info */
    assign decoded_packet.inst = if_id_packet.inst;
    assign decoded_packet.PC   = if_id_packet.PC;
    assign decoded_packet.NPC  = if_id_packet.NPC;
    assign decoded_packet.rob_index = rob_index;

    /* Calculate STALL logic */
    //logic rs_full;
    
    assign rs_full = (alu_entries_full & (decoded_packet.function_type == ALU))
                   | (mult_entries_full & (decoded_packet.function_type == MULT)) 
                   | (load_entries_full & (decoded_packet.function_type == LOAD)) 
                   | (store_entries_full & (decoded_packet.function_type == STORE)) 
                   | (branch_entries_full & (decoded_packet.function_type == BRANCH));
                   
    // rstall if ROB full or RS full or free list empty
    assign stall = (rob_full | rs_full | free_list_empty);

   

     /* Access map table (convert arch opa/opb to physical) */

    // Get arch regs from decoded packet
    assign mt_arch_reg1_idx = decoded_packet.opa_select ? decoded_packet.inst.r.rs1 : `ZERO_REG;
    assign mt_arch_reg2_idx = decoded_packet.opb_select ? decoded_packet.inst.r.rs2 : `ZERO_REG;

    // Update decoded packet with physical regs from map table
    assign decoded_packet.src1_reg = mt_preg1_out;
    assign decoded_packet.src2_reg = mt_preg2_out;
    
    /* Mark decoded packet as valid if it is valid and we are ready to pass to RS/ROB (no stall)*/
    assign decoded_packet.valid = if_id_packet.valid & ~decoded_packet.illegal & (!stall);

    /* Dequeue from free list (generating a physical dest) */
    assign fl_dequeue_en = decoded_packet.valid & decoded_packet.has_dest;
    assign decoded_packet.dest_reg.reg_num = fl_dequeue_pr; 
    // If a packet doesn't have destination, it must be ready
    assign decoded_packet.dest_reg.ready = ~decoded_packet.has_dest;

    /* Update Map Table with newly dequeued pr dest if it should exist */
    assign mt_set_dest_enable = decoded_packet.valid & decoded_packet.has_dest;
    assign mt_arch_dest_idx = decoded_packet.inst.r.rd;
    assign mt_new_dest_pr_idx = decoded_packet.dest_reg.reg_num; // TODO should this be PREG?

endmodule // decoder