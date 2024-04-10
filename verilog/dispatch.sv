module dispatch (
    input IF_ID_PACKET if_id_packet,    // passed from ifetch stage
    input rob_stall,                    // indicates if rob is stalled due to no empty free list
    input rob_full,                     // indicates if rob is full
    // four full signals from RS, indicating the avalibility of specific funits
    input logic alu_entries_full,       
    input logic mult_entries_full,
    input logic load_entries_full,
    input logic store_entries_full,
    input logic branch_entries_full,

    output ID_IS_PACKET id_packet,      // outputs 
    // if rob or RS is not abvailable, stall will be high, keep all inputs same as pervious cycle.
    output logic stall                  
);
    // For counting valid instructions executed
    // and making the fetch stage die on halts/keeping track of when
    // to allow the next instruction out of fetch
    // 0 for HALT, ROB or RS not available, and illegal instructions (end processor on halt)
    
    assign id_packet.valid = if_id_packet.valid & ~id_packet.illegal & (!stall);

    //check if rob and rs is available
    assign stall = (rob_stall | rob_full \
                              | (alu_entries_full & (id_packet.function_type == `ALU)) \
                              | (mult_entries_full & (id_packet.function_type == `MULT)) \
                              | (load_entries_full & (id_packet.function_type == `LOAD)) \
                              | (store_entries_full & (id_packet.function_type == `STORE)) \
                              | (branch_entries_full & (id_packet.function_type == `BRANCH)) \
    );


    logic has_dest_reg;
    assign id_packet.dest_reg_idx = (has_dest_reg) ? if_id_packet.inst.r.rd : `ZERO_REG;

    decoder decoder1 (
        // inputs
        .inst(if_id_packet.inst), 
        .valid(if_id_packet.valid),
        // outputs
        .opa_select(id_packet.opa_select),
        .opb_select(id_packet.opb_select),
        .has_dest(id_packet.has_dest),
        .alu_func(id_packet.alufunc),
        .rd_mem(id_packet.rd_mem),
        .wr_mem(id_packet.wr_mem),
        .cond_branch(id_packet.cond_branch),
        .uncond_branch(id_packet.uncond_branch),
        .csr_op(id_packet.csr_op),
        .halt(id_packet.halt),
        .illegal(id_packet.illegal),
        .function_type(id_packet.function_type)
    );

    assign id_packet.inst = if_id_packet.inst;
    assign id_packet.PC   = if_id_packet.PC;
    assign id_packet.NPC  = if_id_packet.NPC;
    
    
endmodule // decoder