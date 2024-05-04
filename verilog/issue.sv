
// TODO should this module take in clock/reset signal?
// TODO should this module exist at all?
module issue (
    // input from id stage
    input ID_IS_PACKET id_is_reg,

    // input from preg
    input [`XLEN-1:0] rs1_preg_data,
    input [`XLEN-1:0] rs2_preg_data,

    // output to preg
    output [`PHYS_REG_IDX_SZ:0] rs1_preg_idx,
    output [`PHYS_REG_IDX_SZ:0] rs2_preg_idx,

    // output to ex stage
    output IS_EX_PACKET is_packet
);

    assign rs1_preg_idx = id_is_reg.src1_reg.reg_num;
    assign rs2_preg_idx = id_is_reg.src2_reg.reg_num;

    assign is_packet.inst = id_is_reg.inst;
    assign is_packet.PC = id_is_reg.PC;
    assign is_packet.NPC = id_is_reg.NPC;

    assign is_packet.opa_select = id_is_reg.opa_select;
    assign is_packet.opb_select = id_is_reg.opb_select;

    // Retrieved data from regfile 
    assign is_packet.rs1_value = rs1_preg_data;
    assign is_packet.rs2_value = rs2_preg_data;

    // Unpack dest reg idx (probably not needed)
    assign is_packet.dest_reg_idx = id_is_reg.dest_reg.reg_num;
    
    assign is_packet.alu_func = id_is_reg.alu_func;
    assign is_packet.rd_mem = id_is_reg.rd_mem;
    assign is_packet.wr_mem = id_is_reg.wr_mem;
    assign is_packet.cond_branch = id_is_reg.cond_branch;
    assign is_packet.uncond_branch = id_is_reg.uncond_branch;
    assign is_packet.halt = id_is_reg.halt;
    assign is_packet.illegal = id_is_reg.illegal;
    assign is_packet.csr_op = id_is_reg.csr_op;

    assign is_packet.function_type = id_is_reg.function_type;
    assign is_packet.valid = id_is_reg.valid; // TODO any more calculation?

    assign is_packet.rob_index = id_is_reg.rob_index;
    assign is_packet.has_dest = id_is_reg.has_dest;	
    assign is_packet.issued_fu_index = id_is_reg.issued_fu_index;

    assign is_packet.arch_dest_reg_num = id_is_reg.arch_dest_reg_num;

endmodule
