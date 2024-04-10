module issue (
    input RS_PACKET rs_packet,
    input ID_IS_PACKET id_is_reg,

    output IS_EX_PACKET is_packet;
);

    assign is_packet.inst = id_is_reg.inst;
    assign is_packet.PC = id_is_reg.PC;
    assign is_packet.NPC = id_is_reg.NPC;

    assign is_packet.opa_select = id_is_reg.opa_select;
    assign is_packet.opb_select = id_is_reg.obp_select;
    
    assign is_packet.dest_reg_idx = id_is_reg.dest_reg_idx;
    assign is_packet.alu_func = id_is_reg.alu_func;
    assign is_packet.rd_mem = id_is_reg.rd_mem;
    assign is_packet.wr_mem = id_is_reg.wr_mem;
    assign is_packet.cond_branch = id_is_reg.cond_branch;
    assign is_packet.uncond_branch = id_is_reg.uncond_branch;
    assign is_packet.halt = id_is_reg.halt;
    assign is_packet.illegal = id_is_reg.illegal;
    assign is_packet.csr_op = id_is_reg.csr_op;
    assign is_packet.function_type = id_is_reg.function_type;
    assign is_packet.valid = id_is_reg.valid;

endmodule