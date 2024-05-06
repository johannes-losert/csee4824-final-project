module complete (
    input EX_CO_PACKET ex_co_reg,

    output CO_RE_PACKET co_packet,

    // If rolling back, invalidate the current instruction 
    input logic rollback,

    // Completed instruction info (to CDB)
    output logic co_output_en,  // register write enable
    output logic [`PHYS_REG_IDX_SZ:0] co_output_idx,  // register write index
    output logic [`XLEN-1:0] co_output_data,  // register write data 

    // Newly freed FU info (to dispatch/RS)
    output logic [`NUM_FU_ALU-1:0] free_alu,
    output logic [`NUM_FU_MULT-1:0] free_mult,
    output logic [`NUM_FU_BRANCH-1:0] free_branch,
    output logic [`NUM_FU_LOAD-1:0] free_load,
    output logic [`NUM_FU_STORE-1:0] free_store
);

  // Passthrough
  assign co_packet.inst = ex_co_reg.inst;
  assign co_packet.PC = ex_co_reg.PC;
  assign co_packet.NPC = ex_co_reg.NPC;

  assign co_packet.opa_select = ex_co_reg.opa_select;
  assign co_packet.opb_select = ex_co_reg.opb_select;

  assign co_packet.rs1_value = ex_co_reg.rs1_value;
  assign co_packet.rs2_value = ex_co_reg.rs2_value;

  assign co_packet.alu_func = ex_co_reg.alu_func;
  assign co_packet.rd_mem = ex_co_reg.rd_mem;
  assign co_packet.wr_mem = ex_co_reg.wr_mem;
  assign co_packet.cond_branch = ex_co_reg.cond_branch;
  assign co_packet.uncond_branch = ex_co_reg.uncond_branch;
  assign co_packet.halt = ex_co_reg.halt;
  assign co_packet.illegal = ex_co_reg.illegal;
  assign co_packet.csr_op = ex_co_reg.csr_op;

  assign co_packet.function_type = ex_co_reg.function_type;


  /* If rolling back, invalidate packet */
  assign co_packet.valid = rollback ? 0 : ex_co_reg.valid;

  assign co_packet.rob_index = ex_co_reg.rob_index;
  assign co_packet.has_dest = ex_co_reg.has_dest;

  assign co_packet.issued_fu_index = ex_co_reg.issued_fu_index;

  assign co_packet.arch_dest_reg_num = ex_co_reg.arch_dest_reg_num;

  assign co_packet.result = ex_co_reg.result;
  assign co_packet.take_branch = ex_co_reg.take_branch;
  assign co_packet.mem_size = ex_co_reg.mem_size;
  assign co_packet.prev_dword = ex_co_reg.prev_dword;

  logic [`MAX_FU_INDEX-1:0] fu_index;
  assign fu_index = ex_co_reg.issued_fu_index;

  always_comb begin
    free_alu = 0;
    free_mult = 0;
    free_load = 0;
    free_store = 0;
    free_branch = 0;
    if (ex_co_reg.valid & ~ex_co_reg.halt & ~ex_co_reg.illegal) begin
      casez (ex_co_reg.function_type)
        ALU: free_alu[fu_index] = 1;
        MULT: free_mult[fu_index] = 1;
        LOAD: free_load[fu_index] = 1;
        STORE: free_store[fu_index] = 1;
        BRANCH: free_branch[fu_index] = 1;
        default: begin
          free_branch = 0;
        end
      endcase
    end
  end

  // Somewhat redundant
  assign co_output_en = ~rollback && ex_co_reg.valid && (ex_co_reg.dest_reg_idx != `ZERO_REG);

  assign co_output_idx = ex_co_reg.dest_reg_idx;  // PHYSICAL register index 

  assign co_packet.regfile_en = co_output_en;
  assign co_packet.regfile_idx = co_output_idx;
  assign co_packet.regfile_data = co_output_data;

  // Select register writeback data
  assign co_output_data = (ex_co_reg.take_branch) ? ex_co_reg.NPC : ex_co_reg.result;

endmodule  // complete
