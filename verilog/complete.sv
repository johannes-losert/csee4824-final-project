//TODO: add signals to free RS

module complete(
    input EX_CO_PACKET ex_co_reg; //need to add type
    
    // input [`MAX_FU_INDEX-1:0] fu_index;
    
    output CO_RE_PACKET co_packet;

    // Completed instruction info (to CDB)
    output logic             regfile_en,  // register write enable
    output logic [`PHYS_REG_SZ-1:0]              regfile_idx, // register write index
    output logic [`XLEN-1:0] regfile_data, // register write data 
    
    // Newly freed FU info (to dispatch/RS)
    output [`NUM_FU_ALU-1:0] free_alu,
    output [`NUM_FU_ALU-1:0] free_mult,
    output [`NUM_FU_ALU-1:0] free_branch,
    output [`NUM_FU_ALU-1:0] free_load,
    output [`NUM_FU_ALU-1:0] free_store,
);

    logic [`MAX_FU_INDEX-1:0] fu_index;
    assign fu_index = ex_co_reg.issued_fu_index;

    // Passthrough
    assign co_packet.inst = ex_co_reg.inst;
    assign co_packet.PC = ex_co_reg.PC;
    assing co_packet.NPC = ex_co_reg.NPC;

    assign co_packet.opa_select = ex_co_reg.opa_select;
    assign co_packet.opb_select = ex_co_reg.opb_select;

    assign co_packet.opa_value = ex_co_reg.opa_value;
    assign co_packet.opb_value = ex_co_reg.opb_value;

    assign co_packet.alu_func = ex_co_reg.alu_func;
    assign co_packet.rd_mem = ex_co_reg.rd_mem;
    assign co_packet.wr_mem = ex_co_reg.wr_mem;
    assign co_packet.cond_branch = ex_co_reg.cond_branch;
    assign co_packet.uncond_branch = ex_co_reg.uncond_branch;
    assign co_packet.halt = ex_co_reg.halt;
    assign co_packet.illegal = ex_co_reg.illegal;
    assign co_packet.csr_op = ex_co_reg.csr_op;

    assign co_packet.function_type = ex_co_reg.function_type;
    assign co_packet.valid = ex_co_reg.valid;

    

    // assign co_packet.result = ex_co_reg.result;
    // assign co_packet.NPC = ex_co_reg.NPC;
    // assign co_packet.dest_reg_idx = ex_co_reg.dest_reg_idx; // writeback destination (ZERO_REG if no writeback)
    // assign co_packet.take_branch = ex_co_reg.take_branch;
    // assign co_packet.halt = ex_co_reg.halt;    // not used by wb stage
    // assign co_packet.illegal = ex_co_reg.illegal; // not used by wb stage
    // assign co_packet.valid = ex_co_reg.valid;
    // assign co_packet.rob_index = ex_co_reg.rob_index;

    always_comb begin : 
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

    // This enable computation is sort of overkill since the reg file
    // also handles the `ZERO_REG case, but there's no harm in putting this here
    // the valid check is also somewhat redundant
    assign regfile_en = ex_co_reg.valid && (ex_co_reg.dest_reg_idx != `ZERO_REG);

    assign regfile_idx = ex_co_reg.dest_reg_idx; // PHYSICAL register index 

    // Select register writeback data:
    // ALU/MEM result, unless taken branch, in which case we write
    // back the old NPC as the return address. Note that ALL branches
    // and jumps write back the 'link' value, but those that don't
    // use it specify ZERO_REG as the destination.
    assign regfile_data = (ex_co_reg.take_branch) ? ex_co_reg.NPC : ex_co_reg.result;
    
endmodule // complete