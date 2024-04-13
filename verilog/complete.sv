//TODO: add signals to free RS

module complete(
    input EX_CO_PACKAGE ex_co_reg; //need to add type
    input [`MAX_FU_INDEX-1:0] fu_index;
    output CO_RE_PACKAGE co_package;
    output logic             regfile_en,  // register write enable
    output logic [`PHYS_REG_SZ-1:0]              regfile_idx, // register write index
    output logic [`XLEN-1:0] regfile_data, // register write data 
    output [`NUM_FU_ALU-1:0] free_alu,
    output [`NUM_FU_ALU-1:0] free_mult,
    output [`NUM_FU_ALU-1:0] free_branch,
    output [`NUM_FU_ALU-1:0] free_load,
    output [`NUM_FU_ALU-1:0] free_store,
);

    assign co_package.result = ex_co_reg.result;
    assign co_package.NPC = ex_co_reg.NPC;
    assign co_package.dest_reg_idx = ex_co_reg.dest_reg_idx; // writeback destination (ZERO_REG if no writeback)
    assign co_package.take_branch = ex_co_reg.take_branch;
    assign co_package.halt = ex_co_reg.halt;    // not used by wb stage
    assign co_package.illegal = ex_co_reg.illegal; // not used by wb stage
    assign co_package.valid = ex_co_reg.valid;
    assign co_package.rob_index = ex_co_reg.rob_index;

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