module complete(
    input EX_CO_PACKAGE ex_co_reg; //need to add type
    output CO_RE_PACKAGE co_package;
    output logic             regfile_en,  // register write enable
    output PREG              regfile_idx, // register write index
    output logic [`XLEN-1:0] regfile_data // register write data 
);

    assign co_package = ex_co_reg;
    
    // This enable computation is sort of overkill since the reg file
    // also handles the `ZERO_REG case, but there's no harm in putting this here
    // the valid check is also somewhat redundant
    assign regfile_en = ex_co_reg.valid && (ex_co_reg.dest_reg_idx != `ZERO_REG);

    assign regfile_idx.reg_num = ex_co_reg.dest_reg_idx;
    assign regfile_idx.ready = regfile_en;

    // Select register writeback data:
    // ALU/MEM result, unless taken branch, in which case we write
    // back the old NPC as the return address. Note that ALL branches
    // and jumps write back the 'link' value, but those that don't
    // use it specify ZERO_REG as the destination.
    assign regfile_data = (ex_co_reg.take_branch) ? ex_co_reg.NPC : ex_co_reg.result;
    
endmodule // complete