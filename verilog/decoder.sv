
// The decoder, copied from p3/stage_id.sv without changes

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"

// Decode an instruction: generate useful datapath control signals by matching the RISC-V ISA
// This module is purely combinational
module decoder (
    input INST  inst,
    input logic valid, // when low, ignore inst. Output will look like a NOP

    output ALU_OPA_SELECT opa_select,
    output ALU_OPB_SELECT opb_select,
    output logic has_dest,  // if there is a destination register
    output logic has_rs1,  // if there is a source register 1
    output logic has_rs2,  // if there is a source register 2
    output ALU_FUNC alu_func,
    output logic rd_mem,
    wr_mem,
    cond_branch,
    uncond_branch,
    output logic          csr_op, // used for CSR operations, we only use this as a cheap way to get the return code out
    output logic halt,  // non-zero on a halt
    output logic illegal,  // non-zero on an illegal instruction
    output FUNIT function_type
);

  // Note: I recommend using an IDE's code folding feature on this block
  always_comb begin
    // Default control values ((might no longer) looks like a NOP)
    // See sys_defs.svh for the constants used here
    opa_select    = OPA_IS_RS1;
    opb_select    = OPB_IS_RS2;
    alu_func      = ALU_ADD;
    function_type = ALU;
    has_dest      = `FALSE;
    has_rs1       = `TRUE;
    has_rs2       = `TRUE;
    csr_op        = `FALSE;
    rd_mem        = `FALSE;
    wr_mem        = `FALSE;
    cond_branch   = `FALSE;
    uncond_branch = `FALSE;
    halt          = `FALSE;
    illegal       = `FALSE;

    if (valid) begin
      casez (inst)
        `RV32_LUI: begin  /* U-type */
          has_dest   = `TRUE;
          has_rs1    = `FALSE;
          has_rs2    = `FALSE;
          opa_select = OPA_IS_ZERO;
          opb_select = OPB_IS_U_IMM;
        end
        `RV32_AUIPC: begin  /* U-type */
          has_dest   = `TRUE;
          has_rs1    = `FALSE;
          has_rs2    = `FALSE;
          opa_select = OPA_IS_PC;
          opb_select = OPB_IS_U_IMM;
        end
        `RV32_JAL: begin  /* U-type */
          has_dest      = `TRUE;
          has_rs1       = `FALSE;
          has_rs2       = `FALSE;
          opa_select    = OPA_IS_PC;
          opb_select    = OPB_IS_J_IMM;
          uncond_branch = `TRUE;
          function_type = BRANCH;
        end
        `RV32_JALR: begin  /* I-type */
          has_dest      = `TRUE;
          has_rs2       = `FALSE;
          opa_select    = OPA_IS_RS1;
          opb_select    = OPB_IS_I_IMM;
          uncond_branch = `TRUE;
          function_type = BRANCH;
        end
        `RV32_BEQ, `RV32_BNE, `RV32_BLT, `RV32_BGE, `RV32_BLTU, `RV32_BGEU: begin  /* S-type */
          opa_select = OPA_IS_PC;
          opb_select = OPB_IS_B_IMM;
          cond_branch = `TRUE;
          function_type = BRANCH;
        end
        `RV32_LB, `RV32_LH, `RV32_LW, `RV32_LBU, `RV32_LHU: begin  /* I-type */
          has_dest      = `TRUE;
          has_rs2       = `FALSE;
          opb_select    = OPB_IS_I_IMM;
          rd_mem        = `TRUE;
          function_type = LOAD;
        end
        `RV32_SB, `RV32_SH, `RV32_SW: begin  /* S-type */
          has_rs1       = `TRUE;
          has_rs2       = `TRUE;
          opb_select    = OPB_IS_S_IMM;
          wr_mem        = `TRUE;
          function_type = STORE;
        end
        `RV32_ADDI: begin  /* I-type */
          has_dest   = `TRUE;
          has_rs2    = `FALSE;
          opb_select = OPB_IS_I_IMM;
        end
        `RV32_SLTI: begin  /* I-type */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_SLT;
        end
        `RV32_SLTIU: begin  /* I-type */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_SLTU;
        end
        `RV32_ANDI: begin  /* I-type */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_AND;
        end
        `RV32_ORI: begin  /* I-type */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_OR;
        end
        `RV32_XORI: begin  /* I-type */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_XOR;
        end
        `RV32_SLLI: begin  /* I-type? */
          has_dest   = `TRUE;
          has_rs2    = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func   = ALU_SLL;
        end
        `RV32_SRLI: begin  /* I-type? */
          has_dest = `TRUE;
          has_rs2 = `FALSE;
          opb_select = OPB_IS_I_IMM;
          alu_func = ALU_SRL;
        end
        `RV32_SRAI: begin  /* I-type? */
          has_dest   = `TRUE;
          opb_select = OPB_IS_I_IMM;
          alu_func   = ALU_SRA;
        end
        `RV32_ADD: begin  /* R-type */
          has_dest = `TRUE;
        end
        `RV32_SUB: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SUB;
        end
        `RV32_SLT: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SLT;
        end
        `RV32_SLTU: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SLTU;
        end
        `RV32_AND: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_AND;
        end
        `RV32_OR: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_OR;
        end
        `RV32_XOR: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_XOR;
        end
        `RV32_SLL: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SLL;
        end
        `RV32_SRL: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SRL;
        end
        `RV32_SRA: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_SRA;
        end
        `RV32_MUL: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_MUL;
          function_type = MULT;
        end
        `RV32_MULH: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_MULH;
          function_type = MULT;
        end
        `RV32_MULHSU: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_MULHSU;
          function_type = MULT;
        end
        `RV32_MULHU: begin  /* R-type */
          has_dest = `TRUE;
          alu_func = ALU_MULHU;
          function_type = MULT;
        end
        `RV32_CSRRW, `RV32_CSRRS, `RV32_CSRRC: begin
          csr_op  = `TRUE;
          has_rs1 = `FALSE;
          has_rs2 = `FALSE;
        end
        `WFI: begin
          halt = `TRUE;
          has_rs1 = `FALSE;
          has_rs2 = `FALSE;
        end
        default: begin
          illegal = `TRUE;
          has_rs1 = `FALSE;
          has_rs2 = `FALSE;
        end
      endcase  // casez (inst)
    end  // if (valid)
  end  // always

endmodule  // decoder
