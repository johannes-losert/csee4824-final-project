/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  sys_defs.svh                                        //
//                                                                     //
//  Description :  This file has the macro-defines for macros used in  //
//                 the pipeline design.                                //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`ifndef __SYS_DEFS_SVH__
`define __SYS_DEFS_SVH__

// all files should `include "sys_defs.svh" to at least define the timescale
`timescale 1ns / 100ps

`define MAX_CYCLES 200000

// /* Uncomment this to enable visual debugging.
//    If DEBUG_PRINT is defined, don't set MAX_CYCLES to more than ~2000
//     */
// `define DEBUG_PRINT
// `define MAX_CYCLES 10000


///////////////////////////////////
// ---- Starting Parameters ---- //
///////////////////////////////////

// some starting parameters that you should set
// this is *your* processor, you decide these values (try analyzing which is best!)

// superscalar width
`define N 1

// sizes
`define ROB_SZ 8
`define RS_SZ 1 // TODO should this be num FUs?

`define REG_SZ 32
// REG_IDX_SZ MUST BE 4, because instruction logic assumes it
`define REG_IDX_SZ ($clog2(`REG_SZ-1))
`define PHYS_REG_SZ (`REG_SZ + `ROB_SZ)
`define PHYS_REG_IDX_SZ ($clog2(`PHYS_REG_SZ-1))

`define FREE_LIST_SIZE `PHYS_REG_SZ+1 /* One additional 'always free' slot */



// worry about these later
`define BRANCH_PRED_SZ 1
`define LSQ_SZ 1

// functional units (you should decide if you want more or fewer types of FUs)
`define NUM_FU_ALU 1
`define NUM_FU_MULT 1
`define NUM_FU_LOAD 1
`define NUM_FU_STORE 1
`define NUM_FU_BRANCH 1

// TODO make this automated, should equal clog2(max(NUM_FU_ALU, NUM_FU_MULT, NUM_FU_LOAD, NUM_FU_STORE))
`define MAX_FU_INDEX 1

// number of mult stages (2, 4, or 8)
`define MULT_STAGES 8

///////////////////////////////
// ---- Basic Constants ---- //
///////////////////////////////

// NOTE: the global CLOCK_PERIOD is defined in the Makefile

// useful boolean single-bit definitions
`define FALSE 1'h0
`define TRUE 1'h1

// data length
`define XLEN 32

// the zero register
// In RISC-V, any read of this register returns zero and any writes are thrown away
`define ZERO_REG 5'd0

// Basic NOP instruction. Allows pipline registers to clearly be reset with
// an instruction that does nothing instead of Zero which is really an ADDI x0, x0, 0
`define NOP 32'h00000013

//////////////////////////////////
// ---- Memory Definitions ---- //
//////////////////////////////////

// Cache mode removes the byte-level interface from memory, so it always returns
// a double word. The original processor won't work with this defined. Your new
// processor will have to account for this effect on mem.
// Notably, you can no longer write data without first reading.
`define CACHE_MODE

// you are not allowed to change this definition for your final processor
// the project 3 processor has a massive boost in performance just from having no mem latency
// see if you can beat it's CPI in project 4 even with a 100ns latency!
// `define MEM_LATENCY_IN_CYCLES  0
`define MEM_LATENCY_IN_CYCLES (100.0/`CLOCK_PERIOD+0.49999)
// the 0.49999 is to force ceiling(100/period). The default behavior for
// float to integer conversion is rounding to nearest

// How many memory requests can be waiting at once
`define NUM_MEM_TAGS 15

`define MEM_SIZE_IN_BYTES (64*1024)
`define MEM_64BIT_LINES (`MEM_SIZE_IN_BYTES/8)

typedef union packed {
  logic [7:0][7:0]  byte_level;
  logic [3:0][15:0] half_level;
  logic [1:0][31:0] word_level;
} EXAMPLE_CACHE_BLOCK;

typedef enum logic [1:0] {
  BYTE   = 2'h0,
  HALF   = 2'h1,
  WORD   = 2'h2,
  DOUBLE = 2'h3
} MEM_SIZE;

// Memory bus commands
typedef enum logic [1:0] {
  BUS_NONE  = 2'h0,
  BUS_LOAD  = 2'h1,
  BUS_STORE = 2'h2
} BUS_COMMAND;

///////////////////////////////
// ---- Exception Codes ---- //
///////////////////////////////

/**
 * Exception codes for when something goes wrong in the processor.
 * Note that we use HALTED_ON_WFI to signify the end of computation.
 * It's original meaning is to 'Wait For an Interrupt', but we generally
 * ignore interrupts in 470
 *
 * This mostly follows the RISC-V Privileged spec
 * except a few add-ons for our infrastructure
 * The majority of them won't be used, but it's good to know what they are
 */

typedef enum logic [3:0] {
  INST_ADDR_MISALIGN  = 4'h0,
  INST_ACCESS_FAULT   = 4'h1,
  ILLEGAL_INST        = 4'h2,
  BREAKPOINT          = 4'h3,
  LOAD_ADDR_MISALIGN  = 4'h4,
  LOAD_ACCESS_FAULT   = 4'h5,
  STORE_ADDR_MISALIGN = 4'h6,
  STORE_ACCESS_FAULT  = 4'h7,
  ECALL_U_MODE        = 4'h8,
  ECALL_S_MODE        = 4'h9,
  NO_ERROR            = 4'ha,  // a reserved code that we use to signal no errors
  ECALL_M_MODE        = 4'hb,
  INST_PAGE_FAULT     = 4'hc,
  LOAD_PAGE_FAULT     = 4'hd,
  HALTED_ON_WFI       = 4'he,  // 'Wait For Interrupt'. In 470, signifies the end of computation
  STORE_PAGE_FAULT    = 4'hf
} EXCEPTION_CODE;

///////////////////////////////////
// ---- Instruction Typedef ---- //
///////////////////////////////////

// from the RISC-V ISA spec
// TODO update to make architectural register sizes more dynamic
typedef union packed {
  logic [31:0] inst;
  struct packed {
    logic [6:0] funct7;
    logic [4:0] rs2;  // source register 2
    logic [4:0] rs1;  // source register 1
    logic [2:0] funct3;
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } r;  // register-to-register instructions
  struct packed {
    logic [11:0] imm;  // immediate value for calculating address
    logic [4:0] rs1;  // source register 1 (used as address base)
    logic [2:0] funct3;
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } i;  // immediate or load instructions
  struct packed {
    logic [6:0] off;  // offset[11:5] for calculating address
    logic [4:0] rs2;  // source register 2
    logic [4:0] rs1;  // source register 1 (used as address base)
    logic [2:0] funct3;
    logic [4:0] set;  // offset[4:0] for calculating address
    logic [6:0] opcode;
  } s;  // store instructions
  struct packed {
    logic       of;      // offset[12]
    logic [5:0] s;       // offset[10:5]
    logic [4:0] rs2;     // source register 2
    logic [4:0] rs1;     // source register 1
    logic [2:0] funct3;
    logic [3:0] et;      // offset[4:1]
    logic       f;       // offset[11]
    logic [6:0] opcode;
  } b;  // branch instructions
  struct packed {
    logic [19:0] imm;  // immediate value
    logic [4:0] rd;  // destination register
    logic [6:0] opcode;
  } u;  // upper-immediate instructions
  struct packed {
    logic       of;      // offset[20]
    logic [9:0] et;      // offset[10:1]
    logic       s;       // offset[11]
    logic [7:0] f;       // offset[19:12]
    logic [4:0] rd;      // destination register
    logic [6:0] opcode;
  } j;  // jump instructions

  // extensions for other instruction types
`ifdef ATOMIC_EXT
  struct packed {
    logic [4:0] funct5;
    logic       aq;
    logic       rl;
    logic [4:0] rs2;
    logic [4:0] rs1;
    logic [2:0] funct3;
    logic [4:0] rd;
    logic [6:0] opcode;
  } a;  // atomic instructions
`endif
`ifdef SYSTEM_EXT
  struct packed {
    logic [11:0] csr;
    logic [4:0]  rs1;
    logic [2:0]  funct3;
    logic [4:0]  rd;
    logic [6:0]  opcode;
  } sys;  // system call instructions
`endif

} INST;  // instruction typedef, this should cover all types of instructions

////////////////////////////////////////
// ---- Datapath Control Signals ---- //
////////////////////////////////////////

// ALU opA input mux selects
typedef enum logic [1:0] {
  OPA_IS_RS1  = 2'h0,
  OPA_IS_NPC  = 2'h1,
  OPA_IS_PC   = 2'h2,
  OPA_IS_ZERO = 2'h3
} ALU_OPA_SELECT;

// ALU opB input mux selects
typedef enum logic [3:0] {
  OPB_IS_RS2   = 4'h0,
  OPB_IS_I_IMM = 4'h1,
  OPB_IS_S_IMM = 4'h2,
  OPB_IS_B_IMM = 4'h3,
  OPB_IS_U_IMM = 4'h4,
  OPB_IS_J_IMM = 4'h5
} ALU_OPB_SELECT;

// ALU function code input
// probably want to leave these alone
typedef enum logic [4:0] {
  ALU_ADD    = 5'h00,
  ALU_SUB    = 5'h01,
  ALU_SLT    = 5'h02,
  ALU_SLTU   = 5'h03,
  ALU_AND    = 5'h04,
  ALU_OR     = 5'h05,
  ALU_XOR    = 5'h06,
  ALU_SLL    = 5'h07,
  ALU_SRL    = 5'h08,
  ALU_SRA    = 5'h09,
  ALU_MUL    = 5'h0a,  // Mult FU
  ALU_MULH   = 5'h0b,  // Mult FU
  ALU_MULHSU = 5'h0c,  // Mult FU
  ALU_MULHU  = 5'h0d,  // Mult FU
  ALU_DIV    = 5'h0e,  // unused
  ALU_DIVU   = 5'h0f,  // unused
  ALU_REM    = 5'h10,  // unused
  ALU_REMU   = 5'h11   // unused
} ALU_FUNC;

////////////////////////////////
// ---- Datapath Packets ---- //
////////////////////////////////

typedef enum logic [2:0] {
  ALU = 3'b000,
  MULT = 3'b001,
  LOAD = 3'b010,
  STORE = 3'b011,
  BRANCH = 3'b100
} FUNIT;

/**
 * Packets are used to move many variables between modules with
 * just one datatype, but can be cumbersome in some circumstances.
 *
 * Define new ones in project 4 at your own discretion
 */

/**
 * IF_ID Packet:
 * Data exchanged from the IF to the ID stage
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;    // PC + 4
  logic             valid;
} IF_ID_PACKET;


typedef struct packed {
  logic [`PHYS_REG_IDX_SZ:0] reg_num;
  logic ready;
} PREG;

import "DPI-C" function void print_inst(
  int inst,
  int pc,
  int valid_inst
);
import "DPI-C" function void debug_print(string str);
import "DPI-C" function void debug_prinln(string str);

function void print_preg(PREG preg);
  if (preg.ready) $write("%0d+", preg.reg_num);
  else $write("%0d-", preg.reg_num);
endfunction


/**
 * ID_IS Packet:
 * Data from the ID stage to the IS stage AND 
 * packed stored in RS while waiting to issue
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)

  /* Includes 'ready' bit and index*/
  PREG dest_reg;
  PREG src1_reg;
  PREG src2_reg;

  ALU_FUNC alu_func;       // ALU function select (ALU_xxx *)
  logic    rd_mem;         // Does inst read memory?
  logic    wr_mem;         // Does inst write memory?
  logic    cond_branch;    // Is inst a conditional branch?
  logic    uncond_branch;  // Is inst an unconditional branch?
  logic    halt;           // Is this a halt?
  logic    illegal;        // Is this instruction illegal?
  logic    csr_op;         // Is this a CSR operation? (we use this to get return code)

  FUNIT function_type;
  logic valid;

  logic [$clog2(`ROB_SZ)-1:0] rob_index;
  logic has_dest;

  logic [`MAX_FU_INDEX-1:0] issued_fu_index;

  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access

} ID_IS_PACKET;

ID_IS_PACKET INVALID_ID_IS_PACKET = {
  `NOP,  // inst we can't simply assign 0 because NOP is non-zero

  {`XLEN{1'b0}},  // PC
  {`XLEN{1'b0}},  // NPC

  OPA_IS_RS1,  // opa_select
  OPB_IS_RS2,  // opb_select

  // TODO these sizes could be wrong
  {`PHYS_REG_IDX_SZ + 1{1'b0}},  // dest_reg
  {`PHYS_REG_IDX_SZ + 1{1'b0}},  // src1_reg
  {`PHYS_REG_IDX_SZ + 1{1'b0}},  // src2_reg

  ALU_ADD,  // alu_func
  1'b0,  // rd_mem
  1'b0,  // wr_mem
  1'b0,  // cond_branch
  1'b0,  // uncond_branch
  1'b0,  // halt
  1'b0,  // illegal
  1'b0,  // csr_op

  ALU,  // function_type
  1'b0,  // valid

  {$clog2(`ROB_SZ) - 1{1'b0}},  // rob_index
  1'b0,  // has_dest

  {`MAX_FU_INDEX - 1{1'b0}},  // issued_fu_index

  {`REG_IDX_SZ{1'b0}}  // arch_dest_reg_num
};


/**
 * ID_IS Packet:
 * Data exchanged from the ID to the IS stage
 */
typedef struct packed {
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)

  logic [`XLEN-1:0] rs1_value;  // reg A value
  logic [`XLEN-1:0] rs2_value;  // reg B value

  logic [`PHYS_REG_IDX_SZ:0] dest_reg_idx;  // destination (writeback) register index

  ALU_FUNC alu_func;       // ALU function select (ALU_xxx *)
  logic    rd_mem;         // Does inst read memory?
  logic    wr_mem;         // Does inst write memory?
  logic    cond_branch;    // Is inst a conditional branch?
  logic    uncond_branch;  // Is inst an unconditional branch?
  logic    halt;           // Is this a halt?
  logic    illegal;        // Is this instruction illegal?
  logic    csr_op;         // Is this a CSR operation? (we use this to get return code)

  FUNIT function_type;
  logic valid;

  logic [$clog2(`ROB_SZ)-1:0] rob_index;
  logic has_dest;

  logic [`MAX_FU_INDEX-1:0] issued_fu_index;

  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access

} IS_EX_PACKET;

IS_EX_PACKET INVALID_IS_EX_PACKET = {
  `NOP,  // inst we can't simply assign 0 because NOP is non-zero

  {`XLEN{1'b0}},  // PC
  {`XLEN{1'b0}},  // NPC

  OPA_IS_RS1,  // opa_select
  OPB_IS_RS2,  // opb_select

  {`XLEN{1'b0}},  // rs1_value
  {`XLEN{1'b0}},  // rs2_value

  {`PHYS_REG_IDX_SZ{1'b0}},  // dest_reg_idx

  ALU_ADD,  // alu_func
  1'b0,  // rd_mem
  1'b0,  // wr_mem
  1'b0,  // cond_branch
  1'b0,  // uncond_branch
  1'b0,  // halt
  1'b0,  // illegal
  1'b0,  // csr_op

  ALU,  // function_type
  1'b0,  // valid

  {$clog2(`ROB_SZ) - 1{1'b0}},  // rob_index
  1'b0,  // has_dest

  {`MAX_FU_INDEX - 1{1'b0}},  // issued_fu_index

  {`REG_IDX_SZ{1'b0}}  // arch_dest_reg_num
};

typedef struct packed {
  // Mostly pass through (TODO a lot is not needed)
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)

  logic [`XLEN-1:0] rs1_value;  // reg A value
  logic [`XLEN-1:0] rs2_value;  // reg B value

  logic [`PHYS_REG_IDX_SZ:0] dest_reg_idx;  // destination (writeback) register index

  ALU_FUNC alu_func;       // ALU function select (ALU_xxx *)
  logic    rd_mem;         // Does inst read memory?
  logic    wr_mem;         // Does inst write memory?
  logic    cond_branch;    // Is inst a conditional branch?
  logic    uncond_branch;  // Is inst an unconditional branch?
  logic    halt;           // Is this a halt?
  logic    illegal;        // Is this instruction illegal?
  logic    csr_op;         // Is this a CSR operation? (we use this to get return code)

  FUNIT function_type;
  logic valid;

  logic [$clog2(`ROB_SZ)-1:0] rob_index;
  logic has_dest;

  logic [`MAX_FU_INDEX-1:0] issued_fu_index; // TODO name doesn't make sense anymore, why 'issued'?

  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access

  // New stuff from EX stage
  logic [`XLEN-1:0] result;
  logic             take_branch;
  MEM_SIZE          mem_size;

  logic [63:0] prev_dword;  // Only for stores

} EX_CO_PACKET;


EX_CO_PACKET INVALID_EX_CO_PACKET = {
  `NOP,  // inst we can't simply assign 0 because NOP is non-zero

  {`XLEN{1'b0}},  // PC
  {`XLEN{1'b0}},  // NPC

  OPA_IS_RS1,  // opa_select
  OPB_IS_RS2,  // opb_select

  {`XLEN{1'b0}},  // rs1_value
  {`XLEN{1'b0}},  // rs2_value

  {`PHYS_REG_IDX_SZ{1'b0}},  // dest_reg_idx

  ALU_ADD,  // alu_func
  1'b0,  // rd_mem
  1'b0,  // wr_mem
  1'b0,  // cond_branch
  1'b0,  // uncond_branch
  1'b0,  // halt
  1'b0,  // illegal
  1'b0,  // csr_op

  ALU,  // function_type
  1'b0,  // valid

  {$clog2(`ROB_SZ) - 1{1'b0}},  // rob_index
  1'b0,  // has_dest

  {`MAX_FU_INDEX - 1{1'b0}},  // issued_fu_index

  {`REG_IDX_SZ{1'b0}},  // arch_dest_reg_num

  {`XLEN{1'b0}},  // result
  1'b0,  // take_branch
  1'b0,  // mem_size

  {64{1'b0}}  // prev_dword
};

typedef struct packed {
  // Mostly pass through (TODO a lot is not needed)
  INST              inst;
  logic [`XLEN-1:0] PC;
  logic [`XLEN-1:0] NPC;   // PC + 4

  ALU_OPA_SELECT opa_select;  // ALU opa mux select (ALU_OPA_xxx *)
  ALU_OPB_SELECT opb_select;  // ALU opb mux select (ALU_OPB_xxx *)

  logic [`XLEN-1:0] rs1_value;  // reg A value
  logic [`XLEN-1:0] rs2_value;  // reg B value

  logic [`PHYS_REG_IDX_SZ:0] dest_reg_idx;  // destination (writeback) register index

  ALU_FUNC alu_func;       // ALU function select (ALU_xxx *)
  logic    rd_mem;         // Does inst read memory?
  logic    wr_mem;         // Does inst write memory?
  logic    cond_branch;    // Is inst a conditional branch?
  logic    uncond_branch;  // Is inst an unconditional branch?
  logic    halt;           // Is this a halt?
  logic    illegal;        // Is this instruction illegal?
  logic    csr_op;         // Is this a CSR operation? (we use this to get return code)

  FUNIT function_type;
  logic valid;

  logic [$clog2(`ROB_SZ)-1:0] rob_index;
  logic has_dest;

  logic [`MAX_FU_INDEX-1:0] issued_fu_index; // TODO name doesn't make sense anymore, why 'issued'?


  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access

  // stuff from EX stage
  logic [`XLEN-1:0] result;
  logic take_branch;

  // new stuff from complete stage
  logic                        regfile_en;
  logic [`PHYS_REG_IDX_SZ-1:0] regfile_idx;
  logic [`XLEN-1:0]            regfile_data;
  MEM_SIZE                     mem_size;

  logic [63:0] prev_dword;  // Only for stores
} CO_RE_PACKET;


CO_RE_PACKET INVALID_CO_RE_PACKET = {
  `NOP,  // inst we can't simply assign 0 because NOP is non-zero

  {`XLEN{1'b0}},  // PC
  {`XLEN{1'b0}},  // NPC

  OPA_IS_RS1,  // opa_select
  OPB_IS_RS2,  // opb_select

  {`XLEN{1'b0}},  // rs1_value
  {`XLEN{1'b0}},  // rs2_value

  {`PHYS_REG_IDX_SZ{1'b0}},  // dest_reg_idx

  ALU_ADD,  // alu_func
  1'b0,  // rd_mem
  1'b0,  // wr_mem
  1'b0,  // cond_branch
  1'b0,  // uncond_branch
  1'b0,  // halt
  1'b0,  // illegal
  1'b0,  // csr_op

  ALU,  // function_type
  1'b0,  // valid

  {$clog2(`ROB_SZ) - 1{1'b0}},  // rob_index
  1'b0,  // has_dest

  {`MAX_FU_INDEX - 1{1'b0}},  // issued_fu_index

  {`REG_IDX_SZ{1'b0}},  // arch_dest_reg_num

  {`XLEN{1'b0}},  // result
  1'b0,  // take_branch

  1'b0,  // regfile_en
  {`PHYS_REG_IDX_SZ{1'b0}},  // regfile_idx
  {`XLEN{1'b0}},  // regfile_data
  1'b0,  // mem_size

  {64{1'b0}}  // prev_dword
};

/**
 * EX_MEM Packet:
 * Data exchanged from the EX to the MEM stage
 */
typedef struct packed {
  logic [`XLEN-1:0] alu_result;
  logic [`XLEN-1:0] NPC;

  logic                      take_branch;   // Is this a taken branch?
  // Pass-through from decode stage
  logic [`XLEN-1:0]          rs2_value;
  logic                      rd_mem;
  logic                      wr_mem;
  logic [`PHYS_REG_IDX_SZ:0] dest_reg_idx;
  logic                      halt;
  logic                      illegal;
  logic                      csr_op;
  logic                      rd_unsigned;   // Whether proc2Dmem_data is signed or unsigned
  MEM_SIZE                   mem_size;
  logic                      valid;


  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access
} EX_MEM_PACKET;

/**
 * MEM_WB Packet:
 * Data exchanged from the MEM to the WB stage
 *
 * Does not include data sent from the MEM stage to memory
 */
typedef struct packed {
  logic [`XLEN-1:0]          result;
  logic [`XLEN-1:0]          NPC;
  logic [`PHYS_REG_IDX_SZ:0] dest_reg_idx;  // writeback destination (ZERO_REG if no writeback)
  logic                      take_branch;
  logic                      halt;          // not used by wb stage
  logic                      illegal;       // not used by wb stage
  logic                      valid;

  logic [`REG_IDX_SZ:0] arch_dest_reg_num;  // purely for final wb output, to avoid map table access
} MEM_WB_PACKET;

// An entry in the reservation station, with all information about an instruction
// to be issued
typedef struct packed {
  ID_IS_PACKET packet;
  logic busy;
  logic issued;
} RS_ENTRY;

typedef struct packed {
  PREG T, Told;
  INST inst;
  logic [`XLEN-1:0] PC;
  // logic done;
} ROB_ENTRY;

/**
 * Branch Target Buffer:
 * 
 * TODO ADD DESCRIPTION
 * 
 */

// entries are written to using the write_entry struct

`define BTB_TAG_LEN 4
`define BTB_ENTRIES 16 // 2^`BTB_TAG_LEN

// entries are accessed using index
typedef struct packed {
  logic [`XLEN-1:0] target_pc;
  logic valid;
} BTB_ENTRY;

/**
 * Branch Predictor:
 * 
 * TODO ADD DESCRIPTION
 * 
 */

`define BP_TAG_LEN 4
`define BP_ENTRIES 16 // 2^`BP_TAG_LEN

typedef enum logic [1:0] {
  SNT = 2'b00,  // strongly not taken
  WNT = 2'b01,  // weakly not taken
  WT  = 2'b11,  // weakly taken
  ST  = 2'b10   // strongly taken
} BP_ENTRY_STATE;

typedef struct packed {
  BP_ENTRY_STATE state;
  logic valid;
} BP_ENTRY;


typedef struct packed {
  INST                       inst;
  logic [3:0]                completed_insts;
  logic [`XLEN-1:0]          PC;
  logic [`XLEN-1:0]          NPC;
  EXCEPTION_CODE             error_status;
  logic                      regfile_en;       // register write enable
  logic [`PHYS_REG_IDX_SZ:0] regfile_idx;      // register write index
  logic [`XLEN-1:0]          regfile_data;     // register write data 
  logic                      valid;
  FUNIT                      function_type;
  MEM_SIZE                   mem_size;
  logic [`XLEN-1:0]          rs2_value;
  logic [`XLEN-1:0]          result;

  logic [`REG_IDX_SZ:0] arch_dest_reg_num;

  logic [63:0] prev_dword;  // Only for stores
} RETIRE_ENTRY;


`endif  // __SYS_DEFS_SVH__
