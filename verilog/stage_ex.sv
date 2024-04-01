/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  stage_ex.sv                                         //
//                                                                     //
//  Description :  instruction execute (EX) stage of the pipeline;     //
//                 given the instruction command code CMD, select the  //
//                 proper input A and B for the ALU, compute the       //
//                 result, and compute the condition for branches, and //
//                 pass all the results down the pipeline.             //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`include "verilog/mult.sv"

// ALU: computes the result of FUNC applied with operands A and B
// This module is purely combinational
module alu (
    input clock, 
    input reset,
    input [`XLEN-1:0] opa,
    input [`XLEN-1:0] opb,
    input ALU_FUNC    func,
    input logic       alu_en,

    output logic [`XLEN-1:0] result,
    output logic             alu_done
);

    logic signed [`XLEN-1:0]   signed_opa, signed_opb;
    logic signed [2*`XLEN-1:0] signed_mul, mixed_mul;
    logic        [2*`XLEN-1:0] unsigned_mul;

    assign signed_opa   = opa;
    assign signed_opb   = opb;

    // We let verilog do the full 32-bit multiplication for us.
    // This gives a large clock period.
    // You will replace this with your pipelined multiplier in project 4.
    assign signed_mul   = signed_opa * signed_opb;
    assign unsigned_mul = opa * opb;
    assign mixed_mul    = signed_opa * opb;

    always_comb begin
        case (func)
            ALU_ADD:    result = opa + opb;
            ALU_SUB:    result = opa - opb;
            ALU_AND:    result = opa & opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = opa < opb;
            ALU_OR:     result = opa | opb;
            ALU_XOR:    result = opa ^ opb;
            ALU_SRL:    result = opa >> opb[4:0];
            ALU_SLL:    result = opa << opb[4:0];
            ALU_SRA:    result = signed_opa >>> opb[4:0]; // arithmetic from logical shift
            ALU_MUL:    result = signed_mul[`XLEN-1:0];
            ALU_MULH:   result = signed_mul[2*`XLEN-1:`XLEN];
            ALU_MULHSU: result = mixed_mul[2*`XLEN-1:`XLEN];
            ALU_MULHU:  result = unsigned_mul[2*`XLEN-1:`XLEN];

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            alu_done = 1'b0;
        end else if (alu_en) begin
            alu_done = 1'b1;
        end else begin
            alu_done = 1'b0;
        end
    end

endmodule // alu

module multiply (
    input clock,
    input reset,
    input [`XLEN-1:0] mcand,
    input [`XLEN-1:0] mplier,
    ALU_FUNC          func,
    input logic       mult_en,

    output logic [`XLEN-1:0] product,
    output logic             mult_done
);

    logic signed_input1;
    logic signed_input2; 

    // Determine signedness based on which multiplication function is used
    always_comb begin
        case (func)
            ALU_MUL: begin
                signed_input1 = 1;
                signed_input2 = 1;
            end
            ALU_MULH: begin
                signed_input1 = 1;
                signed_input2 = 1;
            end
            ALU_MULHSU: begin
                signed_input1 = 1;
                signed_input2 = 0;
            end
            ALU_MULHU: begin
                signed_input1 = 0;
                signed_input2 = 0;
            end
            default: begin
                signed_input1 = 0;
                signed_input2 = 0;
            end
        endcase
    end

    // declaration of mult module
    mult mult_mod_0 (
        .clock          (clock),
        .reset          (reset),
        .mcand          ({32'b0, mcand}),
        .mplier         ({32'b0, mplier}),
        .signed_input1  (signed_input1),
        .signed_input2  (signed_input2),
        .func           (func),
        .start          (mult_en),
        .product        (product),
        .done           (mult_done)
    );

endmodule

// Conditional branch module: compute whether to take conditional branches
// This module is purely combinational
module conditional_branch (
    input [2:0]       func, // Specifies which condition to check
    input [`XLEN-1:0] rs1,  // Value to check against condition
    input [`XLEN-1:0] rs2,

    output logic take // True/False condition result
);

    logic signed [`XLEN-1:0] signed_rs1, signed_rs2;
    assign signed_rs1 = rs1;
    assign signed_rs2 = rs2;
    always_comb begin
        case (func)
            3'b000:  take = signed_rs1 == signed_rs2; // BEQ
            3'b001:  take = signed_rs1 != signed_rs2; // BNE
            3'b100:  take = signed_rs1 < signed_rs2;  // BLT
            3'b101:  take = signed_rs1 >= signed_rs2; // BGE
            3'b110:  take = rs1 < rs2;                // BLTU
            3'b111:  take = rs1 >= rs2;               // BGEU
            default: take = `FALSE;
        endcase
    end

endmodule // conditional_branch


module stage_ex (
    input clock,
    input reset,
    input ID_EX_PACKET id_ex_reg,
    input FUNIT funit,
    input mult_en,
    input alu_en,

    output EX_MEM_PACKET ex_packet,
    output [`XLEN-1:0] mult_result,
    output [`NUM_FU_ALU-1:0]free_alu,
    output [`NUM_FU_MULT-1:0]free_mult,
    output [`NUM_FU_LOAD-1:0]free_load,
    output [`NUM_FU_STORE-1:0]free_store
);

    logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic take_conditional;

    // Pass-throughs
    assign ex_packet.NPC          = id_ex_reg.NPC;
    assign ex_packet.rs2_value    = id_ex_reg.rs2_value;
    assign ex_packet.rd_mem       = id_ex_reg.rd_mem;
    assign ex_packet.wr_mem       = id_ex_reg.wr_mem;
    assign ex_packet.dest_reg_idx = id_ex_reg.dest_reg_idx;
    assign ex_packet.halt         = id_ex_reg.halt;
    assign ex_packet.illegal      = id_ex_reg.illegal;
    assign ex_packet.csr_op       = id_ex_reg.csr_op;
    assign ex_packet.valid        = id_ex_reg.valid;

    // Break out the signed/unsigned bit and memory read/write size
    assign ex_packet.rd_unsigned  = id_ex_reg.inst.r.funct3[2]; // 1 if unsigned, 0 if signed
    assign ex_packet.mem_size     = MEM_SIZE'(id_ex_reg.inst.r.funct3[1:0]);

    // ultimate "take branch" signal:
    // unconditional, or conditional and the condition is true
    assign ex_packet.take_branch = id_ex_reg.uncond_branch || (id_ex_reg.cond_branch && take_conditional);

    // ALU opA mux
    always_comb begin
        case (id_ex_reg.opa_select)
            OPA_IS_RS1:  opa_mux_out = id_ex_reg.rs1_value;
            OPA_IS_NPC:  opa_mux_out = id_ex_reg.NPC;
            OPA_IS_PC:   opa_mux_out = id_ex_reg.PC;
            OPA_IS_ZERO: opa_mux_out = 0;
            default:     opa_mux_out = `XLEN'hdeadface; // dead face
        endcase
    end

    // ALU opB mux
    always_comb begin
        case (id_ex_reg.opb_select)
            OPB_IS_RS2:   opb_mux_out = id_ex_reg.rs2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(id_ex_reg.inst);
            OPB_IS_S_IMM: opb_mux_out = `RV32_signext_Simm(id_ex_reg.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(id_ex_reg.inst);
            OPB_IS_U_IMM: opb_mux_out = `RV32_signext_Uimm(id_ex_reg.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(id_ex_reg.inst);
            default:      opb_mux_out = `XLEN'hfacefeed; // face feed
        endcase
    end

    // Instantiate the ALU
    alu alu_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .func(id_ex_reg.alu_func),
        .alu_en(funit == ALU && alu_en),

        // Output
        .result(ex_packet.alu_result),
        .alu_done(free_alu)
    );

    // Instantiate multiply functional unit
    multiply mult_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .mcand(opa_mux_out),
        .mplier(opb_mux_out),
        .func(id_ex_reg.alu_func),
        .mult_en(funit == MULT && mult_en),

        // Output
        .product(mult_result),
        .mult_done(free_mult)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .func(id_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .rs1(id_ex_reg.rs1_value),
        .rs2(id_ex_reg.rs2_value),

        // Output
        .take(take_conditional)
    );

endmodule // stage_ex
