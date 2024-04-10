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
// TODO: remove multiplication logic, or ensure that the module only runs if alu_en is set
module alu (
    input               clock, 
    input               reset,
    input [`XLEN-1:0]   opa,
    input [`XLEN-1:0]   opb,
    input ALU_FUNC      func,
    input logic         alu_en,
    input IS_EX_PACKET in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_ALU-1:0]  alu_done,
    output IS_EX_PACKET out_packet
);

    logic [`XLEN-1:0] alu_opa;
    logic [`XLEN-1:0] alu_opb;
    ALU_FUNC alu_func; 
    IS_EX_PACKET alu_packet;

    logic signed [`XLEN-1:0]   signed_opa, signed_opb;

    assign signed_opa   = alu_opa;
    assign signed_opb   = alu_opb;

    always_comb begin
        case (func)
            ALU_ADD:    result = alu_opa + alu_opb;
            ALU_SUB:    result = alu_opa - alu_opb;
            ALU_AND:    result = alu_opa & alu_opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = alu_opa < alu_opb;
            ALU_OR:     result = alu_opa | alu_opb;
            ALU_XOR:    result = alu_opa ^ alu_opb;
            ALU_SRL:    result = alu_opa >> alu_opb[4:0];
            ALU_SLL:    result = alu_opa << alu_opb[4:0];
            ALU_SRA:    result = signed_opa >>> alu_opb[4:0]; // arithmetic from logical shift

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            alu_done[0] <= 1'b0;
            out_packet <= 0;
            alu_opa <= 0;
            alu_opb <= 0;
            alu_func <= 0;
            alu_packet <= 0;
        end else if (alu_en) begin
            alu_done[0] <= 1'b1;
            out_packet <= in_packet;
            alu_opa <= opa;
            alu_opb <= opb;
            alu_func <= func;
            alu_packet <= in_packet;
        end else begin
            alu_done[0] <= 1'b0;
            out_packet <= 0;
        end
    end

endmodule // alu

module branch_calculation (
    input clock, 
    input reset,
    input [`XLEN-1:0] opa,
    input [`XLEN-1:0] opb,
    input ALU_FUNC    func,
    input logic       branch_en,
    input IS_EX_PACKET in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_BRANCH]   branch_done,
    output IS_EX_PACKET out_packet
);

    logic [`XLEN-1:0] branch_opa;
    logic [`XLEN-1:0] branch_opb;
    ALU_FUNC branch_func; 
    IS_EX_PACKET branch_packet;

    assign signed_opa   = branch_opa;
    assign signed_opb   = branch_opb;

    always_comb begin
        case (func)
            ALU_ADD:    result = branch_opa + branch_opb;
            ALU_SUB:    result = branch_opa - branch_opb;
            ALU_AND:    result = branch_opa & branch_opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = branch_opa < branch_opb;
            ALU_OR:     result = branch_opa | branch_opb;
            ALU_XOR:    result = branch_opa ^ branch_opb;
            ALU_SRL:    result = branch_opa >> branch_opb[4:0];
            ALU_SLL:    result = branch_opa << branch_opb[4:0];
            ALU_SRA:    result = signed_opa >>> branch_opb[4:0]; // arithmetic from logical shift

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            branch_done[0] <= 1'b0;
            out_packet <= 0;
            branch_opa <= 0;
            branch_opb <= 0;
            branch_func <= 0;
            branch_packet <= 0;
        end else if (branch_en) begin
            branch_done[0] <= 1'b1;
            out_packet <= in_packet;
            branch_opa <= opa;
            branch_opb <= opb;
            branch_func <= func;
            branch_packet <= in_packet;
        end else begin
            branch_done[0] <= 1'b0;
            out_packet <= 0;
        end
    end

endmodule

module multiply (
    input clock,
    input reset,
    input [`XLEN-1:0] mcand,
    input [`XLEN-1:0] mplier,
    input ALU_FUNC          func,
    input logic       mult_en,
    input IS_EX_PACKET      in_packet,

    output logic [`XLEN-1:0]         product,
    output logic [`NUM_FU_MULT-1:0]  mult_done,
    output IS_EX_PACKET out_packet
);

    logic [`XLEN-1:0] mult_mcand; 
    logic [`XLEN-1:0] mult_mplier;
    ALU_FUNC mult_func;
    IS_EX_PACKET mult_packet;
    logic tmp_mult_en;

    logic signed_input1;
    logic signed_input2; 
    //logic done; 

    //assign mult_done[0] = done;

    // Determine signedness based on which multiplication function is used
    always_comb begin
        case (mult_func)
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
        .mcand          ({32'b0, mult_mcand}),
        .mplier         ({32'b0, mult_mplier}),
        .signed_input1  (signed_input1),
        .signed_input2  (signed_input2),
        .func           (mult_func),
        .start          (tmp_mult_en),
        .product        (product),
        .done           (mult_done[0])
    );

    always_ff @(posedge clock) begin
        if (reset) begin
            mult_mcand <= 0;
            mult_mplier <= 0;
            mult_func <= 0;
            mult_packet <= 0;
            tmp_mult_en <= 0;
            out_packet <= 0;
        end else if (mult_en) begin
            mult_mcand <= mcand;
            mult_mplier <= mplier;
            mult_func <= func; 
            mult_packet <= in_packet;
            tmp_mult_en <= 1;
            out_packet <= in_packet;
        end else begin
            tmp_mult_en <= 0;
        end
    end

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
    // Test cases work with these inputs and outputs
    /* input                               clock,
    input                               reset,
    input ID_EX_PACKET                  is_ex_reg,
    input FUNIT                         funit,
    input                               mult_en,
    input                               alu_en,
    input                               branch_en,
    input logic [`MAX_FU_INDEX-1:0]     issue_fu_index,

    output EX_MEM_PACKET        ex_packet,
    output [`XLEN-1:0]          mult_result,
    output [`XLEN-1:0]          branch_result,
    output [`NUM_FU_ALU-1:0]    free_alu,
    output [`NUM_FU_MULT-1:0]   free_mult,
    output [`NUM_FU_LOAD-1:0]   free_load,
    output [`NUM_FU_STORE-1:0]  free_store,
    output [`NUM_FU_BRANCH-1:0] free_branch*/ 

    input clock,
    input reset,
    input IS_EX_PACKET is_ex_reg,
    input                               mult_en,
    input                               alu_en,
    input                               branch_en,
    input logic [`MAX_FU_INDEX-1:0] issue_fu_index,

    output EX_CO_PACKET ex_packet,
    output [`XLEN-1:0]          mult_result,
    output [`XLEN-1:0]          branch_result,
    output [`NUM_FU_ALU-1:0]    free_alu,
    output [`NUM_FU_MULT-1:0]   free_mult,
    output [`NUM_FU_LOAD-1:0]   free_load,
    output [`NUM_FU_STORE-1:0]  free_store,
    output [`NUM_FU_BRANCH-1:0] free_branch,

    // debug outputs
    output IS_EX_PACKET tmp_alu_packet,
    output IS_EX_PACKET tmp_mult_packet,
    output IS_EX_PACKET tmp_branch_packet
);

    logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic take_conditional;

    /*IS_EX_PACKET tmp_alu_packet;
    IS_EX_PACKET tmp_mult_packet;
    IS_EX_PACKET tmp_branch_packet;*/

    // Pass-throughs
    assign ex_packet.NPC          = is_ex_reg.NPC;
    assign ex_packet.rs2_value    = is_ex_reg.rs2_value;
    assign ex_packet.rd_mem       = is_ex_reg.rd_mem;
    assign ex_packet.wr_mem       = is_ex_reg.wr_mem;
    assign ex_packet.dest_reg_idx = is_ex_reg.dest_reg_idx;
    assign ex_packet.halt         = is_ex_reg.halt;
    assign ex_packet.illegal      = is_ex_reg.illegal;
    assign ex_packet.csr_op       = is_ex_reg.csr_op;
    assign ex_packet.valid        = is_ex_reg.valid;

    // Break out the signed/unsigned bit and memory read/write size
    assign ex_packet.rd_unsigned  = is_ex_reg.inst.r.funct3[2]; // 1 if unsigned, 0 if signed
    assign ex_packet.mem_size     = MEM_SIZE'(is_ex_reg.inst.r.funct3[1:0]);

    // ultimate "take branch" signal:
    // unconditional, or conditional and the condition is true
    assign ex_packet.take_branch = is_ex_reg.uncond_branch || (is_ex_reg.cond_branch && take_conditional);

    // ALU opA mux
    always_comb begin
        case (is_ex_reg.opa_select)
            OPA_IS_RS1:  opa_mux_out = is_ex_reg.rs1_value;
            OPA_IS_NPC:  opa_mux_out = is_ex_reg.NPC;
            OPA_IS_PC:   opa_mux_out = is_ex_reg.PC;
            OPA_IS_ZERO: opa_mux_out = 0;
            default:     opa_mux_out = `XLEN'hdeadface; // dead face
        endcase
    end

    // ALU opB mux
    always_comb begin
        case (is_ex_reg.opb_select)
            OPB_IS_RS2:   opb_mux_out = is_ex_reg.rs2_value;
            OPB_IS_I_IMM: opb_mux_out = `RV32_signext_Iimm(is_ex_reg.inst);
            OPB_IS_S_IMM: opb_mux_out = `RV32_signext_Simm(is_ex_reg.inst);
            OPB_IS_B_IMM: opb_mux_out = `RV32_signext_Bimm(is_ex_reg.inst);
            OPB_IS_U_IMM: opb_mux_out = `RV32_signext_Uimm(is_ex_reg.inst);
            OPB_IS_J_IMM: opb_mux_out = `RV32_signext_Jimm(is_ex_reg.inst);
            default:      opb_mux_out = `XLEN'hfacefeed; // face feed
        endcase
    end

    always_comb begin

    end

    // Instantiate the ALU
    alu alu_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .func(is_ex_reg.alu_func),
        .alu_en(is_ex_reg.function_type == ALU && alu_en && issue_fu_index == 0),
        .in_packet(is_ex_reg),

        // Output
        .result(ex_packet.result),
        .alu_done(free_alu),
        .out_packet(tmp_alu_packet)
    );

    branch_calculation branch_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .func(is_ex_reg.alu_func),
        .branch_en(is_ex_reg.function_type == BRANCH && branch_en && issue_fu_index == 0),
        .in_packet(is_ex_reg),

        // Output
        .result(branch_result),
        .branch_done(free_branch),
        .out_packet(tmp_branch_packet)
    );

    // Instantiate multiply functional unit
    multiply mult_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .mcand(opa_mux_out),
        .mplier(opb_mux_out),
        .func(is_ex_reg.alu_func),
        .mult_en(is_ex_reg.function_type == MULT && mult_en && issue_fu_index == 0),
        .in_packet(is_ex_reg),

        // Output
        .product(mult_result),
        .mult_done(free_mult),
        .out_packet(tmp_mult_packet)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .func(is_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .rs1(is_ex_reg.rs1_value),
        .rs2(is_ex_reg.rs2_value),

        // Output
        .take(take_conditional)
    );

endmodule // stage_ex
