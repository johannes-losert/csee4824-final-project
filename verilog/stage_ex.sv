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
            //out_packet <= 0;
        end
    end

endmodule // alu

module branch_calculation (
    input clock, 
    input reset,
    input [`XLEN-1:0] opa,
    input [`XLEN-1:0] opb,
    input [`XLEN-1:0] rs1,  // Value to check against condition
    input [`XLEN-1:0] rs2,
    input ALU_FUNC    alu_func,
    input [2:0] func,
    input logic       branch_en,
    input IS_EX_PACKET in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_BRANCH]   branch_done,

    output IS_EX_PACKET out_packet
);

    logic [`XLEN-1:0] branch_opa;
    logic [`XLEN-1:0] branch_opb;
    logic [`XLEN-1:0] branch_rs1;
    logic [`XLEN-1:0] branch_rs2;
    ALU_FUNC b_alu_func; 
    logic [2:0] branch_func;
    IS_EX_PACKET branch_packet;

    assign signed_opa   = branch_opa;
    assign signed_opb   = branch_opb;
    assign signed_rs1 = branch_rs1;
    assign signed_rs2 = branch_rs2;

    always_comb begin
        case (alu_func)
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

    /*always_comb begin
        case (func)
            3'b000:  take = signed_rs1 == signed_rs2; // BEQ
            3'b001:  take = signed_rs1 != signed_rs2; // BNE
            3'b100:  take = signed_rs1 < signed_rs2;  // BLT
            3'b101:  take = signed_rs1 >= signed_rs2; // BGE
            3'b110:  take = branch_rs1 < branch_rs2;                // BLTU
            3'b111:  take = branch_rs1 >= branch_rs2;               // BGEU
            default: take = `FALSE;
        endcase
    end*/

    always_ff @(posedge clock) begin
        if(reset) begin
            branch_done[0] <= 1'b0;
            out_packet <= 0;
            branch_opa <= 0;
            branch_opb <= 0;
            branch_rs1 <= 0;
            branch_rs2 <= 0;
            b_alu_func <= 0;
            branch_func <= 0;
            branch_packet <= 0;
        end else if (branch_en) begin
            branch_done[0] <= 1'b1;
            out_packet <= in_packet;
            branch_opa <= opa;
            branch_opb <= opb;
            branch_rs1 <= rs1;
            branch_rs2 <= rs2;
            b_alu_func <= alu_func;
            branch_func <= func;
            branch_packet <= in_packet;
        end else begin
            branch_done[0] <= 1'b0;
            branch_opa <= branch_opa;
            branch_opb <= branch_opb;
            branch_rs1 <= branch_rs1;
            branch_rs2 <= branch_rs2;
            //out_packet <= 0;
        end
    end

endmodule

// Conditional branch module: compute whether to take conditional branches
// This module is purely combinational
module conditional_branch (
    input clock,
    input reset,
    input [2:0]       func, // Specifies which condition to check
    input [`XLEN-1:0] rs1,  // Value to check against condition
    input [`XLEN-1:0] rs2,
    input logic cond_en,

    output logic take // True/False condition result
);

    logic [`XLEN-1:0] cond_rs1;
    logic [`XLEN-1:0] cond_rs2;

    logic signed [`XLEN-1:0] signed_rs1, signed_rs2;
    assign signed_rs1 = cond_rs1;
    assign signed_rs2 = cond_rs2;
    always_comb begin
        case (func)
            3'b000:  take = signed_rs1 == signed_rs2; // BEQ
            3'b001:  take = signed_rs1 != signed_rs2; // BNE
            3'b100:  take = signed_rs1 < signed_rs2;  // BLT
            3'b101:  take = signed_rs1 >= signed_rs2; // BGE
            3'b110:  take = cond_rs1 < cond_rs2;                // BLTU
            3'b111:  take = cond_rs1 >= cond_rs2;               // BGEU
            default: take = `FALSE;
        endcase
    end

    always_ff @(posedge clock) begin
        if (reset) begin
            cond_rs1 <= 0;
            cond_rs2 <= 0;
        end else if (cond_en) begin
            cond_rs1 <= rs1; 
            cond_rs2 <= rs2;
        end
    end

endmodule // conditional_branch


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
    output logic [`NUM_FU_ALU-1:0]    free_alu,
    output logic [`NUM_FU_MULT-1:0]   free_mult,
    output logic [`NUM_FU_LOAD-1:0]   free_load,
    output logic [`NUM_FU_STORE-1:0]  free_store,
    output logic [`NUM_FU_BRANCH-1:0] free_branch,
    output logic take_conditional,

    // debug outputs
    output IS_EX_PACKET tmp_alu_packet,
    output IS_EX_PACKET tmp_mult_packet,
    output IS_EX_PACKET tmp_branch_packet
);

    logic [`XLEN-1:0] alu_result;
    logic [`XLEN-1:0] mult_result;
    logic [`XLEN-1:0] branch_result;

    logic [`XLEN-1:0] opa_mux_out, opb_mux_out;
    logic take_conditional;

    // ultimate "take branch" signal:
    // unconditional, or conditional and the condition is true
    // assign ex_packet.take_branch = is_ex_reg.uncond_branch || (is_ex_reg.cond_branch && take_conditional);

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

    logic alu_done; 
    logic mult_done; 
    logic branch_done;

    logic [`XLEN-1:0] tmp_alu_result; 
    logic [`XLEN-1:0] tmp_mult_result;
    logic [`XLEN-1:0] tmp_branch_result;

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
        .result(alu_result),
        .alu_done(alu_done),
        .out_packet(tmp_alu_packet)
    );

    branch_calculation branch_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .opa(opa_mux_out),
        .opb(opb_mux_out),
        .rs1(is_ex_reg.rs1_value),
        .rs2(is_ex_reg.rs2_value),
        .alu_func(is_ex_reg.alu_func),
        .func(is_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .branch_en(is_ex_reg.function_type == BRANCH && branch_en && issue_fu_index == 0),
        .in_packet(is_ex_reg),

        // Output
        .result(branch_result),
        .branch_done(branch_done),

        .out_packet(tmp_branch_packet)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        .clock(clock),
        .reset(reset),
        .func(is_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .rs1(is_ex_reg.rs1_value),
        .rs2(is_ex_reg.rs2_value),
        .cond_en(is_ex_reg.function_type == BRANCH && branch_en && issue_fu_index == 0),

        // Output
        .take(take_conditional)
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
        .mult_done(mult_done),
        .out_packet(tmp_mult_packet)
    );

    logic [5:0] waiting_fus; 
    logic alu_done_process;
    logic mult_done_process;
    logic branch_done_process;
    logic tmp_take_conditional;

    always_comb begin
        if(alu_done_process) begin
            ex_packet.result = tmp_alu_result;
            ex_packet.NPC = tmp_alu_packet.NPC;
            ex_packet.take_branch = 0;

            ex_packet.rs2_value = tmp_alu_packet.rs2_value;
            ex_packet.rd_mem = tmp_alu_packet.rd_mem;
            ex_packet.wr_mem = tmp_alu_packet.wr_mem;
            ex_packet.dest_reg_idx = tmp_alu_packet.dest_reg_idx;
            ex_packet.halt = tmp_alu_packet.halt;
            ex_packet.illegal = tmp_alu_packet.illegal;
            ex_packet.csr_op = tmp_alu_packet.csr_op;
            ex_packet.rd_unsigned  = tmp_alu_packet.inst.r.funct3[2];
            ex_packet.mem_size     = MEM_SIZE'(tmp_alu_packet.inst.r.funct3[1:0]);
            ex_packet.valid = tmp_alu_packet.valid;
            ex_packet.rob_index = tmp_alu_packet.rob_index;
        end else if (mult_done_process) begin
            ex_packet.result = tmp_mult_result;
            ex_packet.NPC = tmp_mult_packet.NPC;
            ex_packet.take_branch = 0;

            ex_packet.rs2_value = tmp_mult_packet.rs2_value;
            ex_packet.rd_mem = tmp_mult_packet.rd_mem;
            ex_packet.wr_mem = tmp_mult_packet.wr_mem;
            ex_packet.dest_reg_idx = tmp_mult_packet.dest_reg_idx;
            ex_packet.halt = tmp_mult_packet.halt;
            ex_packet.illegal = tmp_mult_packet.illegal;
            ex_packet.csr_op = tmp_mult_packet.csr_op;
            ex_packet.rd_unsigned  = tmp_mult_packet.inst.r.funct3[2];
            ex_packet.mem_size     = MEM_SIZE'(tmp_mult_packet.inst.r.funct3[1:0]);
            ex_packet.valid = tmp_mult_packet.valid;
            ex_packet.rob_index = tmp_mult_packet.rob_index;
        end else if (branch_done_process) begin
            ex_packet.result = tmp_branch_result;
            ex_packet.NPC = tmp_branch_packet.NPC;
            ex_packet.take_branch = tmp_branch_packet.uncond_branch || (tmp_branch_packet.cond_branch && tmp_take_conditional);

            ex_packet.rs2_value = tmp_branch_packet.rs2_value;
            ex_packet.rd_mem = tmp_branch_packet.rd_mem;
            ex_packet.wr_mem = tmp_branch_packet.wr_mem;
            ex_packet.dest_reg_idx = tmp_branch_packet.dest_reg_idx;
            ex_packet.halt = tmp_branch_packet.halt;
            ex_packet.illegal = tmp_branch_packet.illegal;
            ex_packet.csr_op = tmp_branch_packet.csr_op;
            ex_packet.rd_unsigned  = tmp_mult_packet.inst.r.funct3[2];
            ex_packet.mem_size     = MEM_SIZE'(tmp_branch_packet.inst.r.funct3[1:0]);
            ex_packet.valid = tmp_branch_packet.valid;
            ex_packet.rob_index = tmp_branch_packet.rob_index;
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            waiting_fus <= 0;
            tmp_alu_result <= 0;
            tmp_mult_result <= 0;
            tmp_branch_result <= 0;
            alu_done_process <= 0;
            mult_done_process <= 0;
            branch_done_process <= 0;
            tmp_take_conditional <= 0;
        end
        else begin
            if(alu_done) begin
                waiting_fus[ALU] <= 1;
                tmp_alu_result <= alu_result;
            end 
            if(mult_done) begin
                waiting_fus[MULT] <= 1; 
                tmp_mult_result <= mult_result;
            end
            if(branch_done) begin
                waiting_fus[BRANCH] <= 1; 
                tmp_branch_result <= branch_result;
                tmp_take_conditional <= take_conditional;
            end 

            // Defaults
            alu_done_process <= 0;
            mult_done_process <= 0;
            branch_done_process <= 0;
            free_alu[0] <= 0;
            free_mult[0] <= 0;
            free_branch[0] <= 0;

            if(waiting_fus[ALU] == 1) begin
                waiting_fus[ALU] <= 0;
                alu_done_process <= 1; 
                free_alu[0] <= 1;
            end else if(waiting_fus[MULT] == 1) begin
                waiting_fus[MULT] <= 0;
                mult_done_process <= 1;
                free_mult[0] <= 1;
            end else if(waiting_fus[BRANCH] == 1) begin
                waiting_fus[BRANCH] <= 0;
                branch_done_process <= 1;
                free_branch[0] <= 1;
            end
            
            
        end
    end     

    

endmodule // stage_ex
