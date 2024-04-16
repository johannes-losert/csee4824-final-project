`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
// `include "verilog/mult.sv"

// ALU: computes the result of FUNC applied with operands A and B
module alu (
    input                           clock, 
    input                           reset,
    input [`XLEN-1:0]               opa,
    input [`XLEN-1:0]               opb,
    input ALU_FUNC                  func,
    input logic                     alu_en,
    input IS_EX_PACKET              in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_ALU-1:0]  alu_done,
    output IS_EX_PACKET             out_packet
);

    logic [`XLEN-1:0]           alu_opa, alu_opb;
    logic signed [`XLEN-1:0]    signed_opa, signed_opb;

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
            alu_done[0]     <= 1'b0;
            out_packet      <= 0;
            alu_opa         <= 0;
            alu_opb         <= 0;
        end else if (alu_en) begin
            alu_done[0]     <= 1'b1;
            out_packet      <= in_packet;
            alu_opa         <= opa;
            alu_opb         <= opb;
        end else begin
            alu_done[0]     <= 1'b0;
            alu_opa         <= alu_opa;
            alu_opb         <= alu_opb;
            out_packet      <= out_packet;
        end
    end

endmodule // alu

// For calculating branch addresses 
module branch_calculation (
    input               clock, 
    input               reset,
    input [`XLEN-1:0]   opa,
    input [`XLEN-1:0]   opb,
    input ALU_FUNC      alu_func,
    input logic         branch_en,
    input IS_EX_PACKET  in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_BRANCH]   branch_done,
    output IS_EX_PACKET             out_packet
);

    logic [`XLEN-1:0] branch_opa, branch_opb;

    assign signed_opa   = branch_opa;
    assign signed_opb   = branch_opb;

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

    always_ff @(posedge clock) begin
        if(reset) begin
            branch_done[0]  <= 1'b0;
            out_packet      <= 0;
            branch_opa      <= 0;
            branch_opb      <= 0;
        end else if (branch_en) begin
            branch_done[0]  <= 1'b1;
            out_packet      <= in_packet;
            branch_opa      <= opa;
            branch_opb      <= opb;
        end else begin
            branch_done[0]  <= 1'b0;
            branch_opa      <= branch_opa;
            branch_opb      <= branch_opb;
            out_packet      <= out_packet;
        end
    end

endmodule

// Conditional branch module: compute whether to take conditional branches
module conditional_branch (
    input               clock,
    input               reset,
    input [2:0]         func, // Specifies which condition to check
    input [`XLEN-1:0]   rs1,  // Value to check against condition
    input [`XLEN-1:0]   rs2,
    input logic         cond_en,

    output logic        take // True/False condition result
);

    logic [`XLEN-1:0]           cond_rs1, cond_rs2;
    logic signed [`XLEN-1:0]    signed_rs1, signed_rs2;

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
        end else begin
            cond_rs1 <= cond_rs1; 
            cond_rs2 <= cond_rs2;
        end
    end

endmodule // conditional_branch


module multiply (
    input                               clock,
    input                               reset,
    input [`XLEN-1:0]                   mcand,
    input [`XLEN-1:0]                   mplier,
    input ALU_FUNC                      func,
    input logic                         mult_en,
    input IS_EX_PACKET                  in_packet,

    output logic [`XLEN-1:0]            product,
    output logic [`NUM_FU_MULT-1:0]     mult_done,
    output IS_EX_PACKET                 out_packet
);

    logic [`XLEN-1:0]   mult_mcand, mult_mplier;
    ALU_FUNC            mult_func;
    logic               tmp_mult_en, signed_input1, signed_input2;

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
            mult_mcand  <= 0;
            mult_mplier <= 0;
            mult_func   <= 0;
            tmp_mult_en <= 0;
            out_packet  <= 0;
        end else if (mult_en) begin
            mult_mcand  <= mcand;
            mult_mplier <= mplier;
            mult_func   <= func; 
            tmp_mult_en <= 1;
            out_packet  <= in_packet;
        end else begin
            tmp_mult_en <= 0;
            mult_mcand  <= mult_mcand;
            mult_mplier <= mult_mplier;
            mult_func   <= mult_func;
            out_packet  <= out_packet;
        end
    end

endmodule