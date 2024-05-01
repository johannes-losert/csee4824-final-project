`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
`include "verilog/load.sv"
// `include "verilog/mult.sv"

// ALU: computes the result of FUNC applied with operands A and B
module alu (
    //input                           clock, 
    //input                           reset,
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
    assign alu_done[0] = alu_en;
    assign alu_opa = opa;
    assign alu_opb = opb;
    assign out_packet = in_packet;

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

/*
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
*/
endmodule // alu

// For calculating branch addresses 
module branch_calculation (
    //input               clock, 
    //input               reset,
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
    assign branch_done[0] = branch_en;
    assign out_packet = in_packet;
    assign branch_opa = opa;
    assign branch_opb = opb;
/*
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
*/
endmodule

// Conditional branch module: compute whether to take conditional branches
module conditional_branch (
    //input               clock,
    //input               reset,
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

    assign cond_rs1 = rs1;
    assign cond_rs2 = rs2;

/*
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
*/
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

module load_alu (
    input clock,
    input reset, 
    input [`XLEN-1:0] opa, 
    input [`XLEN-1:0] opb,
    input IS_EX_PACKET in_packet,
    input ALU_FUNC alu_func,
    input load_en,
    // the BUS_LOAD response will magically be present in the *same* cycle it's requested (0ns latency)
    // this will not be true in project 4 (100ns latency)
    input [`XLEN-1:0]   Dmem2proc_data,
    input [3:0]         Dmem2proc_response,

    output logic [`XLEN-1:0]  result,
    output IS_EX_PACKET out_packet,
    output logic [`NUM_FU_LOAD] load_done,
    output logic [1:0]       proc2Dmem_command, // The memory command
    output MEM_SIZE          proc2Dmem_size,    // Size of data to read or write
    output logic [`XLEN-1:0] proc2Dmem_addr,    // Address sent to Data memory
    output logic [`XLEN-1:0] proc2Dmem_data     // Data sent to Data memory
); 

    logic [`XLEN-1:0] load_opa, load_opb, address;
    logic start;
    assign signed_opa   = load_opa;
    assign signed_opb   = load_opb;

    always_comb begin
        case (alu_func)
            ALU_ADD:    address = load_opa + load_opb;
            ALU_SUB:    address = load_opa - load_opb;
            ALU_AND:    address = load_opa & load_opb;
            ALU_SLT:    address = signed_opa < signed_opb;
            ALU_SLTU:   address = load_opa < load_opb;
            ALU_OR:     address = load_opa | load_opb;
            ALU_XOR:    address = load_opa ^ load_opb;
            ALU_SRL:    address = load_opa >> load_opb[4:0];
            ALU_SLL:    address = load_opa << load_opb[4:0];
            ALU_SRA:    address = signed_opa >>> load_opb[4:0]; // arithmetic from logical shift

            default:    address = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

    load load_0 (
        .start (start),
        .is_ex_reg (out_packet), 
        .address (address),
        .Dmem2proc_data (Dmem2proc_data),
        .Dmem2proc_response (Dmem2proc_response),

        .proc2Dmem_command (proc2Dmem_command),
        .proc2Dmem_size (proc2Dmem_size),
        .proc2Dmem_addr (proc2Dmem_addr),
        .proc2Dmem_data (proc2Dmem_data),
        .result(result),
        .done(load_done)
    );
/*
    assign out_packet = in_packet;
    assign load_opa = opa;
    assign load_opb = opb;
    assign start = load_en;
*/
    always_ff @(posedge clock) begin
        if(reset) begin
            out_packet      <= 0;
            load_opa      <= 0;
            load_opb      <= 0;
            start <= 0;
        end else if (load_en) begin
            out_packet      <= in_packet;
            load_opa      <= opa;
            load_opb      <= opb;
            start <= 1;
        end else if(!load_done) begin
            load_opa      <= load_opa;
            load_opb      <= load_opb;
            out_packet    <= out_packet;
            start <= start; 
        end else begin
	    load_opa      <= load_opa;
            load_opb      <= load_opb;
            out_packet    <= out_packet;
            start <= 0; 
	end
    end

endmodule

// ALU: computes the result of FUNC applied with operands A and B
module store (
    //input                           clock, 
    //input                           reset,
    input [`XLEN-1:0]               opa,
    input [`XLEN-1:0]               opb,
    input INST			    inst,
    input ALU_FUNC                  func,
    input logic                     store_en,
    input IS_EX_PACKET              in_packet,

    output logic [`XLEN-1:0]        result,
    output logic [`NUM_FU_ALU-1:0]  store_done,
    output MEM_SIZE		    mem_size,
    output IS_EX_PACKET             out_packet
);

    logic [`XLEN-1:0]           store_opa, store_opb;
    logic signed [`XLEN-1:0]    signed_opa, signed_opb;

    assign signed_opa   = store_opa;
    assign signed_opb   = store_opb;

    always_comb begin
        case (func)
            ALU_ADD:    result = store_opa + store_opb;
            ALU_SUB:    result = store_opa - store_opb;
            ALU_AND:    result = store_opa & store_opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = store_opa < store_opb;
            ALU_OR:     result = store_opa | store_opb;
            ALU_XOR:    result = store_opa ^ store_opb;
            ALU_SRL:    result = store_opa >> store_opb[4:0];
            ALU_SLL:    result = store_opa << store_opb[4:0];
            ALU_SRA:    result = signed_opa >>> store_opb[4:0]; // arithmetic from logical shift

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

    assign mem_size     = MEM_SIZE'(inst.r.funct3[1:0]);
    assign store_done[0] = store_en;
    assign store_opa = opa;
    assign store_opb = opb;
    assign out_packet = in_packet;
/*
    always_ff @(posedge clock) begin
        if(reset) begin
            store_done[0]     <= 1'b0;
            out_packet      <= 0;
            store_opa         <= 0;
            store_opb         <= 0;
        end else if (store_en) begin
            store_done[0]     <= 1'b1;
            out_packet      <= in_packet;
            store_opa         <= opa;
            store_opb         <= opb;
        end else begin
            store_done[0]     <= 1'b0;
            store_opa         <= store_opa;
            store_opb         <= store_opb;
            out_packet      <= out_packet;
        end
    end
*/
endmodule // alu
