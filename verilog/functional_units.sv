`include "verilog/sys_defs.svh"
`include "verilog/ISA.svh"
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
    output logic alu_done,
    output IS_EX_PACKET             out_packet
);

    logic [`XLEN-1:0]           alu_opa, alu_opb;
    logic signed [`XLEN-1:0]    signed_opa, signed_opb;
    logic [`MAX_FU_INDEX-1:0]   idx;

    assign signed_opa   = alu_opa;
    assign signed_opb   = alu_opb;
    assign alu_done = alu_en;
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

endmodule // alu

// For calculating branch addresses 
module branch_calculation (
    input [`XLEN-1:0]   opa,
    input [`XLEN-1:0]   opb,
    input ALU_FUNC      alu_func,
    input logic         branch_en,
    input IS_EX_PACKET  in_packet,

    output logic [`XLEN-1:0]        result,
    output logic    branch_done,
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

    assign branch_done = branch_en;
    assign out_packet = in_packet;
    assign branch_opa = opa;
    assign branch_opb = opb;

endmodule

// Conditional branch module: compute whether to take conditional branches
module conditional_branch (
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
    output logic     mult_done,
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
        .done           (mult_done)
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


/* Combinationally do ALU aritmetic */
module arithmetic (
    input [`XLEN-1:0] arith_opa,
    input [`XLEN-1:0] arith_opb,
    input ALU_FUNC alu_func,

    output logic [`XLEN-1:0] result
);

    logic signed [`XLEN-1:0] signed_opa, signed_opb;

    assign signed_opa = arith_opa;
    assign signed_opb = arith_opb;

    always_comb begin
        case (alu_func)
            ALU_ADD:    result = arith_opa + arith_opb;
            ALU_SUB:    result = arith_opa - arith_opb;
            ALU_AND:    result = arith_opa & arith_opb;
            ALU_SLT:    result = signed_opa < signed_opb;
            ALU_SLTU:   result = arith_opa < arith_opb;
            ALU_OR:     result = arith_opa | arith_opb;
            ALU_XOR:    result = arith_opa ^ arith_opb;
            ALU_SRL:    result = arith_opa >> arith_opb[4:0];
            ALU_SLL:    result = arith_opa << arith_opb[4:0];
            ALU_SRA:    result = signed_opa >>> arith_opb[4:0]; // arithmetic from logical shift

            default:    result = `XLEN'hfacebeec;  // here to prevent latches
        endcase
    end

endmodule 


/* Takes in load instruction, calculates address, sends request to dcache, waits for response */
module load ( 
    input clock, 
    input reset, 

    input start_load, /* Stays high until load finishes */

    /* Instruction information */
    input [`XLEN-1:0] in_opa,
    input [`XLEN-1:0] in_opb,
    input IS_EX_PACKET in_packet,
    input ALU_FUNC alu_func,

    /* Input from dcache */
    input [63:0] Dcache_data_out, // Data is mem[proc2Dcache_addr]
    input Dcache_valid_out, // When valid is high

    /* Output to dcache */
    output logic load_en,
    output [`XLEN-1:0] load2Dcache_addr,

    /* Output for pipeline */
    output logic [`XLEN-1:0] result,
    output IS_EX_PACKET out_packet,
    output logic load_done

);
    /* Must keep a copy of all the state we need, since inputs may change */
    logic started;
    IS_EX_PACKET curr_packet;
    ALU_FUNC curr_alu_func; 
    logic [`XLEN-1:0] curr_opa, curr_opb;

    /* Just forward the current packet to output packet */
    assign out_packet = curr_packet;

    /* Check if we should issue load request to dcache */
    assign load_en = (started && curr_packet.valid && curr_packet.rd_mem);

    /* Calculate address to load from */
    arithmetic arith_0 (
        .arith_opa (curr_opa),
        .arith_opb (curr_opb),
        .alu_func (curr_alu_func),
        .result (load2Dcache_addr)
    );

    /* Extract data word from double word returned by dcache:
        - if address[2] is 1, word is bits 63-32
        - else, word is bits 31-0
       Compute extension bytes based on instruction:
        - if unsigned, zero-extend
        - if signed, sign-extend
       Mask with extend bytes based on size:
        - if byte, mask all but last 8 bits
    */ 
    
    MEM_SIZE memory_size;
    logic rd_unsigned;
    logic [`XLEN-1:0] data_word;

    assign memory_size = MEM_SIZE'(curr_packet.inst.r.funct3[1:0]);
    assign rd_unsigned = curr_packet.inst.r.funct3[2];
    assign data_word = load2Dcache_addr[2] ? Dcache_data_out[63:`XLEN] : Dcache_data_out[`XLEN-1:0];

    always_comb begin 
        /* If dcache has finished with our request */
        if (started && Dcache_valid_out) begin
            result = data_word;
            if(rd_unsigned) begin 
                // unsigned: zero-extend the data
                if(memory_size == BYTE) begin
                    result[`XLEN-1:8] = 0;
                end else if(memory_size == HALF) begin
                    result[`XLEN-1:16] = 0;
                end
            end else begin
                if(memory_size == BYTE) begin
                    result[`XLEN-1:8] = {(`XLEN-8){data_word[7]}};
                end else if(memory_size == HALF) begin
                    result[`XLEN-1:16] = {(`XLEN-16){data_word[15]}};
                end
            end
            load_done = 1; 
        end else begin
            result = 0;
            load_done = 0;
        end
    end 

    always_ff @(posedge clock) begin
        if(reset) begin
            curr_packet    <= 0;
            curr_opa      <= 0;
            curr_opb      <= 0;
            curr_alu_func <= 0;
            started <= 0;
        end else if (start_load) begin
            curr_packet    <= in_packet;
            curr_opa      <= in_opa;
            curr_opb      <= in_opb;
            curr_alu_func <= alu_func;
            started <= 1;
        end else if (load_done) begin 
	        curr_packet   <= 0;
            curr_opa      <= 0;
            curr_opb      <= 0;
            curr_alu_func <= 0;
            started <= 0; 
        end
    end
endmodule 

// ALU: computes the result of FUNC applied with operands A and B
module store (
    input [`XLEN-1:0]               opa,
    input [`XLEN-1:0]               opb,
    input INST			    inst,
    input ALU_FUNC                  func,
    input logic                     store_en,
    input IS_EX_PACKET              in_packet,

    output logic [`XLEN-1:0]        result,
    output logic  store_done,
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
    assign store_done = store_en;
    assign store_opa = opa;
    assign store_opb = opb;
    assign out_packet = in_packet;

endmodule // alu
