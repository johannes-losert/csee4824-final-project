// This is a pipelined multiplier that multiplies two 64-bit integers and
// returns the low 64 bits of the result.
// This is not an ideal multiplier but is sufficient to allow a faster clock
// period than straight multiplication.

`include "verilog/sys_defs.svh"

module mult (
    input clock,
    reset,
    input [2*`XLEN-1:0] mcand,
    mplier,
    input signed_input1,
    input signed_input2,
    input ALU_FUNC func,
    input start,

    output logic [`XLEN-1:0] product,
    output done
);

  logic signed [63:0] signed_mcand, signed_mplier;
  logic signed [63:0] signed_product;
  logic [63:0] next_product;
  logic [`MULT_STAGES-2:0] internal_dones;
  logic signed [(64*(`MULT_STAGES-1))-1:0]
      internal_product_sums, internal_mcands, internal_mpliers;
  logic [63:0] mcand_out, mplier_out;

  // set mcand & mplier to appropriate value based on sign
  always @(*) begin
    if (signed_input1) begin
      signed_mcand = $signed(mcand);
    end else begin
      signed_mcand = mcand;
    end

    if (signed_input2) begin
      signed_mplier = $signed(mplier);
    end else begin
      signed_mplier = mplier;
    end
  end

  // instantiate an array of mult_stage modules
  // this uses concatenation syntax for internal wiring, see lab 2 slides
  mult_stage mstage[`MULT_STAGES-1:0] (
      .clock      (clock),
      .reset      (reset),
      .start      ({internal_dones, start}),
      .prev_sum   ({internal_product_sums, 64'h0}),
      .mplier     ({internal_mpliers, signed_mplier}),
      .mcand      ({internal_mcands, signed_mcand}),
      .product_sum({signed_product, internal_product_sums}),
      .next_mplier({mplier_out, internal_mpliers}),
      .next_mcand ({mcand_out, internal_mcands}),
      .done       ({done, internal_dones})
  );

  // crop outputs depending on whether the lower or upper 32-bits of the product are desired
  always_comb begin
    logic [2*`XLEN-1:0] tmp_product;
    //$display(signed_product);
    case (func)
      ALU_MUL: product = signed_product[`XLEN-1:0];
      ALU_MULH: product = signed_product[2*`XLEN-1:`XLEN];
      ALU_MULHSU: product = signed_product[2*`XLEN-1:`XLEN];
      ALU_MULHU: begin
        tmp_product = {1'b0, signed_product};
        product = tmp_product[2*`XLEN-1:`XLEN];
      end
      default: product = signed_product[2*`XLEN-1:`XLEN];
    endcase
    //$display(product);
  end

endmodule
