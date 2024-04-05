// Simple bimodal branch predictor

module branch_predictor (
    input logic clock, 
    input logic reset, 

    input logic [`XLEN-1:0] if_pc,

    // update the branch 
    input logic [`XLEN-1:0] ex_pc,
    input logic ex_branch_taken,

    // the module predicts the branch to be taken when this is high
    output logic predict_branch_taken, 
);



endmodule