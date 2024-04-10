// Simple bimodal branch predictor
`include "verilog/sys_defs.svh"


// simply bimodal branch predictor
// At initialization all branches are predicted to be strongly not taken
module branch_predictor (
    input logic clock, 
    input logic reset, 

    input logic [`XLEN-1:0] if_pc,

    // update the branch 
    input logic [`XLEN-1:0] ex_pc,
    input logic ex_is_branch_taken,
    input logic ex_is_branch_not_taken,

    // the module predicts the branch to be taken when this is high
    output logic predict_branch_taken, 
    output logic hit
);

    BP_ENTRY buffer [`BP_ENTRIES-1:0];
    BP_ENTRY n_buffer [`BP_ENTRIES-1:0];

    logic [3:0] ex_tag;
    assign ex_tag = ex_pc[`BTB_TAG_LEN:0];

    logic [3:0] if_tag;
    assign if_tag = if_pc[`BTB_TAG_LEN:0];

    assign hit = buffer[if_tag].valid;


    always_comb begin
        if (reset) begin
            for (int i = 0; i < `BP_ENTRIES; i = i + 1) begin
                n_buffer[i].state = SNT;
                n_buffer[i].valid = 0;
            end
        end else if (ex_is_branch_taken) begin
            n_buffer[ex_tag].valid = 1;
            case (buffer[ex_tag].state)
                SNT : n_buffer[ex_tag].state = WNT; 
                WNT : n_buffer[ex_tag].state = WT;
                WT  : n_buffer[ex_tag].state = ST;
                ST  : n_buffer[ex_tag].state = ST;
            endcase 
        end else if (ex_is_branch_not_taken) begin
            n_buffer[ex_tag].valid = 1;
            case (buffer[ex_tag].state)
                SNT : n_buffer[ex_tag].state = SNT; 
                WNT : n_buffer[ex_tag].state = SNT;
                WT  : n_buffer[ex_tag].state = WNT;
                ST  : n_buffer[ex_tag].state = WT;
            endcase 
        end

        case (buffer[if_tag].state)
            SNT : predict_branch_taken = 0;
            WNT : predict_branch_taken = 0;
            WT  : predict_branch_taken = 1;
            ST  : predict_branch_taken = 1;
        endcase 
    end

    always_ff @(posedge clock) begin
        buffer <= n_buffer;
    end

endmodule