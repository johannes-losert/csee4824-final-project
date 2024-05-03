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
// `include "verilog/functional_units.sv"

module stage_ex (
    
    input                               clock,
    input                               reset,
    input IS_EX_PACKET                  is_ex_reg,
    // input logic [`MAX_FU_INDEX-1:0]     issue_fu_index, // add this to packet at earlier stage of the pipeline

    output EX_CO_PACKET ex_packet,

    output logic [`NUM_FU_ALU-1:0]    free_alu,
    output logic [`NUM_FU_MULT-1:0]   free_mult,
    output logic [`NUM_FU_LOAD-1:0]   free_load,
    output logic [`NUM_FU_STORE-1:0]  free_store,
    output logic [`NUM_FU_BRANCH-1:0] free_branch,
    input [`XLEN-1:0]   Dmem2proc_data,
    input [3:0]         Dmem2proc_response,

    // debug outputs
    output IS_EX_PACKET alu_packet,
    output IS_EX_PACKET mult_packet,
    output IS_EX_PACKET branch_packet,
    output IS_EX_PACKET load_packet,
    output IS_EX_PACKET store_packet,
    output logic [1:0]       proc2Dmem_command, // The memory command
    output MEM_SIZE          proc2Dmem_size,    // Size of data to read or write
    output logic [`XLEN-1:0] proc2Dmem_addr,    // Address sent to Data memory
    output logic [`XLEN-1:0] proc2Dmem_data     // Data sent to Data memory
);

    logic [`XLEN-1:0]   alu_result;
    logic [`XLEN-1:0]   mult_result;
    logic [`XLEN-1:0]   branch_result;
    logic [`XLEN-1:0]   load_result;
    logic [`XLEN-1:0] store_result;
    logic [`XLEN-1:0]   opa_mux_out, opb_mux_out;
    logic               take_conditional;

    // ALU opA mux
    always_comb begin
        case (is_ex_reg.opa_select)
            OPA_IS_RS1:  opa_mux_out = is_ex_reg.opa_value;
            OPA_IS_NPC:  opa_mux_out = is_ex_reg.NPC;
            OPA_IS_PC:   opa_mux_out = is_ex_reg.PC;
            OPA_IS_ZERO: opa_mux_out = 0;
            default:     opa_mux_out = `XLEN'hdeadface; // dead face
        endcase
    end

    // ALU opB mux
    always_comb begin
        case (is_ex_reg.opb_select)
            OPB_IS_RS2:   opb_mux_out = is_ex_reg.opb_value;
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
    logic load_done; 
    logic store_done;
    logic [`XLEN-1:0] tmp_alu_result; 
    logic [`XLEN-1:0] tmp_mult_result;
    logic [`XLEN-1:0] tmp_branch_result;
    logic [`XLEN-1:0] tmp_load_result;
    logic [`XLEN-1:0] tmp_store_result;
    IS_EX_PACKET tmp_alu_packet;
    IS_EX_PACKET tmp_mult_packet;
    IS_EX_PACKET tmp_branch_packet;
    IS_EX_PACKET tmp_load_packet;
    IS_EX_PACKET tmp_store_packet;
    MEM_SIZE mem_size;

    logic [`MAX_FU_INDEX-1:0] issue_fu_index;

    assign issue_fu_index = is_ex_reg.issued_fu_index;

    // Instantiate the ALU
    alu alu_0 (
        // Inputs
        //.clock      (clock),
        //.reset      (reset),
        .opa        (opa_mux_out),
        .opb        (opb_mux_out),
        .func       (is_ex_reg.alu_func),
        .alu_en     (is_ex_reg.function_type == ALU && is_ex_reg.valid && issue_fu_index == 0),
        .in_packet  (is_ex_reg),

        // Output
        .result     (alu_result),
        .alu_done   (alu_done),
        .out_packet (alu_packet)
    );

    branch_calculation branch_0 (
        // Inputs
        //.clock      (clock),
        //.reset      (reset),
        .opa        (opa_mux_out),
        .opb        (opb_mux_out),
        .alu_func   (is_ex_reg.alu_func),
        .branch_en  (is_ex_reg.function_type == BRANCH && is_ex_reg.valid && issue_fu_index == 0),
        .in_packet  (is_ex_reg),

        // Output
        .result         (branch_result),
        .branch_done    (branch_done),
        .out_packet     (branch_packet)
    );

    // Instantiate the conditional branch module
    conditional_branch conditional_branch_0 (
        // Inputs
        //.clock      (clock),
        //.reset      (reset),
        .func       (is_ex_reg.inst.b.funct3), // instruction bits for which condition to check
        .rs1        (is_ex_reg.opa_value),
        .rs2        (is_ex_reg.opb_value),
        .cond_en    (is_ex_reg.function_type == BRANCH && is_ex_reg.valid && issue_fu_index == 0),

        // Output
        .take       (take_conditional)
    );  

    // Instantiate multiply functional unit
    multiply mult_0 (
        // Inputs
        .clock      (clock),
        .reset      (reset),
        .mcand      (opa_mux_out),
        .mplier     (opb_mux_out),
        .func       (is_ex_reg.alu_func),
        .mult_en    (is_ex_reg.function_type == MULT && is_ex_reg.valid && issue_fu_index == 0),
        .in_packet  (is_ex_reg),

        // Output
        .product    (mult_result),
        .mult_done  (mult_done),
        .out_packet (mult_packet)
    );

    load_alu load_alu_0 (
        .clock (clock), 
        .reset (reset),
        .opa (opa_mux_out),
        .opb (opb_mux_out),
        .in_packet (is_ex_reg),
        .alu_func (is_ex_reg.alu_func),
        .load_en (is_ex_reg.function_type == LOAD && is_ex_reg.valid && issue_fu_index == 0),
        .Dmem2proc_data (Dmem2proc_data),
        .Dmem2proc_response (Dmem2proc_response),

        .result (load_result),
        .load_done (load_done),
        .out_packet (load_packet),
        .proc2Dmem_command (proc2Dmem_command),
        .proc2Dmem_size (proc2Dmem_size),
        .proc2Dmem_addr (proc2Dmem_addr),
        .proc2Dmem_data (proc2Dmem_data)
    );
    
    // Instantiate the ALU
    store store_0 (
        // Inputs
        //.clock      (clock),
        //.reset      (reset),
        .opa        (opa_mux_out),
        .opb        (opb_mux_out),
	.inst       (is_ex_reg.inst),
        .func       (is_ex_reg.alu_func),
        .store_en     (is_ex_reg.function_type == STORE && is_ex_reg.valid && issue_fu_index == 0),
        .in_packet  (is_ex_reg),

        // Output
        .result     (store_result),
        .store_done   (store_done),
	.mem_size	(mem_size),
        .out_packet (store_packet)
    );

    // Choose which FU to move to the next stage

    logic [5:0] waiting_fus; 
    logic alu_done_process;
    logic mult_done_process;
    logic branch_done_process;
    logic load_done_process;
    logic store_done_process;
    logic tmp_take_conditional;

    // If a FU is selected to be done with the ex stage, then set the ex_packet attributes to their
    // corresponding values
    always_comb begin
        if(alu_done_process) begin
            ex_packet.result        = tmp_alu_result;
            ex_packet.take_branch   = 0;
	        ex_packet.mem_size = 0;


            // Pass throughs 
            ex_packet.inst = tmp_alu_packet.inst;
            ex_packet.PC = tmp_alu_packet.PC;
            ex_packet.NPC = tmp_alu_packet.NPC;

            ex_packet.opa_select = tmp_alu_packet.opa_select;
            ex_packet.opb_select = tmp_alu_packet.opb_select;

            ex_packet.opa_value = tmp_alu_packet.opa_value;
            ex_packet.opb_value = tmp_alu_packet.opb_value;

            ex_packet.dest_reg_idx = tmp_alu_packet.dest_reg_idx;
        
            ex_packet.alu_func = tmp_alu_packet.alu_func;
            ex_packet.rd_mem = tmp_alu_packet.rd_mem;
            ex_packet.wr_mem = tmp_alu_packet.wr_mem;
            ex_packet.cond_branch = tmp_alu_packet.cond_branch;
            ex_packet.uncond_branch = tmp_alu_packet.uncond_branch;
            ex_packet.halt = tmp_alu_packet.halt;
            ex_packet.illegal = tmp_alu_packet.illegal;
            ex_packet.csr_op = tmp_alu_packet.csr_op;

            ex_packet.function_type = tmp_alu_packet.function_type;
            ex_packet.valid = tmp_alu_packet.valid;

            ex_packet.rob_index = tmp_alu_packet.rob_index;
            ex_packet.has_dest = tmp_alu_packet.has_dest;

            ex_packet.issued_fu_index = tmp_alu_packet.issued_fu_index;
        end else if (mult_done_process) begin
            ex_packet.result        = tmp_mult_result;
            ex_packet.take_branch   = 0;
	    ex_packet.mem_size = 0;

            
            // Pass throughs
            ex_packet.inst = tmp_mult_packet.inst;
            ex_packet.PC = tmp_mult_packet.PC;
            ex_packet.NPC = tmp_mult_packet.NPC;

            ex_packet.opa_select = tmp_mult_packet.opa_select;
            ex_packet.opb_select = tmp_mult_packet.opb_select;

            ex_packet.opa_value = tmp_mult_packet.opa_value;
            ex_packet.opb_value = tmp_mult_packet.opb_value;

            ex_packet.dest_reg_idx = tmp_mult_packet.dest_reg_idx;

            ex_packet.alu_func = tmp_mult_packet.alu_func;
            ex_packet.rd_mem = tmp_mult_packet.rd_mem;
            ex_packet.wr_mem = tmp_mult_packet.wr_mem;
            ex_packet.cond_branch = tmp_mult_packet.cond_branch;
            ex_packet.uncond_branch = tmp_mult_packet.uncond_branch;
            ex_packet.halt = tmp_mult_packet.halt;
            ex_packet.illegal = tmp_mult_packet.illegal;
            ex_packet.csr_op = tmp_mult_packet.csr_op;

            ex_packet.function_type = tmp_mult_packet.function_type;
            ex_packet.valid = tmp_mult_packet.valid;

            ex_packet.rob_index = tmp_mult_packet.rob_index;
            ex_packet.has_dest = tmp_mult_packet.has_dest;

            ex_packet.issued_fu_index = tmp_mult_packet.issued_fu_index;
        end else if (branch_done_process) begin
            ex_packet.result        = tmp_branch_result;
            ex_packet.take_branch   = tmp_branch_packet.uncond_branch || (tmp_branch_packet.cond_branch && tmp_take_conditional);
	        ex_packet.mem_size = 0;

            // Pass throughs

            ex_packet.inst = tmp_branch_packet.inst;
            ex_packet.PC = tmp_branch_packet.PC;
            ex_packet.NPC = tmp_branch_packet.NPC;

            ex_packet.opa_select = tmp_branch_packet.opa_select;
            ex_packet.opb_select = tmp_branch_packet.opb_select;

            ex_packet.opa_value = tmp_branch_packet.opa_value;
            ex_packet.opb_value = tmp_branch_packet.opb_value;

            ex_packet.dest_reg_idx = tmp_branch_packet.dest_reg_idx;

            ex_packet.alu_func = tmp_branch_packet.alu_func;
            ex_packet.rd_mem = tmp_branch_packet.rd_mem;
            ex_packet.wr_mem = tmp_branch_packet.wr_mem;
            ex_packet.cond_branch = tmp_branch_packet.cond_branch;
            ex_packet.uncond_branch = tmp_branch_packet.uncond_branch;
            ex_packet.halt = tmp_branch_packet.halt;
            ex_packet.illegal = tmp_branch_packet.illegal;
            ex_packet.csr_op = tmp_branch_packet.csr_op;

            ex_packet.function_type = tmp_branch_packet.function_type;
            ex_packet.valid = tmp_branch_packet.valid;

            ex_packet.rob_index = tmp_branch_packet.rob_index;
            ex_packet.has_dest = tmp_branch_packet.has_dest;

            ex_packet.issued_fu_index = tmp_branch_packet.issued_fu_index;

        end else if (load_done_process) begin
            ex_packet.result        = tmp_load_result;
            ex_packet.take_branch   = 0;
	    ex_packet.mem_size = 0;

            
            // Pass throughs
            ex_packet.inst = tmp_load_packet.inst;
            ex_packet.PC = tmp_load_packet.PC;
            ex_packet.NPC = tmp_load_packet.NPC;

            ex_packet.opa_select = tmp_load_packet.opa_select;
            ex_packet.opb_select = tmp_load_packet.opb_select;

            ex_packet.opa_value = tmp_load_packet.opa_value;
            ex_packet.opb_value = tmp_load_packet.opb_value;

            ex_packet.dest_reg_idx = tmp_load_packet.dest_reg_idx;

            ex_packet.alu_func = tmp_load_packet.alu_func;
            ex_packet.rd_mem = tmp_load_packet.rd_mem;
            ex_packet.wr_mem = tmp_load_packet.wr_mem;
            ex_packet.cond_branch = tmp_load_packet.cond_branch;
            ex_packet.uncond_branch = tmp_load_packet.uncond_branch;
            ex_packet.halt = tmp_load_packet.halt;
            ex_packet.illegal = tmp_load_packet.illegal;
            ex_packet.csr_op = tmp_load_packet.csr_op;

            ex_packet.function_type = tmp_load_packet.function_type;
            ex_packet.valid = tmp_load_packet.valid;

            ex_packet.rob_index = tmp_load_packet.rob_index;
            ex_packet.has_dest = tmp_load_packet.has_dest;

            ex_packet.issued_fu_index = tmp_load_packet.issued_fu_index;
        end else if (store_done_process) begin
            ex_packet.result        = tmp_store_result;
            ex_packet.take_branch   = 0;
            
            // Pass throughs
            ex_packet.inst = tmp_store_packet.inst;
            ex_packet.PC = tmp_store_packet.PC;
            ex_packet.NPC = tmp_store_packet.NPC;

            ex_packet.opa_select = tmp_store_packet.opa_select;
            ex_packet.opb_select = tmp_store_packet.opb_select;

            ex_packet.opa_value = tmp_store_packet.opa_value;
            ex_packet.opb_value = tmp_store_packet.opb_value;

            ex_packet.dest_reg_idx = tmp_store_packet.dest_reg_idx;

            ex_packet.alu_func = tmp_store_packet.alu_func;
            ex_packet.rd_mem = tmp_store_packet.rd_mem;
            ex_packet.wr_mem = tmp_store_packet.wr_mem;
            ex_packet.cond_branch = tmp_store_packet.cond_branch;
            ex_packet.uncond_branch = tmp_store_packet.uncond_branch;
            ex_packet.halt = tmp_store_packet.halt;
            ex_packet.illegal = tmp_store_packet.illegal;
            ex_packet.csr_op = tmp_store_packet.csr_op;

            ex_packet.function_type = tmp_store_packet.function_type;
            ex_packet.valid = tmp_store_packet.valid;

            ex_packet.rob_index = tmp_store_packet.rob_index;
            ex_packet.has_dest = tmp_store_packet.has_dest;
	    ex_packet.mem_size = mem_size;


            ex_packet.issued_fu_index = tmp_store_packet.issued_fu_index;
        end else begin 
            ex_packet = INVALID_EX_CO_PACKET;
        end
    end

    // Chooses which FU to move forward to the next stage
    always_ff @(posedge clock) begin
        if(reset) begin
            waiting_fus             <= 0;
            tmp_alu_result          <= 0;
            tmp_mult_result         <= 0;
            tmp_branch_result       <= 0;
            tmp_load_result         <= 0;
            tmp_store_result <= 0;
            tmp_alu_packet <= 0;
            tmp_mult_packet <= 0;
            tmp_branch_packet <= 0;
            tmp_load_packet <= 0;
            tmp_store_packet <= 0;
            alu_done_process        <= 0;
            mult_done_process       <= 0;
            branch_done_process     <= 0;
            load_done_process <= 0;
            store_done_process <= 0;
            tmp_take_conditional    <= 0;
        end
        else begin
            // If any of the FUs is outputting its down signal, add it to a list that signifies that that 
            // FU is ready to move forward to the next stage (waiting_fus)
            if(alu_done) begin
                $display("[EX] ALU done, result=%h", alu_result);
                waiting_fus[ALU]    <= 1;
                tmp_alu_result      <= alu_result;
                tmp_alu_packet <= alu_packet;
            end

            if(mult_done) begin
                waiting_fus[MULT]   <= 1; 
                tmp_mult_result     <= mult_result;
                tmp_mult_packet <= mult_packet;
            end
            if(branch_done) begin
                $write("[EX] BRANCH done,");
                if (take_conditional) begin
                    $write(" take branch=true");
                end else begin 
                    $write(" take branch=false");
                end
                $display(" branch_result=%h", branch_result);
                waiting_fus[BRANCH] <= 1; 
                tmp_branch_result   <= branch_result;
                tmp_branch_packet <= branch_packet;
                tmp_take_conditional<= take_conditional;
            end 
            if (load_done) begin
                waiting_fus[LOAD] <= 1; 
                tmp_load_result   <= load_result;
                tmp_load_packet <= load_packet;
            end
            if (store_done) begin
                waiting_fus[STORE] <= 1;
                tmp_store_result <= store_result;
                tmp_store_packet <= store_packet;
            end

            // Defaults
            alu_done_process        <= 0;
            mult_done_process       <= 0;
            branch_done_process     <= 0;
            load_done_process <= 0;
            store_done_process <= 0;
            free_alu[0]             <= 0;
            free_mult[0]            <= 0;
            free_branch[0]          <= 0;
            free_load[0] <= 0;
            free_store[0] <= 0;

            // Choose one of those FUs from waiting_fus. Set its passthroughs (in an always_comb block above), 
            // and set that free_[FU] variable to 1. Also remember to say that that FU is no longer ready to move on to the
            // next stage. 
            if(waiting_fus[ALU] == 1) begin
                waiting_fus[ALU]    <= 0;
                alu_done_process    <= 1; 
                free_alu[0]         <= 1;
            end else if(waiting_fus[MULT] == 1) begin
                waiting_fus[MULT]   <= 0;
                mult_done_process   <= 1;
                free_mult[0]        <= 1;
            end else if(waiting_fus[BRANCH] == 1) begin
                waiting_fus[BRANCH] <= 0;
                branch_done_process <= 1;
                free_branch[0]      <= 1;
            end else if(waiting_fus[LOAD] == 1) begin
                waiting_fus[LOAD] <= 0;
                load_done_process <= 1;
                free_load[0] <= 1;
            end else if(waiting_fus[STORE] == 1) begin
                waiting_fus[STORE] <= 0;
                store_done_process <= 1;
                free_store[0] <= 1;
            end
            
        end
    end     

endmodule // stage_ex
