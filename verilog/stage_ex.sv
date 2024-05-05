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
    
    /* Input from dcache */
    input [63:0]   Dcache_data_out,
    input logic         Dcache_valid_out,

    /* output to dcache*/
    output logic         ex_load_en,
    output logic [`XLEN-1:0] ex_load2Dcache_addr,    // Address sent to Data memory

    /* Input to roll back */
    input logic rollback,

    // debug outputs
    output IS_EX_PACKET [`NUM_FU_ALU-1:0] alu_packet,
    output IS_EX_PACKET [`NUM_FU_MULT-1:0] mult_packet,
    output IS_EX_PACKET [`NUM_FU_BRANCH-1:0] branch_packet,
    output IS_EX_PACKET [`NUM_FU_LOAD-1:0] load_packet,
    output IS_EX_PACKET [`NUM_FU_STORE-1:0] store_packet
);

    logic [`NUM_FU_ALU-1:0] [`XLEN-1:0]   alu_result;
    logic [`NUM_FU_MULT-1:0] [`XLEN-1:0]   mult_result;
    logic [`NUM_FU_BRANCH-1:0] [`XLEN-1:0]   branch_result;
    logic [`NUM_FU_LOAD-1:0] [`XLEN-1:0]   load_result;
    logic [`NUM_FU_STORE-1:0] [`XLEN-1:0]   store_result;
    logic [`NUM_FU_STORE-1:0] [63:0]   prev_dword;
    logic [`XLEN-1:0]   opa_mux_out, opb_mux_out;

    logic [`NUM_FU_BRANCH-1:0]              take_conditional;
    logic [`NUM_FU_LOAD-1:0] load_load_en;
    logic [`NUM_FU_LOAD-1:0] [`XLEN-1:0] load_load2Dcache_addr;

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

    logic [`NUM_FU_ALU-1:0] alu_done; 
    logic [`NUM_FU_MULT-1:0] mult_done; 
    logic [`NUM_FU_BRANCH-1:0] branch_done;
    logic [`NUM_FU_LOAD-1:0] load_done; 
    logic [`NUM_FU_STORE-1:0] store_done;
    logic [`NUM_FU_ALU-1:0] [`XLEN-1:0] tmp_alu_result;
    logic [`NUM_FU_MULT-1:0] [`XLEN-1:0] tmp_mult_result;
    logic [`NUM_FU_BRANCH-1:0] [`XLEN-1:0] tmp_branch_result;
    logic [`NUM_FU_LOAD-1:0] [`XLEN-1:0] tmp_load_result;
    logic [`NUM_FU_STORE-1:0] [`XLEN-1:0] tmp_store_result;
    logic [`NUM_FU_STORE-1:0] [63:0] tmp_store_prev_dword;
    IS_EX_PACKET [`NUM_FU_ALU-1:0] tmp_alu_packet;
    IS_EX_PACKET [`NUM_FU_MULT-1:0] tmp_mult_packet;
    IS_EX_PACKET [`NUM_FU_BRANCH-1:0] tmp_branch_packet;
    IS_EX_PACKET [`NUM_FU_LOAD-1:0] tmp_load_packet;
    IS_EX_PACKET [`NUM_FU_STORE-1:0] tmp_store_packet;
    MEM_SIZE [`NUM_FU_STORE] store_mem_size, tmp_store_mem_size;

    logic [`MAX_FU_INDEX-1:0] issue_fu_index;
    assign issue_fu_index = is_ex_reg.issued_fu_index;

    genvar idx;
    generate
        for (idx = 0; idx < `NUM_FU_ALU; idx++) begin : generate_alu_fus
            alu alu_0 (
                .opa        (opa_mux_out),
                .opb        (opb_mux_out),
                .func       (is_ex_reg.alu_func),
                .alu_en     (is_ex_reg.function_type == ALU && is_ex_reg.valid && issue_fu_index == idx),
                .in_packet  (is_ex_reg),

                // Output
                .result     (alu_result[idx]),
                .alu_done   (alu_done[idx]),
                .out_packet (alu_packet[idx])
            );
        end 
    endgenerate

    generate
        for (idx = 0; idx < `NUM_FU_BRANCH; idx++) begin : generate_branch_fus
            branch_calculation branch_0 (
                .opa        (opa_mux_out),
                .opb        (opb_mux_out),
                .alu_func   (is_ex_reg.alu_func),
                .branch_en  (is_ex_reg.function_type == BRANCH && is_ex_reg.valid && issue_fu_index == idx),
                .in_packet  (is_ex_reg),

                // Output
                .result         (branch_result[idx]),
                .branch_done    (branch_done[idx]),
                .out_packet     (branch_packet[idx])
            );
        end 
    endgenerate

    generate
        for (idx = 0; idx < `NUM_FU_BRANCH; idx++) begin : generate_take_branch_fus
            conditional_branch conditional_branch_0 (
                .func       (is_ex_reg.inst.b.funct3), // instruction bits for which condition to check
                .rs1        (is_ex_reg.rs1_value),
                .rs2        (is_ex_reg.rs2_value),
                .cond_en    (is_ex_reg.function_type == BRANCH && is_ex_reg.valid && issue_fu_index == idx),

                // Output
                .take       (take_conditional[idx])
            );
        end 
    endgenerate

    generate
        for (idx = 0; idx < `NUM_FU_MULT; idx++) begin : generate_mult_fus
            multiply multiply_0 (
                .clock      (clock),
                .reset      (reset),
                .mcand      (opa_mux_out),
                .mplier     (opb_mux_out),
                .func       (is_ex_reg.alu_func),
                .mult_en    (is_ex_reg.function_type == MULT && is_ex_reg.valid && issue_fu_index == idx),
                .in_packet  (is_ex_reg),

                // Output
                .product    (mult_result[idx]),
                .mult_done  (mult_done[idx]),
                .out_packet (mult_packet[idx])
            );
        end 
    endgenerate

    generate
        for (idx = 0; idx < `NUM_FU_LOAD; idx++) begin : generate_load_fus
            load load_0 (
                .clock(clock),
                .reset(reset),

                .start_load(is_ex_reg.function_type == LOAD && is_ex_reg.valid && issue_fu_index == idx),
                
                /* Instruction info */
                .in_opa(opa_mux_out),
                .in_opb(opb_mux_out),
                .in_packet(is_ex_reg),
                .alu_func(is_ex_reg.alu_func),

                /* Input from dcache */
                .Dcache_data_out(Dcache_data_out),
                .Dcache_valid_out(Dcache_valid_out),

                /* Output to dcache */
                .load_en(load_load_en[idx]),
                .load2Dcache_addr(load_load2Dcache_addr_arr[idx]),

                /* Output for pipeline */
                .result(load_result[idx]),
                .out_packet(load_packet[idx]),
                .load_done(load_done[idx])
            );
        end 
    endgenerate   

    logic [`NUM_FU_STORE-1:0] store_load_en;
    logic [`NUM_FU_STORE-1:0] [`XLEN-1:0] store_load2Dcache_addr;

    generate
        for (idx = 0; idx < `NUM_FU_STORE; idx++) begin : generate_store_fus
            store store (
                .clock(clock),
                .reset(reset),

                .start_store(is_ex_reg.function_type == STORE && is_ex_reg.valid && issue_fu_index == idx),
                
                /* Instruction info */
                .in_opa(opa_mux_out),
                .in_opb(opb_mux_out),
                .in_packet(is_ex_reg),
                .alu_func(is_ex_reg.alu_func),

                /* Input from dcache */
                .Dcache_data_out(Dcache_data_out),
                .Dcache_valid_out(Dcache_valid_out),

                /* Output to dcache */
                .load_en(store_load_en[idx]),
                .load2Dcache_addr(store_load2Dcache_addr[idx]),

                /* Output for pipeline */
                .result(store_result[idx]),
                .prev_dword(prev_dword[idx]),
                .out_packet(store_packet[idx]),
                .load_done(store_done[idx]),
                .mem_size(store_mem_size[idx])
            );
        end
    endgenerate


    /* Select whether to send signal from load or store to dcache */
    always_comb begin
        if (load_load_en != 0) begin
            for(int i = 0; i < `NUM_FU_LOAD; i++) begin
                if(load_load_en[i]) begin
                    ex_load_en = 1;
                    ex_load2Dcache_addr = load_load2Dcache_addr[i];
                    break;
                end
            end
        end else if (store_load_en != 0) begin
            for(int i = 0; i < `NUM_FU_STORE; i++) begin
                if(store_load_en[i]) begin
                    ex_load_en = 1;
                    ex_load2Dcache_addr = store_load2Dcache_addr[i];
                    break;
                end
            end
        end else begin 
            assert(!(load_load_en && store_load_en)); // Shouldn't do both at the same time
            ex_load_en = 0;
            ex_load2Dcache_addr = 0;
        end
    end 

    // Choose which FU to move to the next stage
    logic [`NUM_FU_ALU-1:0] alu_ready; 
    logic [`NUM_FU_ALU-1:0] alu_done_process;
    logic [`NUM_FU_MULT-1:0] mult_ready; 
    logic [`NUM_FU_MULT-1:0] mult_done_process;
    logic [`NUM_FU_BRANCH-1:0] branch_ready; 
    logic [`NUM_FU_BRANCH-1:0] branch_done_process;
    logic [`NUM_FU_LOAD-1:0] load_ready; 
    logic [`NUM_FU_LOAD-1:0] load_done_process;
    logic [`NUM_FU_STORE-1:0] store_ready; 
    logic [`NUM_FU_STORE-1:0] store_done_process;
    logic [`NUM_FU_BRANCH] tmp_take_conditional;
    logic [`NUM_FU_STORE] tmp_mem_size;
    logic [`NUM_FU_LOAD] tmp_load_en;
    logic [`NUM_FU_LOAD] tmp_load2Dcache_addr;

    // If a FU is selected to be done with the ex stage, then set the ex_packet attributes to their
    // corresponding values
    always_comb begin
        for (int i = 0; i < `NUM_FU_ALU; i++)  begin
                if (alu_done_process[i]) begin
                    ex_packet.result        = tmp_alu_result[i][`XLEN-1:0];
                    ex_packet.take_branch   = 0;
                    ex_packet.mem_size = 0;

                    // Pass throughs 
                    ex_packet.inst = tmp_alu_packet[i].inst;
                    ex_packet.PC = tmp_alu_packet[i].PC;
                    ex_packet.NPC = tmp_alu_packet[i].NPC;

                    ex_packet.opa_select = tmp_alu_packet[i].opa_select;
                    ex_packet.opb_select = tmp_alu_packet[i].opb_select;

                    ex_packet.rs1_value = tmp_alu_packet[i].rs1_value;
                    ex_packet.rs2_value = tmp_alu_packet[i].rs2_value;

                    ex_packet.dest_reg_idx = tmp_alu_packet[i].dest_reg_idx;
                
                    ex_packet.alu_func = tmp_alu_packet[i].alu_func;
                    ex_packet.rd_mem = tmp_alu_packet[i].rd_mem;
                    ex_packet.wr_mem = tmp_alu_packet[i].wr_mem;
                    ex_packet.cond_branch = tmp_alu_packet[i].cond_branch;
                    ex_packet.uncond_branch = tmp_alu_packet[i].uncond_branch;
                    ex_packet.halt = tmp_alu_packet[i].halt;
                    ex_packet.illegal = tmp_alu_packet[i].illegal;
                    ex_packet.csr_op = tmp_alu_packet[i].csr_op;

                    ex_packet.function_type = tmp_alu_packet[i].function_type;
                    ex_packet.valid = tmp_alu_packet[i].valid;

                    ex_packet.rob_index = tmp_alu_packet[i].rob_index;
                    ex_packet.has_dest = tmp_alu_packet[i].has_dest;

                    ex_packet.issued_fu_index = tmp_alu_packet[i].issued_fu_index;
                    ex_packet.arch_dest_reg_num = tmp_alu_packet[i].arch_dest_reg_num;
                    ex_packet.prev_dword = 64'hdeadface;
                end
        end
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            if(mult_done_process[i]) begin
                ex_packet.result        = tmp_mult_result[i];
                ex_packet.take_branch   = 0;
                ex_packet.mem_size = 0;

                // Pass throughs
                ex_packet.inst = tmp_mult_packet[i].inst;
                ex_packet.PC = tmp_mult_packet[i].PC;
                ex_packet.NPC = tmp_mult_packet[i].NPC;

                ex_packet.opa_select = tmp_mult_packet[i].opa_select;
                ex_packet.opb_select = tmp_mult_packet[i].opb_select;

                ex_packet.rs1_value = tmp_mult_packet[i].rs1_value;
                ex_packet.rs2_value = tmp_mult_packet[i].rs2_value;

                ex_packet.dest_reg_idx = tmp_mult_packet[i].dest_reg_idx;

                ex_packet.alu_func = tmp_mult_packet[i].alu_func;
                ex_packet.rd_mem = tmp_mult_packet[i].rd_mem;
                ex_packet.wr_mem = tmp_mult_packet[i].wr_mem;
                ex_packet.cond_branch = tmp_mult_packet[i].cond_branch;
                ex_packet.uncond_branch = tmp_mult_packet[i].uncond_branch;
                ex_packet.halt = tmp_mult_packet[i].halt;
                ex_packet.illegal = tmp_mult_packet[i].illegal;
                ex_packet.csr_op = tmp_mult_packet[i].csr_op;

                ex_packet.function_type = tmp_mult_packet[i].function_type;
                ex_packet.valid = tmp_mult_packet[i].valid;

                ex_packet.rob_index = tmp_mult_packet[i].rob_index;
                ex_packet.has_dest = tmp_mult_packet[i].has_dest;

                ex_packet.issued_fu_index = tmp_mult_packet[i].issued_fu_index;

                ex_packet.arch_dest_reg_num = tmp_mult_packet[i].arch_dest_reg_num;
                ex_packet.prev_dword = 64'hdeadface;
            end
        end
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if(branch_done_process[i]) begin
                ex_packet.result        = tmp_branch_result[i];
                ex_packet.take_branch   = tmp_branch_packet[i].uncond_branch || (tmp_branch_packet[i].cond_branch && tmp_take_conditional[i]);
                ex_packet.mem_size = 0;

                // Pass throughs

                ex_packet.inst = tmp_branch_packet[i].inst;
                ex_packet.PC = tmp_branch_packet[i].PC;
                ex_packet.NPC = tmp_branch_packet[i].NPC;

                ex_packet.opa_select = tmp_branch_packet[i].opa_select;
                ex_packet.opb_select = tmp_branch_packet[i].opb_select;

                ex_packet.rs1_value = tmp_branch_packet[i].rs2_value;
                ex_packet.rs2_value = tmp_branch_packet[i].rs2_value;

                ex_packet.dest_reg_idx = tmp_branch_packet[i].dest_reg_idx;

                ex_packet.alu_func = tmp_branch_packet[i].alu_func;
                ex_packet.rd_mem = tmp_branch_packet[i].rd_mem;
                ex_packet.wr_mem = tmp_branch_packet[i].wr_mem;
                ex_packet.cond_branch = tmp_branch_packet[i].cond_branch;
                ex_packet.uncond_branch = tmp_branch_packet[i].uncond_branch;
                ex_packet.halt = tmp_branch_packet[i].halt;
                ex_packet.illegal = tmp_branch_packet[i].illegal;
                ex_packet.csr_op = tmp_branch_packet[i].csr_op;

                ex_packet.function_type = tmp_branch_packet[i].function_type;
                ex_packet.valid = tmp_branch_packet[i].valid;

                ex_packet.rob_index = tmp_branch_packet[i].rob_index;
                ex_packet.has_dest = tmp_branch_packet[i].has_dest;

                ex_packet.issued_fu_index = tmp_branch_packet[i].issued_fu_index;

                ex_packet.arch_dest_reg_num = tmp_branch_packet[i].arch_dest_reg_num;
                ex_packet.prev_dword = 64'hdeadface;
            end
        end

        for (int i = 0; i < `NUM_FU_LOAD; i++) begin
            if(load_done_process[i]) begin
                ex_packet.result        = tmp_load_result[i];
                ex_packet.take_branch   = 0;
                ex_packet.mem_size = 0;

                
                // Pass throughs
                ex_packet.inst = tmp_load_packet[i].inst;
                ex_packet.PC = tmp_load_packet[i].PC;
                ex_packet.NPC = tmp_load_packet[i].NPC;

                ex_packet.opa_select = tmp_load_packet[i].opa_select;
                ex_packet.opb_select = tmp_load_packet[i].opb_select;

                ex_packet.rs1_value = tmp_load_packet[i].rs1_value;
                ex_packet.rs2_value = tmp_load_packet[i].rs2_value;

                ex_packet.dest_reg_idx = tmp_load_packet[i].dest_reg_idx;

                ex_packet.alu_func = tmp_load_packet[i].alu_func;
                ex_packet.rd_mem = tmp_load_packet[i].rd_mem;
                ex_packet.wr_mem = tmp_load_packet[i].wr_mem;
                ex_packet.cond_branch = tmp_load_packet[i].cond_branch;
                ex_packet.uncond_branch = tmp_load_packet[i].uncond_branch;
                ex_packet.halt = tmp_load_packet[i].halt;
                ex_packet.illegal = tmp_load_packet[i].illegal;
                ex_packet.csr_op = tmp_load_packet[i].csr_op;

                ex_packet.function_type = tmp_load_packet[i].function_type;
                ex_packet.valid = tmp_load_packet[i].valid;

                ex_packet.rob_index = tmp_load_packet[i].rob_index;
                ex_packet.has_dest = tmp_load_packet[i].has_dest;

                ex_packet.issued_fu_index = tmp_load_packet[i].issued_fu_index;

                ex_packet.arch_dest_reg_num = tmp_load_packet[i].arch_dest_reg_num;
                ex_packet.prev_dword = 64'hdeadface;
            end
        end
        for (int i = 0; i < `NUM_FU_STORE; i++) begin
            if(store_done_process[i]) begin
                ex_packet.result        = tmp_store_result[i];
                ex_packet.take_branch   = 0;
                
                // Pass throughs
                ex_packet.inst = tmp_store_packet[i].inst;
                ex_packet.PC = tmp_store_packet[i].PC;
                ex_packet.NPC = tmp_store_packet[i].NPC;

                ex_packet.opa_select = tmp_store_packet[i].opa_select;
                ex_packet.opb_select = tmp_store_packet[i].opb_select;

                ex_packet.rs1_value = tmp_store_packet[i].rs1_value;
                ex_packet.rs2_value = tmp_store_packet[i].rs2_value;

                ex_packet.dest_reg_idx = tmp_store_packet[i].dest_reg_idx;

                ex_packet.alu_func = tmp_store_packet[i].alu_func;
                ex_packet.rd_mem = tmp_store_packet[i].rd_mem;
                ex_packet.wr_mem = tmp_store_packet[i].wr_mem;
                ex_packet.cond_branch = tmp_store_packet[i].cond_branch;
                ex_packet.uncond_branch = tmp_store_packet[i].uncond_branch;
                ex_packet.halt = tmp_store_packet[i].halt;
                ex_packet.illegal = tmp_store_packet[i].illegal;
                ex_packet.csr_op = tmp_store_packet[i].csr_op;

                ex_packet.function_type = tmp_store_packet[i].function_type;
                ex_packet.valid = tmp_store_packet[i].valid;

                ex_packet.rob_index = tmp_store_packet[i].rob_index;
                ex_packet.has_dest = tmp_store_packet[i].has_dest;
                ex_packet.mem_size = tmp_store_mem_size[i];

                ex_packet.issued_fu_index = tmp_store_packet[i].issued_fu_index;

                ex_packet.arch_dest_reg_num = tmp_store_packet[i].arch_dest_reg_num;
                ex_packet.prev_dword = tmp_store_prev_dword[i];
            end 
        end
        if (!alu_done_process && !mult_done_process && !branch_done_process && !load_done_process && !store_done_process) begin 
            ex_packet = INVALID_EX_CO_PACKET;
        end
    end

    // Chooses which FU to move forward to the next stage
    always_ff @(posedge clock) begin
        if(reset || rollback) begin
            alu_ready <= 0;
            tmp_alu_result          <= 0;
            mult_ready <= 0;
            tmp_mult_result         <= 0;
            branch_ready <= 0;
            tmp_branch_result       <= 0;
            load_ready <= 0;
            tmp_load_result         <= 0;
            store_ready <= 0;
            tmp_store_result <= 0;
            tmp_store_prev_dword <= 0;
            tmp_store_mem_size <= 0;
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
            tmp_mem_size <= 0;
            tmp_load_en <= 0;
            tmp_load2Dcache_addr <= 0;
        end
        else begin
            // If any of the FUs is outputting its down signal, add it to a list that signifies that that 
            // FU is ready to move forward to the next stage (waiting_fus)
            for(int i = 0; i < `NUM_FU_ALU; i++) begin
                if(alu_done[i]) begin
                    `ifdef DEBUG_PRINT
                    $display("[EX] ALU done, result=%h", alu_result[i]);
                    `endif
                    // waiting_fus[ALU]    <= 1;
                    alu_ready[i] <= 1;
                    tmp_alu_result[i]      <= alu_result[i];
                    tmp_alu_packet[i] <= alu_packet[i];
                end
            end

            for(int i = 0; i < `NUM_FU_MULT; i++) begin
                if(mult_done[i]) begin
                    mult_ready[i] <= 1;
                    tmp_mult_result[i]      <= mult_result[i];
                    tmp_mult_packet[i] <= mult_packet[i];
                end
            end

            for(int i = 0; i < `NUM_FU_BRANCH; i++) begin
                if(branch_done[i]) begin
                    branch_ready[i] <= 1;
                    tmp_branch_result[i]      <= branch_result[i];
                    tmp_take_conditional[i] <= take_conditional[i];
                    tmp_branch_packet[i] <= branch_packet[i];
                end
            end

            for(int i = 0; i < `NUM_FU_LOAD; i++) begin
                if(load_done[i]) begin
                    load_ready[i] <= 1;
                    tmp_load_result[i]      <= load_result[i];
                    tmp_load_packet[i] <= load_packet[i];
                end
            end

            for(int i = 0; i < `NUM_FU_STORE; i++) begin
                if(store_done[i]) begin
                    store_ready[i] <= 1;
                    tmp_store_result[i]      <= store_result[i];
                    tmp_store_prev_dword[i] <= prev_dword[i];
                    tmp_store_mem_size[i] <= store_mem_size[i];
                    tmp_store_packet[i] <= store_packet[i];
                end
            end

            // Defaults
            alu_done_process        <= 0;
            mult_done_process       <= 0;
            branch_done_process     <= 0;
            load_done_process <= 0;
            store_done_process <= 0;
            free_alu             <= 0;
            free_mult            <= 0;
            free_branch          <= 0;
            free_load <= 0;
            free_store <= 0;

            // Choose one of those FUs from waiting_fus. Set its passthroughs (in an always_comb block above), 
            // and set that free_[FU] variable to 1. Also remember to say that that FU is no longer ready to move on to the
            // next stage. 
            if(alu_ready != 0) begin
                for(int i = 0; i < `NUM_FU_ALU; i++) 
                    if (alu_ready[i]) begin
                        $display("[EX] ALU done, result=%h", tmp_alu_result[i]);
                        alu_ready[i] <= 0;
                        alu_done_process[i] <= 1;
                        free_alu[i] <= 1;
                        break;
                    end
            end else if(mult_ready != 0) begin
                for(int i = 0; i < `NUM_FU_MULT; i++) 
                    if (mult_ready[i]) begin
                        mult_ready[i] <= 0;
                        mult_done_process[i] <= 1;
                        free_mult[i] <= 1;
                        break;
                    end
            end else if(branch_ready != 0) begin
                for(int i = 0; i < `NUM_FU_BRANCH; i++) 
                    if (branch_ready[i]) begin
                        branch_ready[i] <= 0;
                        branch_done_process[i] <= 1;
                        free_branch[i] <= 1;
                        break;
                    end
            end else if (load_ready != 0) begin
                for(int i = 0; i < `NUM_FU_LOAD; i++) 
                    if (load_ready[i]) begin
                        load_ready[i] <= 0;
                        load_done_process[i] <= 1;
                        free_load[i] <= 1;
                        break;
                    end
            end else if (store_ready != 0) begin
                for(int i = 0; i < `NUM_FU_STORE; i++) 
                    if (store_ready[i]) begin
                        store_ready[i] <= 0;
                        store_done_process[i] <= 1;
                        free_store[i] <= 1;
                        break;
                    end
            end 
        end
    end     

endmodule // stage_ex
