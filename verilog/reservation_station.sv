
`include "verilog/sys_defs.svh"

// TODO: maybe add enable?
module reservation_station (
    input clock, reset, 
   
    /* Allocating */
    input allocate,
    input RS_PACKET input_packet,
    output logic done,

    /* Updating given PREG (from CDB) */
    input update, 
    input PREG ready_reg,

    /* Issuing */
    input logic issue_enable,
    output logic ready,
    output RS_PACKET issued_packet,
    output logic [`MAX_FU_INDEX-1:0] issue_fu_index,

    /* Output indicating whether each type of functional unit entry is full */
    output logic alu_entries_full,
    output logic mult_entries_full,
    output logic load_entries_full,
    output logic store_entries_full,
    output logic branch_entries_full,

    /* Freeing */
    input [`NUM_FU_ALU-1:0]free_alu,
    input [`NUM_FU_MULT-1:0]free_mult,
    input [`NUM_FU_LOAD-1:0]free_load,
    input [`NUM_FU_STORE-1:0]free_store,
    input [`NUM_FU_BRANCH-1:0]free_branch
    // input logic [`MAX_FU_INDEX-1:0] free_fu_index,
    // input FUNIT free_funit
);

    /* Reservation Station Entries */
    RS_ENTRY alu_entries[`NUM_FU_ALU-1:0];
    logic [$clog2(`NUM_FU_ALU)-1:0] alu_available_index;
    logic alu_available_index_found;
    logic [$clog2(`NUM_FU_ALU)-1:0] alu_issue_index;
    logic alu_issuable;

    RS_ENTRY mult_entries[`NUM_FU_MULT-1:0];
    logic [$clog2(`NUM_FU_MULT)-1:0] mult_available_index;
    logic mult_available_index_found;
    logic [$clog2(`NUM_FU_MULT)-1:0] mult_issue_index;
    logic mult_issuable;

    RS_ENTRY load_entries[`NUM_FU_LOAD-1:0];
    logic [$clog2(`NUM_FU_LOAD)-1:0] load_available_index;
    logic load_available_index_found;
    logic [$clog2(`NUM_FU_LOAD)-1:0] load_issue_index;
    logic load_issuable;

    RS_ENTRY store_entries[`NUM_FU_STORE-1:0];
    logic [$clog2(`NUM_FU_STORE)-1:0] store_available_index;
    logic store_available_index_found;
    logic [$clog2(`NUM_FU_STORE)-1:0] store_issue_index;
    logic store_issuable;

    RS_ENTRY branch_entries[`NUM_FU_BRANCH-1:0];
    logic [$clog2(`NUM_FU_BRANCH)-1:0] branch_available_index;
    logic branch_available_index_found;
    logic [$clog2(`NUM_FU_BRANCH)-1:0] branch_issue_index;
    logic branch_issuable;

   // combinational: calculate issuing output (searches for ready operands)
      // check all entries for first with both operands ready 
    always_comb begin : issuing
        // TODO better logic for picking packet to issue 
        alu_issuable = 1'b0;
        alu_issue_index = 0; // TODO this is really undefined

        alu_entries_full = 1'b1;
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            if (!alu_entries[i].busy) begin
                alu_entries_full = 1'b0;
            end
            if (~alu_entries[i].issued && alu_entries[i].packet.src1_reg.ready && alu_entries[i].packet.src2_reg.ready) begin
                alu_issuable = 1'b1;
                alu_issue_index = i;
            end
        end
        mult_issuable = 1'b0;
        mult_issue_index = 0; // TODO this is really undefined

        mult_entries_full = 1'b1;
        for (int i = 0; i < `NUM_FU_MULT; i++) begin

            if (!mult_entries[i].busy) begin
                mult_entries_full = 1'b0;
            end

            if (~mult_entries[i].issued && mult_entries[i].packet.src1_reg.ready && mult_entries[i].packet.src2_reg.ready) begin
                mult_issuable = 1'b1;
                mult_issue_index = i;
            end
        end
        load_issuable = 1'b0;
        load_issue_index = 0; // TODO this is really undefined

        load_entries_full = 1'b1;
        for (int i = 0; i < `NUM_FU_LOAD; i++) begin
            if (~load_entries[i].issued && load_entries[i].packet.src1_reg.ready && load_entries[i].packet.src2_reg.ready) begin
                load_issuable = 1'b1;
                load_issue_index = i;
            end

            if (!load_entries[i].busy) begin
                load_entries_full = 1'b0;
            end
        end

        store_issuable = 1'b0;
        store_issue_index = 0; // TODO this is really undefined

        store_entries_full = 1'b1;
        for (int i = 0; i < `NUM_FU_STORE; i++) begin
            if (!store_entries[i].busy) begin
                store_entries_full = 1'b0;
            end
            if (~store_entries[i].issued && store_entries[i].packet.src1_reg.ready && store_entries[i].packet.src2_reg.ready) begin
                store_issuable = 1'b1;
                store_issue_index = i;
            end
        end    

        branch_issuable = 1'b0;
        branch_issue_index = 0; // TODO this is really undefined

        branch_entries_full = 1'b1;
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            if (!branch_entries[i].busy) begin
                branch_entries_full = 1'b0;
            end
            if (~branch_entries[i].issued && branch_entries[i].packet.src1_reg.ready && branch_entries[i].packet.src2_reg.ready) begin
                branch_issuable = 1'b1;
                branch_issue_index = i;
            end
        end   
    end  

    // combinational: calculating the next available index 
    always_comb begin 
        for(int i = `NUM_FU_ALU; i > 0; i--) begin
            if(!alu_entries[i - 1].busy) begin 
                alu_available_index = i - 1;
                alu_available_index_found = 1;
            end else begin 
                alu_available_index_found = 0;
            end
        end

        for(int i = `NUM_FU_MULT; i > 0; i--) begin
            if(!mult_entries[i - 1].busy) begin 
                mult_available_index = i - 1;
                mult_available_index_found = 1;
            end else begin 
                mult_available_index_found = 0;
            end
        end

        for(int i = `NUM_FU_LOAD; i > 0; i--) begin
            if(!load_entries[i - 1].busy) begin 
                load_available_index = i - 1;
                load_available_index_found = 1;
            end else begin 
                load_available_index_found = 0;
            end
        end

        for(int i = `NUM_FU_STORE; i > 0; i--) begin
            if(!store_entries[i - 1].busy) begin 
                store_available_index = i - 1;
                store_available_index_found = 1;
            end else begin 
                store_available_index_found = 0;
            end
        end

        for(int i = `NUM_FU_BRANCH; i > 0; i--) begin
            if(!branch_entries[i - 1].busy) begin 
                branch_available_index = i - 1;
                branch_available_index = 1;
            end else begin 
                branch_available_index_found = 0;
            end
        end
    end


    always_ff @(posedge clock) begin
        if (reset) begin
            // TODO add valid bit to entries
            // TODO reset packets themeselves
            for (int i = 0; i < `NUM_FU_ALU; i++) begin
                alu_entries[i].busy <= 1'b0;
                alu_entries[i].issued <= 1'b1;
            end
            for (int i = 0; i < `NUM_FU_MULT; i++) begin
                mult_entries[i].busy <= 1'b0;
                mult_entries[i].issued <= 1'b1;
            end
            for (int i = 0; i < `NUM_FU_LOAD; i++) begin
                load_entries[i].busy <= 1'b0;
                load_entries[i].issued <= 1'b1;
            end
            for (int i = 0; i < `NUM_FU_STORE; i++) begin
                store_entries[i].busy <= 1'b0;
                store_entries[i].issued <= 1'b1;
            end
            for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
                branch_entries[i].busy <= 1'b0;
                branch_entries[i].issued <= 1'b1;
            end
        end else begin 
            
            // Updating (TODO might be one cycle delay)
            if(update) begin
                for (int i = 0; i < `NUM_FU_ALU; i++) begin
                    if(alu_entries[i].packet.src1_reg == ready_reg) begin
                        alu_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(alu_entries[i].packet.src2_reg == ready_reg) begin
                        alu_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_MULT; i++) begin
                    if(mult_entries[i].packet.src1_reg == ready_reg) begin
                        mult_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(mult_entries[i].packet.src2_reg == ready_reg) begin
                        mult_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_LOAD; i++) begin
                    if(load_entries[i].packet.src1_reg == ready_reg) begin
                        load_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(load_entries[i].packet.src2_reg == ready_reg) begin
                        load_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_STORE; i++) begin
                    if(store_entries[i].packet.src1_reg == ready_reg) begin
                        store_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(store_entries[i].packet.src2_reg == ready_reg) begin
                        store_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
                    if(branch_entries[i].packet.src1_reg == ready_reg) begin
                        branch_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(branch_entries[i].packet.src2_reg == ready_reg) begin
                        branch_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
            end     


            // Issuing 
            if (issue_enable) begin 
                if (alu_issuable) begin
                    ready <= 1'b1;
                    issued_packet <= alu_entries[alu_issue_index].packet;
                    alu_entries[alu_issue_index].issued <= 1;
                    issue_fu_index <= alu_issue_index;
                end else if (mult_issuable) begin
                    ready <= 1'b1;
                    issued_packet <= mult_entries[mult_issue_index].packet;
                    mult_entries[mult_issue_index].issued <= 1;
                    issue_fu_index <= mult_issue_index;
                end else if (load_issuable) begin
                    ready <= 1'b1;
                    issued_packet <= load_entries[load_issue_index].packet;
                    load_entries[load_issue_index].issued <= 1;
                    issue_fu_index <= load_issue_index;
                end else if (store_issuable) begin
                    ready <= 1'b1;
                    issued_packet <= store_entries[store_issue_index].packet;
                    store_entries[store_issue_index].issued <= 1;
                    issue_fu_index <= store_issue_index;
                end else if (branch_issuable) begin
                    ready <= 1'b1;
                    issued_packet <= branch_entries[branch_issue_index].packet;
                    branch_entries[branch_issue_index].issued <= 1;
                    issue_fu_index <= branch_issue_index;
                end else begin 
                    ready <= 1'b0;
                    issued_packet <= 0; // TODO this is really undefined 
                    issue_fu_index <= 0; // TODO this is really undefined
                end
            end else begin 
                ready <= 1'b0;
                issued_packet <= 0; // TODO this is really undefined 
                issue_fu_index <= 0; // TODO this is really undefined
            end


            // Freeing (TODO figure out of we can allocate and free on the same cycle)
            for(int i = 0; i < `NUM_FU_ALU; i++) begin
                if (free_alu[i]) begin
                    alu_entries[i].busy <= 0;
                    alu_entries[i].valid <= 0;
                    alu_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_MULT; i++) begin
                if (free_mult[i]) begin
                    mult_entries[i].busy <= 0;
                    mult_entries[i].valid <= 0;
                    mult_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_STORE; i++) begin
                if (free_store[i]) begin
                    store_entries[i].busy <= 0;
                    store_entries[i].valid <= 0;
                    store_entries[i].issued <= 1;
                end
            end


            for(int i = 0; i < `NUM_FU_LOAD; i++) begin
                if (free_load[i]) begin
                    load_entries[i].busy <= 0;
                    load_entries[i].valid <= 0;
                    load_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_BRANCH; i++) begin
                if (free_branch[i]) begin
                    branch_entries[i].busy <= 0;
                    branch_entries[i].valid <= 0;
                    branch_entries[i].issued <= 1;
                end
            end


            // Allocating
            // Try allocation 
            // find not busy entry in the four RS_ENTRYs
            if(allocate) begin 
                if(input_packet.funit == ALU) begin
                    if (alu_available_index_found) begin
                        alu_entries[alu_available_index].busy <= 1;
                        alu_entries[alu_available_index].packet <= input_packet;
                        alu_entries[alu_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        done <= 0;
                    end
                end else if(input_packet.funit == MULT) begin
                    if (mult_available_index_found) begin
                        mult_entries[mult_available_index].busy <= 1;
                        mult_entries[mult_available_index].packet <= input_packet;
                        mult_entries[mult_available_index].issued <= 0;
                        done <= 1;
                    end else begin
                        done <= 0;
                    end
                end else if(input_packet.funit == LOAD) begin
                    if (load_available_index_found) begin
                        load_entries[load_available_index].busy <= 1;
                        load_entries[load_available_index].packet <= input_packet;
                        load_entries[load_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        done <= 0;
                    end
                end else if(input_packet.funit == STORE) begin
                    if (store_available_index_found) begin
                        store_entries[store_available_index].busy <= 1;
                        store_entries[store_available_index].packet <= input_packet;
                        store_entries[store_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        done <= 0;
                    end
                end else if(input_packet.funit == BRANCH) begin
                    if (branch_available_index_found) begin
                        branch_entries[branch_available_index].busy <= 1;
                        branch_entries[branch_available_index].packet <= input_packet;
                        branch_entries[branch_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        done <= 0;
                    end
                end
            end else begin 
                done <= 0;
            end
        end 
    end
endmodule