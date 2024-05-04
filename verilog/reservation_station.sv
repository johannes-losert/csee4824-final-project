
`include "verilog/sys_defs.svh"

// TODO: maybe add enable?
module reservation_station (
    input clock, reset, 
   
    /* Allocating */
    input logic allocate,
    input ID_IS_PACKET input_packet,
    output logic done,

    /* Updating given PREG (from CDB) */
    input logic update, 
    input PREG ready_reg,

    /* Issuing */
    input logic issue_enable,
  //  output logic ready, Replaced with valid bit inside issued packet
    output ID_IS_PACKET issued_packet,

    /* From ROB, so branch knows if it is head */
    input logic [$clog2(`ROB_SZ)-1:0] rob_head_index,

    /* Output indicating whether each type of functional unit entry is full */
    output logic alu_entries_full,
    output logic mult_entries_full,
    output logic load_entries_full,
    output logic store_entries_full,
    output logic branch_entries_full,

    /* Freeing */
    input logic [`NUM_FU_ALU-1:0]free_alu,
    input logic [`NUM_FU_MULT-1:0]free_mult,
    input logic [`NUM_FU_LOAD-1:0]free_load,
    input logic [`NUM_FU_STORE-1:0]free_store,
    input logic [`NUM_FU_BRANCH-1:0]free_branch,
    // input logic [`MAX_FU_INDEX-1:0] free_fu_index,
    // input function_type free_function_type

    // DEBUG
    output logic alu_issuable,
    output logic mult_issuable,
    output logic load_issuable,
    output logic store_issuable,
    output logic branch_issuable

);

    /* Reservation Station Entries */
    RS_ENTRY alu_entries[`NUM_FU_ALU-1:0];
    logic [$clog2(`NUM_FU_ALU)-1:0] alu_available_index;
    logic alu_available_index_found;
    logic [$clog2(`NUM_FU_ALU)-1:0] alu_issue_index;
    //logic alu_issuable;

    RS_ENTRY mult_entries[`NUM_FU_MULT-1:0];
    logic [$clog2(`NUM_FU_MULT)-1:0] mult_available_index;
    logic mult_available_index_found;
    logic [$clog2(`NUM_FU_MULT)-1:0] mult_issue_index;
  //  logic mult_issuable;

    RS_ENTRY load_entries[`NUM_FU_LOAD-1:0];
    logic [$clog2(`NUM_FU_LOAD)-1:0] load_available_index;
    logic load_available_index_found;
    logic [$clog2(`NUM_FU_LOAD)-1:0] load_issue_index;
   // logic load_issuable;

    RS_ENTRY store_entries[`NUM_FU_STORE-1:0];
    logic [$clog2(`NUM_FU_STORE)-1:0] store_available_index;
    logic store_available_index_found;
    logic [$clog2(`NUM_FU_STORE)-1:0] store_issue_index;
    //logic store_issuable;

    RS_ENTRY branch_entries[`NUM_FU_BRANCH-1:0];
    logic [$clog2(`NUM_FU_BRANCH)-1:0] branch_available_index;
    logic branch_available_index_found;
    logic [$clog2(`NUM_FU_BRANCH)-1:0] branch_issue_index;
  //  logic branch_issuable;

    function void printRSEntry(int i, string fu, RS_ENTRY entry);
        $write("%0d \t|", i);
        $write(" %s \t|", fu);
        if (entry.busy) begin
            $write(" y \t|");
        end else begin
            $write(" n \t|");
        end

        if (entry.issued) begin
            $write(" y \t|");
        end else begin
            $write(" n \t|");
        end

        print_inst(entry.packet.inst, entry.packet.PC, entry.packet.valid);
        $write("\t| ");

        print_preg(entry.packet.dest_reg);

        $write(" \t| ");

        print_preg(entry.packet.src1_reg);

        $write(" \t| ");

        print_preg(entry.packet.src2_reg);

        $display("");

    endfunction

    // Function to print the current state of the ALU entries in the reservation station
    function void printReservationStation();
        $display("RESERVATION STATIONS");
        $display("Issuable? alu:%0d mult:%0d load:%0d store:%0d branch:%0d", alu_issuable, mult_issuable, load_issuable, store_issuable, branch_issuable);
        $display("Freeing?  alu:%0d mult:%0d load:%0d store:%0d branch:%0d", free_alu, free_mult, free_load, free_store, free_branch);
        $display("Full?     alu:%0d mult:%0d load:%0d store:%0d branch:%0d", alu_entries_full, mult_entries_full, load_entries_full, store_entries_full, branch_entries_full);
        // $write("Issuing: ");
        // print_inst(issued_packet.inst, issued_packet.PC, issued_packet.valid);
        // $display("");
        $display("NUM \t| FU \t| BUSY \t ISS'D \t| OP \t\t| DEST \t| SRC1 \t| SRC2 |");
        for (int i = 0; i < `NUM_FU_ALU; i++) begin
            printRSEntry(i + 1, "ALU", alu_entries[i]);
        end
        for (int i = 0; i < `NUM_FU_MULT; i++) begin
            printRSEntry(i + `NUM_FU_ALU + 1, "MULT", mult_entries[i]);
        end
        for (int i = 0; i < `NUM_FU_LOAD; i++) begin
            printRSEntry(i + `NUM_FU_ALU + `NUM_FU_MULT + 1, "LD", load_entries[i]);
        end
        for (int i = 0; i < `NUM_FU_STORE; i++) begin
            printRSEntry(i + `NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LOAD + 1, "ST", store_entries[i]);
        end
        for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
            printRSEntry(i + `NUM_FU_ALU + `NUM_FU_MULT + `NUM_FU_LOAD + `NUM_FU_STORE + 1, "BRN", branch_entries[i]);
        end
    endfunction
            
            /* $write("%0d \t|", i);
            $write(" ALU \t|");
            if (alu_entries[i].busy) begin
                $write(" yes \t|");
            end else begin
                $write(" no \t|");
            end
            print_inst(alu_entries[i].packet.inst, alu_entries[i].packet.PC, alu_entries[i].packet.valid);
            $write("\t|");
            $write(" %d \t|", alu_entries[i].packet.dest_reg);
            $write(" %d \t|", alu_entries[i].packet.src1_reg);
            $write(" %d \t|", alu_entries[i].packet.src2_reg);
            $display(""); */
  //      end
//    endfunction


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
                /* Branches should wait for everything in front of them to commit before issuing */
                if (branch_entries[i].packet.rob_index == rob_head_index) begin
                    branch_issuable = 1'b1;
                    branch_issue_index = i;
                end
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
                branch_available_index_found = 1;
            end else begin 
                branch_available_index_found = 0;
            end
        end
    end

    always_ff @(negedge clock) begin 
        `ifdef DEBUG_PRINT
        printReservationStation();
        `endif
    end

    always_ff @(posedge clock) begin
       // printReservationStation();
        if (reset) begin
            // $display("[RS] resetting");
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
                `ifdef DEBUG_PRINT
                $display("[RS] CDB update, ready_reg:%d", ready_reg.reg_num);
                `endif
                for (int i = 0; i < `NUM_FU_ALU; i++) begin
                    if(alu_entries[i].packet.src1_reg.reg_num == ready_reg.reg_num) begin
                        alu_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(alu_entries[i].packet.src2_reg.reg_num == ready_reg.reg_num) begin
                        alu_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_MULT; i++) begin
                    if(mult_entries[i].packet.src1_reg.reg_num == ready_reg.reg_num) begin
                        mult_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(mult_entries[i].packet.src2_reg.reg_num == ready_reg.reg_num) begin
                        mult_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_LOAD; i++) begin
                    if(load_entries[i].packet.src1_reg.reg_num == ready_reg.reg_num) begin
                        load_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(load_entries[i].packet.src2_reg.reg_num == ready_reg.reg_num) begin
                        load_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_STORE; i++) begin
                    if(store_entries[i].packet.src1_reg.reg_num == ready_reg.reg_num) begin
                        store_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(store_entries[i].packet.src2_reg.reg_num == ready_reg.reg_num) begin
                        store_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
                for (int i = 0; i < `NUM_FU_BRANCH; i++) begin
                    if(branch_entries[i].packet.src1_reg.reg_num == ready_reg.reg_num) begin
                        branch_entries[i].packet.src1_reg.ready <= 1'b1;
                    end
                    if(branch_entries[i].packet.src2_reg.reg_num == ready_reg.reg_num) begin
                        branch_entries[i].packet.src2_reg.ready <= 1'b1;
                    end
                end
            end else begin 
                `ifdef DEBUG_PRINT
                $display("[RS] No CDB Update");
                `endif
            end   


            // Issuing 
            if (issue_enable) begin 
                if (alu_issuable) begin
                    `ifdef DEBUG_PRINT
                    $display("[RS] Issue ALU packet, pc:%p", alu_entries[alu_issue_index].packet.PC);
                    `endif
                 //   ready <= 1'b1;
                    issued_packet <= alu_entries[alu_issue_index].packet;
                    alu_entries[alu_issue_index].issued <= 1;
                    issued_packet.issued_fu_index <= alu_issue_index;
                end else if (mult_issuable) begin
              //      ready <= 1'b1;
                    issued_packet <= mult_entries[mult_issue_index].packet;
                    mult_entries[mult_issue_index].issued <= 1;
                    issued_packet.issued_fu_index <= mult_issue_index;
                end else if (load_issuable) begin
             //       ready <= 1'b1;
                    issued_packet <= load_entries[load_issue_index].packet;
                    load_entries[load_issue_index].issued <= 1;
                    issued_packet.issued_fu_index <= load_issue_index;
                end else if (store_issuable) begin
              //      ready <= 1'b1;
                    issued_packet <= store_entries[store_issue_index].packet;
                    store_entries[store_issue_index].issued <= 1;
                    issued_packet.issued_fu_index <= store_issue_index;
                end else if (branch_issuable) begin
               //     ready <= 1'b1;
                    issued_packet <= branch_entries[branch_issue_index].packet;
                    branch_entries[branch_issue_index].issued <= 1;
                    issued_packet.issued_fu_index <= branch_issue_index;
                end else begin 
              //      ready <= 1'b0;
                    `ifdef DEBUG_PRINT
                    $display("[RS] No packet to issue");
                    `endif
                    issued_packet <= INVALID_ID_IS_PACKET;

                end
            end else begin 
               issued_packet <= INVALID_ID_IS_PACKET;
            end

            // Freeing (TODO figure out of we can allocate and free on the same cycle)
            for(int i = 0; i < `NUM_FU_ALU; i++) begin
                if (free_alu[i]) begin
                    `ifdef DEBUG_PRINT
                    $display("[RS] Freeing ALU packet, pc:%p", alu_entries[i].packet.PC);
                    `endif
                    alu_entries[i].busy <= 0;
                    alu_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_MULT; i++) begin
                if (free_mult[i]) begin
                    mult_entries[i].busy <= 0;
                    mult_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_STORE; i++) begin
                if (free_store[i]) begin
                    store_entries[i].busy <= 0;
                    store_entries[i].issued <= 1;
                end
            end


            for(int i = 0; i < `NUM_FU_LOAD; i++) begin
                if (free_load[i]) begin
                    load_entries[i].busy <= 0;
                    load_entries[i].issued <= 1;
                end
            end

            for(int i = 0; i < `NUM_FU_BRANCH; i++) begin
                if (free_branch[i]) begin
                    branch_entries[i].busy <= 0;
                    branch_entries[i].issued <= 1;
                end
            end


            // Allocating
            // Try allocation 
            // find not busy entry in the four RS_ENTRYs
            if(allocate) begin 
                `ifdef DEBUG_PRINT
                $display("[RS] Trying to allocate packet, pc:%p", input_packet.PC);
                `endif
                if(input_packet.function_type == ALU) begin
                    if (alu_available_index_found) begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Found index, allocating new ALU packet, pc:%p", input_packet.PC);
                        `endif
                        alu_entries[alu_available_index].busy <= 1;
                        alu_entries[alu_available_index].packet <= input_packet;
                        alu_entries[alu_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        `ifdef DEBUG_PRINT
                        $display("[RS] Can't allocate ALU packet, pc:%p", input_packet.PC);
                        `endif
                        done <= 0;
                    end
                end else if(input_packet.function_type == MULT) begin
                    if (mult_available_index_found) begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Found index, allocating new MULT packet, pc:%p", input_packet.PC);
                        `endif
                        mult_entries[mult_available_index].busy <= 1;
                        mult_entries[mult_available_index].packet <= input_packet;
                        mult_entries[mult_available_index].issued <= 0;
                        done <= 1;
                    end else begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Can't allocate MULT packet, pc:%p", input_packet.PC);
                        `endif
                        done <= 0;
                    end
                end else if(input_packet.function_type == LOAD) begin
                    if (load_available_index_found) begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Found index, allocating new LOAD packet, pc:%p", input_packet.PC);
                        `endif
                        load_entries[load_available_index].busy <= 1;
                        load_entries[load_available_index].packet <= input_packet;
                        load_entries[load_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        `ifdef DEBUG_PRINT
                        $display("[RS] Can't allocate LOAD packet, pc:%p", input_packet.PC);
                        `endif
                        done <= 0;
                    end
                end else if(input_packet.function_type == STORE) begin
                    if (store_available_index_found) begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Found index, allocating new STORE packet, pc:%p", input_packet.PC);
                        `endif
                        store_entries[store_available_index].busy <= 1;
                        store_entries[store_available_index].packet <= input_packet;
                        store_entries[store_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        `ifdef DEBUG_PRINT
                        $display("[RS] Can't allocate STORE packet, pc:%p", input_packet.PC);
                        `endif
                        done <= 0;
                    end
                end else if(input_packet.function_type == BRANCH) begin
                    if (branch_available_index_found) begin
                        `ifdef DEBUG_PRINT
                        $display("[RS] Found index, allocating new BRANCH packet, pc:%p", input_packet.PC);
                        `endif
                        branch_entries[branch_available_index].busy <= 1;
                        branch_entries[branch_available_index].packet <= input_packet;
                        branch_entries[branch_available_index].issued <= 0;
                        done <= 1;
                    end else begin 
                        `ifdef DEBUG_PRINT
                        $display("[RS] Can't allocate BRANCH packet, pc:%p", input_packet.PC);
                        `endif
                        done <= 0;
                    end
                end else begin 
                        assert(0);
                        done <= 0;
                end
            end else begin 
                `ifdef DEBUG_PRINT
                $display("[RS] No allocation");
                $display("[RS] --");
                `endif
                done <= 0;
            end
        end 
    end
endmodule
