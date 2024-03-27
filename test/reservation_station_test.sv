
`include "verilog/sys_defs.svh"


module testbench;

    logic clock, reset;

    logic issue_enable;

    logic allocate;
    RS_PACKET input_packet;

    logic update;
    PREG ready_reg;

    logic alloc_done;

    logic issue_ready;
    RS_PACKET issued_packet;
    logic [`MAX_FU_INDEX-1:0] issue_fu_index;


    logic [`NUM_FU_ALU-1:0] free_alu;
    logic [`NUM_FU_MULT-1:0] free_mult;
    logic [`NUM_FU_LOAD-1:0] free_load;
    logic [`NUM_FU_STORE-1:0] free_store;

    reservation_station dut(
        .clock(clock),
        .reset(reset),
        
        /* Allocate */
        .allocate(allocate),
        .input_packet(input_packet),

        .done(alloc_done),

        /* Update */
        .update(update),
        .ready_reg(ready_reg),

        /* Issue */
        .issue_enable(issue_enable),

        .ready(issue_ready),
        .issued_packet(issued_packet),
        .issue_fu_index(issue_fu_index),

        /* Free */
        .free_alu(free_alu),
        .free_mult(free_mult),
        .free_load(free_load),
        .free_store(free_store)
 
    );

    // CLOCK_PERIOD is defined on the commandline by the makefile
    always begin
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock;
    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin
        // Everything preg 1-4 number ready status
        $monitor("alloc: %h, input_funit: %h, input_dest: %h, input_src1: %h, input_src2: %h, alloc_done: %h, \
                issue_en: %h, issue_ready: %h, issued_funit: %h, issued_dest: %h, issued_src1: %h, issued_src2: %h, \
                update: %h, ready_reg.reg_num: %h, \
                free_alu: %h, free_mult: %h, free_load: %h, free_store: %h", 
                allocate, input_packet.funit, input_packet.dest_reg.reg_num, input_packet.src1_reg.reg_num, input_packet.src2_reg.reg_num, alloc_done, 
                issue_enable, issue_ready, issued_packet.funit, issued_packet.dest_reg.reg_num, issued_packet.src1_reg.reg_num, issued_packet.src2_reg.reg_num,
                update, ready_reg.reg_num,
                free_alu, free_mult, free_load, free_store);

        clock     = 0;
        reset     = 0;

        allocate = 0;
        input_packet = 0; // TODO fill in

        update = 0;
        ready_reg = 0; // TODO fill in

        issue_enable = 0;

        free_alu = 0;
        free_load = 0;
        free_mult = 0;
        free_store = 0;
        
        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        // cycle1
        allocate = 1;
        input_packet.funit = LOAD;
        input_packet.inst = 54;
        input_packet.dest_reg.reg_num = 5;
        input_packet.dest_reg.ready = 0;

        input_packet.src1_reg.reg_num = 0;
        input_packet.src1_reg.ready = 1;
        
        input_packet.src2_reg.reg_num = 4;
        input_packet.src2_reg.ready = 1;

        update = 0;
        ready_reg.reg_num = 0;
        ready_reg.ready = 0;

        issue_enable = 1;

        free_alu = 0;
        free_load = 0;
        free_store = 0;
        free_mult = 0;
        
        @(negedge clock)
        assert(alloc_done) else exit_on_error;
        assert(!issue_ready) else exit_on_error;

        // cycle2
        allocate = 1;
        input_packet.funit = MULT;
        input_packet.inst = 64;
        input_packet.dest_reg.reg_num = 6;
        input_packet.dest_reg.ready = 0;

        input_packet.src1_reg.reg_num = 1;
        input_packet.src1_reg.ready = 1;
        
        input_packet.src2_reg.reg_num = 5;
        input_packet.src2_reg.ready = 0;

        update = 0;
        ready_reg.reg_num = 0;
        ready_reg.ready = 0;

        issue_enable = 1;

        free_alu = 0;
        free_load = 0;
        free_store = 0;
        free_mult = 0;

        @(negedge clock)
        assert(alloc_done) else exit_on_error;
        
        assert(issue_ready) else exit_on_error;
        assert(issued_packet.funit == LOAD) else exit_on_error;
        assert(issued_packet.inst == 54) else exit_on_error;
        assert(issued_packet.dest_reg.reg_num == 5) else exit_on_error;
        assert(issued_packet.src1_reg.reg_num == 0) else exit_on_error;
        assert(issued_packet.src2_reg.reg_num == 4) else exit_on_error;
        assert(issue_fu_index == 0) else exit_on_error;
        

        // cycle3
        allocate = 1;
        input_packet.funit = STORE;
        input_packet.inst = 74;
        input_packet.dest_reg.reg_num = 0;
        input_packet.dest_reg.ready = 1;

        input_packet.src1_reg.reg_num = 6;
        input_packet.src1_reg.ready = 0;
        
        input_packet.src2_reg.reg_num = 4;
        input_packet.src2_reg.ready = 1;
        
        update = 0;
        ready_reg.reg_num = 0;
        ready_reg.ready = 0;

        issue_enable = 1;

        free_alu = 0;
        free_load = 1;
        free_store = 0;
        free_mult = 0;
        
        @(negedge clock)
        assert(alloc_done) else exit_on_error;

        assert(!issue_ready) else exit_on_error;

        
        // cycle4
        allocate = 1;
        input_packet.funit = ALU;
        input_packet.inst = 34;
        input_packet.dest_reg.reg_num = 7;
        input_packet.dest_reg.ready = 0;

        input_packet.src1_reg.reg_num = 4;
        input_packet.src1_reg.ready = 1;
        
        input_packet.src2_reg.reg_num = 0;
        input_packet.src2_reg.ready = 1;

        update = 1;
        ready_reg.reg_num = 5;
        ready_reg.ready = 1;

        issue_enable = 1;

        free_alu = 0;
        free_load = 0;
        free_store = 0;
        free_mult = 0;

        @(negedge clock)

        assert(alloc_done) else exit_on_error;

        assert(!issue_ready) else exit_on_error;

//slides cycle 5 instruction 
        allocate = 1;
        input_packet.funit = LOAD;
        input_packet.inst = 96;
        input_packet.dest_reg.reg_num = 8;
        input_packet.dest_reg.ready = 0;

        input_packet.src1_reg.reg_num = 0;
        input_packet.src1_reg.ready = 1;
        
        input_packet.src2_reg.reg_num = 7;
        input_packet.src2_reg.ready = 0;

        update = 0;
        ready_reg.reg_num = 5;
        ready_reg.ready = 1;

        issue_enable = 1;

        free_alu = 0;
        free_load = 0;
        free_store = 0;
        free_mult = 1;

        @(negedge clock)

        assert(alloc_done) else exit_on_error;

        assert(issue_ready) else exit_on_error;
        assert(issued_packet.funit == ALU) else exit_on_error;
        assert(issued_packet.inst == 34) else exit_on_error;
        assert(issued_packet.dest_reg.reg_num == 7) else exit_on_error;
        assert(issued_packet.src1_reg.reg_num == 4) else exit_on_error;
        assert(issued_packet.src2_reg.reg_num == 0) else exit_on_error;
        assert(issue_fu_index == 0) else exit_on_error;
        
//slides cycle 6 instruction 
        allocate = 0;
        input_packet.funit = LOAD;
        input_packet.inst = 96;
        input_packet.dest_reg.reg_num = 8;
        input_packet.dest_reg.ready = 0;

        input_packet.src1_reg.reg_num = 0;
        input_packet.src1_reg.ready = 1;
        
        input_packet.src2_reg.reg_num = 7;
        input_packet.src2_reg.ready = 0;

        update = 0;
        ready_reg.reg_num = 5;
        ready_reg.ready = 1;

        issue_enable = 0;

        free_alu = 1;
        free_load = 1;
        free_store = 1;
        free_mult = 1;

        @(negedge clock)

        assert(!alloc_done) else exit_on_error;

        assert(!issue_ready) else exit_on_error;
        assert(issued_packet.funit == ALU) else exit_on_error;
        
        
        $display("@@@Passed");
        $finish;
    end 
endmodule
/*
        // Check that all elements start invalid
        @(negedge clock);
        assert(!hit) else exit_on_error;

        // Initialize the memory
        command = WRITE;
        enable = 1;
        for(int i=0; i<`CAM_SIZE; i++) begin
            write_idx = i;
            data = $random;
            @(negedge clock);
        end

        // Overwrite memory locations with new data
        for(int i=0; i<(2**$clog2(`CAM_SIZE)); i++) begin
            write_idx = i;
            data = i;
            @(negedge clock);
        end

        // Read back data
        command = READ;
        for(int i=0; i<`CAM_SIZE; i++) begin
            data = i;
            @(negedge clock);
            assert(hit && read_idx == i) else exit_on_error;
        end

        // Check the size of read data
        data = `CAM_SIZE;
        @(negedge clock);
        assert(!hit) else exit_on_error;

        // And again with random values
        command   = WRITE;
        data      = $random;
        write_idx = 0;
        @(negedge clock);
        repeat (5) begin
            write_idx = $random;
            @(negedge clock);
        end

        command = READ;
        @(negedge clock);
        assert(read_idx == 0) else exit_on_error;

        $display("@@@Passed");
        $finish;
    end

    assign cres = a * b;
    assign correct = ~done || (cres === result);


    always @(posedge clock) begin
        #(`CLOCK_PERIOD*0.2); // a short wait to let signals stabilize
        if (!correct) begin
            $display("@@@ Incorrect at time %4.0f", $time);
            $display("@@@ done:%b a:%h b:%h result:%h", done, a, b, result);
            $display("@@@ Expected result:%h", cres);
            $finish;
        end
    end


    // Some students have had problems just using "@(posedge done)" because their
    // "done" signals glitch (even though they are the output of a register). This
    // prevents that by making sure "done" is high at the clock edge.
    task wait_until_done;
        forever begin : wait_loop
            @(posedge done);
            @(negedge clock);
            if (done) begin
                disable wait_until_done;
            end
        end
    endtask


    initial begin
        // NOTE: monitor starts using 5-digit decimal values for printing
        $monitor("Time:%4.0f done:%b a:%5d b:%5d result:%5d correct:%5d",
                 $time, done, a, b, result, cres);

        $display("\nBeginning edge-case testing:");

        reset = 1;
        clock = 0;
        a = 2;
        b = 3;
        start = 1;
        @(negedge clock);
        reset = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 5;
        b = 50;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 0;
        b = 257;
        @(negedge clock);
        start = 0;
        wait_until_done();

        // change the monitor to hex for these values
        $monitor("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                 $time, done, a, b, result, cres);

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 64'hFFFF_FFFF_FFFF_FFFF;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 3;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'hFFFF_FFFF_FFFF_FFFF;
        b = 0;
        @(negedge clock);
        start = 0;
        wait_until_done();

        start = 1;
        a = 64'h5555_5555_5555_5555;
        b = 64'hCCCC_CCCC_CCCC_CCCC;
        @(negedge clock);
        start = 0;
        wait_until_done();

        $monitor(); // turn off monitor for the for-loop
        $display("\nBeginning random testing:");

        for (i = 0; i <= 15; i = i+1) begin
            start = 1;
            a = {$random, $random}; // multiply random 64-bit numbers
            b = {$random, $random};
            @(negedge clock);
            start = 0;
            wait_until_done();
            $display("Time:%4.0f done:%b a:%h b:%h result:%h correct:%h",
                     $time, done, a, b, result, cres);
        end

        $display("@@@ Passed\n");
        $finish;
        
    end

endmodule

*/