
`include "verilog/sys_defs.svh"

// TODO add assertions to test peeking at the tail
module testbench;

    logic clock, reset;

    // READ from head 
    logic dequeue_en;
    logic [`PHYS_REG_IDX_SZ:0] dequeue_pr;
    logic was_dequeued;

    // WRITE to tail
    logic enqueue_en;
    logic [`PHYS_REG_IDX_SZ:0] enqueue_pr;
    logic was_enqueued;

    logic is_empty;
    logic is_full;  

    logic [`PHYS_REG_IDX_SZ:0] free_list_raw[`PHYS_REG_SZ+1];

    free_list dut(
        .clk(clock),
        .reset(reset),
        
        .dequeue_en(dequeue_en),
        .dequeue_pr(dequeue_pr),
        .was_dequeued(was_dequeued),

        .enqueue_en(enqueue_en),
        .enqueue_pr(enqueue_pr),
        .was_enqueued(was_enqueued),

        .is_empty(is_empty),
        .is_full(is_full),

        .free_list(free_list_raw)
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
        $monitor("clock=%b, reset=%b, dequeue_en=%b, dequeue_pr=%h, was_dequeued=%b, \
        enqueue_en=%b, enqueue_pr=%h, was_enqueued=%b, is_empty=%b, is_full=%b", clock, reset, dequeue_en, 
        dequeue_pr, was_dequeued, enqueue_en, enqueue_pr, was_enqueued, is_empty, is_full);

        clock     = 0;
        reset     = 0;

        
        dequeue_en = 0;
        enqueue_en = 0;
        
        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

       assert (was_dequeued == 0) else exit_on_error;
       assert (was_enqueued == 0) else exit_on_error;


        // Test3: Enqueue 5, Dequeue 1, Enqueue 3 (empty forwarding)
        dequeue_en = 1;
        enqueue_en = 1;
        enqueue_pr = 2;
        
        @(negedge clock)
        assert (was_enqueued == 1) else exit_on_error;
        assert (was_dequeued == 1) else exit_on_error;
        assert (dequeue_pr == 2) else exit_on_error;

        // Test enqueuing until full
        for (int i = 0; i < `PHYS_REG_SZ; i++) begin
            dequeue_en = 0;
            enqueue_en = 1;
            enqueue_pr = i;
            @(negedge clock)
            assert (was_enqueued == 1) else exit_on_error;
            assert (was_dequeued == 0) else exit_on_error;
            // for (int i = 0; i <= `FREE_LIST_SIZE; i++) begin
            //     $display("free_list[%0d] = %h", i, free_list_raw[i]);
            // end
        end

        // Test enqueueing while full (should fail)
        dequeue_en = 0;
        enqueue_en = 1;
        enqueue_pr = 5;
        @(negedge clock)
        assert (was_enqueued == 0) else exit_on_error;
        assert (was_dequeued == 0) else exit_on_error;

        // Test enqueuing and dequeuing while full (should work)
        dequeue_en = 1;
        enqueue_en = 1;
        enqueue_pr = `PHYS_REG_SZ;
        @(negedge clock)
        assert (was_enqueued == 1) else exit_on_error;
        assert (was_dequeued == 1) else exit_on_error;
        assert (dequeue_pr == 0) else exit_on_error; /* Dequeued the first value */

        // Test dequuing until empty
        for (int i = 1; i <= `PHYS_REG_SZ; i++) begin
            dequeue_en = 1;
            enqueue_en = 0;
            @(negedge clock)
            assert (was_enqueued == 0) else exit_on_error;
            assert (was_dequeued == 1) else exit_on_error;
            assert (dequeue_pr == i) else exit_on_error;
        end

        // Test dequeuing while empty (should fail)
        dequeue_en = 1;
        enqueue_en = 0;
        @(negedge clock)
        assert (was_enqueued == 0) else exit_on_error;
        assert (was_dequeued == 0) else exit_on_error;

        $display("@@@Passed");
        $finish;
    end 
endmodule