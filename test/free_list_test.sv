
`include "verilog/sys_defs.svh"


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

    free_list dut(
        .clk(clock),
        .reset(reset),
        
        .dequeue_en(dequeue_en),
        .dequeue_pr(dequeue_pr),
        .was_dequeued(was_dequeued),

        .enqueue_en(enqueue_en),
        .enqueue_pr(enqueue_pr),
        .was_enqueued(was_enqueued)       
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
        enqueue_en=%b, enqueue_pr=%h, was_enqueued=%b", clock, reset, dequeue_en, 
        dequeue_pr, was_dequeued, enqueue_en, enqueue_pr, was_enqueued);

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


        // Test3: Enqueue 2, Dequeue 1, Enqueue 3
        dequeue_en = 1;
        enqueue_en = 1;
        enqueue_pr = 2;
        
        @(negedge clock)
        assert (was_enqueued == 1) else exit_on_error;
        assert (was_dequeued == 1) else exit_on_error;
        assert (dequeue_pr == 2) else exit_on_error;

        // dequeue_en = 1;
        // enqueue_en = 1;
        // enqueue_pr = 3;
        
        // @(negedge clock)
        // assert (was_enqueued == 1) else exit_on_error;
        // assert (was_dequeued == 1) else exit_on_error;

        // // Test4: Enqueue 4, Enqueue 5, Dequeue 2
        // dequeue_en = 0;
        // enqueue_en = 1;
        // enqueue_pr = 4;
        
        // @(negedge clock)
        // assert (was_enqueued == 1) else exit_on_error;
        // assert (was_dequeued == 0) else exit_on_error;

        // enqueue_en = 1;
        // enqueue_pr = 5;
        
        // @(negedge clock)
        // assert (was_enqueued == 1) else exit_on_error;
        // assert (was_dequeued == 0) else exit_on_error;

        // dequeue_en = 1;
        
        // @(negedge clock)
        // assert (was_enqueued == 0) else exit_on_error;
        // assert (was_dequeued == 1) else exit_on_error;


        $display("@@@Passed");
        $finish;
    end 
endmodule