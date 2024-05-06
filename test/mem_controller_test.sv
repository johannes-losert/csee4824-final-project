`include "verilog/sys_defs.svh"
`include "test/mem.sv"


module testbench; 

    logic clock, reset; 

    // icache request
    BUS_COMMAND       icache_command;
    logic [`XLEN-1:0] icache_addr;
    
    // dcache request
    BUS_COMMAND dcache_command;
    logic [`XLEN-1:0] dcache_addr;

    // To memory
    BUS_COMMAND proc2mem_command;
    logic [`XLEN-1:0] proc2mem_addr;
    
    // From memory
    logic [3:0]  mem2proc_response; // Should be zero unless there is a response
    logic [63:0] mem2proc_data;
    logic [3:0]  mem2proc_tag;

    // to caches (both are connected)
    logic [3:0]  control2cache_response; // Should be zero unless there is a response
    DEST_CACHE control2cache_response_which; // always either ICACHE, or DCACHE
    logic [63:0] control2cache_data; 
    logic [3:0]  control2cache_tag;
    DEST_CACHE control2cache_tag_which; 

    REQ_STATUS current_request_status;

    mem_controller dut (
        .clock(clock),
        .reset(reset),

        .icache_command(icache_command),
        .icache_addr(icache_addr),
        
        .dcache_command(dcache_command),
        .dcache_addr(dcache_addr),
        
        .proc2mem_command(proc2mem_command),
        .proc2mem_addr(proc2mem_addr),

        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag),

        .current_request_status(current_request_status),

        .control2cache_response(control2cache_response),
        .control2cache_response_which(control2cache_response_which),
        .control2cache_data(control2cache_data),
        .control2cache_tag(control2cache_tag),
        .control2cache_tag_which(control2cache_tag_which)
    );

    logic clk; // Memory clock

    mem mem_0 (
        .clk(clk),

        .proc2mem_addr(proc2mem_addr),
        .proc2mem_data(),
        // .proc2mem_size(proc2mem_size),
        .proc2mem_command(proc2mem_command),
        
        .mem2proc_response(mem2proc_response),
        .mem2proc_data(mem2proc_data),
        .mem2proc_tag(mem2proc_tag)
    );

    int i = 0;

    always begin       
        #(`CLOCK_PERIOD/2.0);
        clock = ~clock; 
        if (i % 2 * `MEM_LATENCY_IN_CYCLES == 0) begin
            clk = ~clk;
        end
        i = i + 1;
    end

    task exit_on_error;
        begin
            $display("@@@Failed at time %d", $time);
            $finish;
        end
    endtask

    initial begin 
        // $monitor();

        clock = 0;
        clk = 0;
        reset = 1;
        @(negedge clock)
        reset = 0;

        $display("Test case 1: Controller gets tag from mem.");
        icache_command = BUS_LOAD;
        icache_addr = 111;

        dcache_command = BUS_LOAD;
        dcache_addr = 222;
        
        // the current request status shouldn't update until the next clock cycle
        @(negedge clock)
        assert(current_request_status == AWAIT_TAG_DCACHE) else exit_on_error;

        @(posedge |mem2proc_response)
        $display("mem2proc_response: %d", mem2proc_response);
        assert(|mem2proc_response) else exit_on_error;

        $display("@@@Passed");
        $finish;
    end

endmodule