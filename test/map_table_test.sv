
`include "verilog/sys_defs.svh"


module testbench;

    logic clock, reset;

    // GET operations
    logic [`REG_IDX_SZ:0] opa_areg_idx, opb_areg_idx;
    PREG preg1_out, preg2_out;

    // GET/SET operation
    logic [`REG_IDX_SZ:0] dest_areg_idx;
    logic [`PHYS_REG_IDX_SZ:0] new_dest_preg_idx;
    logic set_dest_enable;
    PREG old_dest_preg;

    // SET READY operation (CDB)
    logic set_ready_enable;
    logic [`PHYS_REG_IDX_SZ:0] ready_preg_idx;

    map_table dut(
        .clock(clock),
        .reset(reset),
        
        .arch_reg1_idx(opa_areg_idx),
        .arch_reg2_idx(opb_areg_idx),
        .preg1_out(preg1_out),
        .preg2_out(preg2_out),

        .arch_dest_idx(dest_areg_idx),
        .set_dest_enable(set_dest_enable),
        .new_dest_pr(new_dest_preg_idx),
        .old_dest_pr(old_dest_preg),

        .set_ready_enable(set_ready_enable),
        .ready_phys_idx(ready_preg_idx) 
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
        $monitor("clock=%b, reset=%b, opa_areg_idx=%h, opb_areg_idx=%h, \
        preg1_out_reg_num=%h, preg1_out_ready=%b, preg2_out_reg_num=%h, \
        preg2_out_ready=%b, dest_areg_idx=%h, set_dest_enable=%b, \
        new_dest_preg_idx=%h, old_dest_pr_reg_num=%h, old_dest_pr_ready=%b, \
        set_ready_enable=%b, ready_preg_idx=%h", clock, reset, opa_areg_idx,
        opb_areg_idx, preg1_out.reg_num, preg1_out.ready, preg2_out.reg_num,
        preg2_out.ready, dest_areg_idx, set_dest_enable, new_dest_preg_idx,
        old_dest_preg.reg_num, old_dest_preg.ready, set_ready_enable,
        ready_preg_idx);

        clock     = 0;
        reset     = 0;

        
        opa_areg_idx = 0;
        opb_areg_idx = 0;

        dest_areg_idx = 0;

        set_dest_enable = 0;
        new_dest_preg_idx = 0;


        set_ready_enable = 0;
        ready_preg_idx = 0;
        
        // Initial Reset
        reset = 1;
        @(negedge clock)
        reset = 0;

        assert(preg1_out.reg_num == 0) else exit_on_error;
        assert(preg1_out.ready == 1) else exit_on_error;
        assert(preg2_out.reg_num == 0) else exit_on_error;
        assert(preg2_out.ready == 1) else exit_on_error;

        assert(old_dest_preg.reg_num == 0) else exit_on_error;
        assert(old_dest_preg.ready == 1) else exit_on_error;


        // Test0: write to MT (no CDB)
        opa_areg_idx = 0; 
        opb_areg_idx = 0; /* Read from zero reg */

        
        dest_areg_idx = 1; /* Update areg 1 to map to preg 1 */
        
        set_dest_enable = 1;
        new_dest_preg_idx = 1; /* Set to preg 1 */

        set_ready_enable = 0;
        ready_preg_idx = 0;
    
        @(negedge clock)
        assert(preg1_out.reg_num == 0) else exit_on_error;
        assert(preg1_out.ready == 1) else exit_on_error;
        assert(preg2_out.reg_num == 0) else exit_on_error;
        assert(preg2_out.ready == 1) else exit_on_error;

        /* Old dest reg undefined */ 

        
        $display("@@@Passed");
        $finish;
    end 
endmodule