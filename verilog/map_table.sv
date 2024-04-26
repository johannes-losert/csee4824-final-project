
// TODO figure out what to do on conflicts with restore operation (what happens if set CDB and restore?)

module map_table (
    input logic clk,
    input logic reset,
    // GET operation 1 (opa)
    input logic [`REG_IDX_SZ:0] arch_reg1_idx,
    output PREG preg1_out,

    // GET operation 2 (opb)
    input logic [`REG_IDX_SZ:0] arch_reg2_idx,
    output PREG preg2_out,

    // GET/SET operation (get old dest, replace with new dest)
    input logic [`REG_IDX_SZ:0] arch_dest_idx, 
    input logic set_dest_enable, /* Enable to overwrite dest with new pr */
    input logic [`PHYS_REG_IDX_SZ:0] new_dest_pr_idx,
    output PREG old_dest_pr,

    // GET physical register from architecture map 
    input logic [`REG_IDX_SZ:0] arch_map_arch_reg_idx,
    output PREG arch_map_phys_reg_out,

    // Retire operation (copies this preg to retired_preg)
    input logic [`REG_IDX_SZ:0] retire_arch_reg,
    input logic retire_enable,
    
    // Restore operation (copies all retired_pregs back to preg)
    input logic restore_enable,

    // SET READY operation (CDB)
    input logic set_ready_enable,
    input logic [`PHYS_REG_IDX_SZ:0] ready_phys_idx
);

    // One PREG per arch reg, always ready
    PREG [`REG_IDX_SZ:1] preg_entries;
    PREG [`REG_IDX_SZ:1] retired_preg_entries;
    PREG temp_old_dest_pr;

    // Read port 1
    always_comb begin 
        if (arch_reg1_idx == `ZERO_REG) begin
            preg1_out.ready = 1;
            preg1_out.reg_num = `ZERO_REG;
        end else begin 
             // Forwarding; if CDB broadcasting to the PR we're reading from, use its value for the ready bit
             // Note: still need to update the ready bit in the PR
            if (set_ready_enable && (ready_phys_idx == preg_entries[arch_reg1_idx].reg_num)) begin
                preg1_out.ready = 1;
            end else begin
                preg1_out.ready = preg_entries[arch_reg1_idx].ready;
            end 

            preg1_out.reg_num = preg_entries[arch_reg1_idx].reg_num;
        end
    end 

    // Read port 2
    always_comb begin 
        if (arch_reg2_idx == `ZERO_REG) begin
            preg2_out.ready = 1;
            preg2_out.reg_num = `ZERO_REG;
        end else begin 
            if (set_ready_enable && (ready_phys_idx == preg_entries[arch_reg2_idx].reg_num)) begin
                preg2_out.ready = 1;
            end else begin
                preg2_out.ready = preg_entries[arch_reg2_idx].ready;
            end 

            preg2_out.reg_num = preg_entries[arch_reg2_idx].reg_num;
        end
    end

    // Arch map read port 
    always_comb begin 
        if (arch_map_arch_reg_idx == `ZERO_REG) begin
            arch_map_phys_reg_out.ready = 1;
            arch_map_phys_reg_out.reg_num = `ZERO_REG;
        end else begin 
            if (retire_enable && (retire_arch_reg == arch_map_arch_reg_idx)) begin
                arch_map_phys_reg_out = preg_entries[arch_map_arch_reg_idx];
            end else begin
                arch_map_phys_reg_out = retired_preg_entries[arch_map_arch_reg_idx];
            end
        end
    end

    // GET old dest (TODO test that new dest doesn't corrupt this)
    always_comb begin 
        if (arch_dest_idx == `ZERO_REG) begin
            old_dest_pr.ready = 1;
            old_dest_pr.reg_num = `ZERO_REG;
        end else begin 
            if (set_ready_enable && (ready_phys_idx == temp_old_dest_pr.reg_num)) begin
                old_dest_pr.ready = 1;
            end else begin
                old_dest_pr.ready = temp_old_dest_pr.ready;
            end 

            old_dest_pr.reg_num = temp_old_dest_pr.reg_num;
        end
    end 

    function void print_mt_entry(int arch_idx, PREG entry); 
        $write("%0d \t| ", arch_idx);
        print_preg(entry);
        $write(" \t|");
        // $write(" %0d \t| %0d \t|", arch_idx, entry.reg_num);
        // if (entry.ready) begin
        //     $write(" Y \t|");
        // end else begin
        //     $write(" N \t|");
        // end
    endfunction

    function void print_map_table();
        $display("MAP TABLE");
        $display("opa: req_idx=%0d, preg_num=%0d, ready=%0d", arch_reg1_idx, preg1_out.reg_num, preg1_out.ready);
        $display("opb: req_idx=%0d, preg_num=%0d, ready=%0d", arch_reg2_idx, preg2_out.reg_num, preg2_out.ready);
        $display("dst: req_idx=%0d, preg_num=%0d, ready=%0d", arch_dest_idx, old_dest_pr.reg_num, old_dest_pr.ready);
        $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
        $display(" Arch \t| Phys \t| Arch \t| Phys \t|");
        for (int i = 0; i < (`REG_SZ/4); i++) begin
            print_mt_entry(i, preg_entries[i]);
            print_mt_entry(i + (`REG_SZ/4), preg_entries[i + (`REG_SZ/4)]);
            print_mt_entry(i + (`REG_SZ/2), preg_entries[i + (`REG_SZ/2)]);
            print_mt_entry(i + 3*(`REG_SZ/4), preg_entries[i + 3*(`REG_SZ/4)]);
            $display("");
        end
    endfunction

    function void print_arch_table();
        $display("ARCH MAP");
        $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
        $display(" Arch \t| Phys \t| Arch \t| Phys \t|");
        for (int i = 0; i < (`REG_SZ/4); i++) begin
            print_mt_entry(i, retired_preg_entries[i]);
            print_mt_entry(i + (`REG_SZ/4), retired_preg_entries[i + (`REG_SZ/4)]);
            print_mt_entry(i + (`REG_SZ/2), retired_preg_entries[i + (`REG_SZ/2)]);
            print_mt_entry(i + 3*(`REG_SZ/4), retired_preg_entries[i + 3*(`REG_SZ/4)]);
            $display("");
        end
        // $write(" Arch \t| Phys \t| Rdy \t| Arch \t| Phys \t| Rdy \t|");
        // $display(" Arch \t| Phys \t| Rdy \t| Arch \t| Phys \t| Rdy \t|");
        // for (int i = 0; i < (`REG_SZ/4); i++) begin
        //     print_mt_entry(i, preg_entries[i]);
        //     print_mt_entry(i + (`REG_SZ/4), preg_entries[i + (`REG_SZ/4)]);
        //     print_mt_entry(i + (`REG_SZ/2), preg_entries[i + (`REG_SZ/2)]);
        //     print_mt_entry(i + 3*(`REG_SZ/4), preg_entries[i + 3*(`REG_SZ/4)]);
        //     $display("");
        // end
    endfunction

    always @(negedge clk) begin 
        print_map_table();
        print_arch_table();
    end

    always @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < `PHYS_REG_SZ; i++) begin
                // TODO make sure this is right
                preg_entries[i].reg_num <= 0;
                preg_entries[i].ready <= 0;

                retired_preg_entries[i].reg_num <= 0;
                retired_preg_entries[i].ready <= 0;
            end
        end else begin 
            // Store for forwarding (in case new dest overwrites)
            temp_old_dest_pr <= preg_entries[arch_dest_idx];
            
            // SET new dest
            if (set_dest_enable && (arch_dest_idx != `ZERO_REG)) begin
                preg_entries[arch_dest_idx].reg_num <= new_dest_pr_idx;
                // TODO do I need to forward ready bit from CDB? probably not
                preg_entries[arch_dest_idx].ready <= 0;
            end 

            // Update from CDB
            if (set_ready_enable) begin
                preg_entries[ready_phys_idx].ready <= 1;
            end


            // If retiring, copy preg to retired_pre
            if (retire_enable) begin
                retired_preg_entries[retire_arch_reg] <= preg_entries[retire_arch_reg];
            end

            // If restoring, copy all retired_pregs back to preg
            if (restore_enable) begin
                for (int i = 0; i < `PHYS_REG_SZ; i++) begin
                    preg_entries[i] <= retired_preg_entries[i];
                end
            end

        end
    end
endmodule