
/* Map table module stores physical register to architecture register mappings,
   also includes duplicate 'retire' copy for rollbacks */
module map_table (
    input logic clk,
    input logic reset,
    // GET operation 1 (rs1)
    input logic [`REG_IDX_SZ:0] arch_reg1_idx,
    output PREG preg1_out,

    // GET operation 2 (rs2)
    input logic [`REG_IDX_SZ:0] arch_reg2_idx,
    output PREG preg2_out,

    // GET/SET operation (get old dest, replace with new dest)
    input logic [`REG_IDX_SZ:0] arch_dest_idx,
    input logic set_dest_enable,  /* Enable to overwrite dest with new pr */
    input logic [`PHYS_REG_IDX_SZ:0] new_dest_pr_idx,
    output PREG old_dest_pr,

    // GET physical register from architecture map 
    input logic [`REG_IDX_SZ:0] arch_map_arch_reg_idx,
    output PREG arch_map_phys_reg_out,

    // Retire operation (copies this preg to retired_preg)
    input logic [`REG_IDX_SZ:0] retire_arch_reg,
    input PREG retire_phys_reg,
    input logic retire_enable,

    // Restore operation (copies all retired_pregs back to preg)
    input logic restore_enable,
    input logic [`REG_IDX_SZ:0] immune_reg_idx,  // Don't restore this register
    input PREG immune_preg,

    // SET READY operation (CDB)
    input logic set_ready_enable,
    input logic [`PHYS_REG_IDX_SZ:0] ready_phys_idx,

    // DEBUG

    output PREG [`REG_SZ] preg_entries,
    output PREG [`REG_SZ] retired_preg_entries,
    output PREG temp_old_dest_pr
);

  // Read port 1
  always_comb begin
    if (arch_reg1_idx == `ZERO_REG) begin
      preg1_out.ready   = 1;
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
      preg2_out.ready   = 1;
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
      arch_map_phys_reg_out.ready   = 1;
      arch_map_phys_reg_out.reg_num = `ZERO_REG;
    end else begin
      if (retire_enable && (retire_arch_reg == arch_map_arch_reg_idx)) begin
        // arch_map_phys_reg_out = preg_entries[arch_map_arch_reg_idx];
        arch_map_phys_reg_out = retire_phys_reg;
      end else begin
        arch_map_phys_reg_out = retired_preg_entries[arch_map_arch_reg_idx];
      end
    end
  end

  // GET old dest 
  always_comb begin
    if (arch_dest_idx == `ZERO_REG) begin
      old_dest_pr.ready   = 1;
      old_dest_pr.reg_num = `ZERO_REG;
    end else begin
      old_dest_pr = preg_entries[arch_dest_idx];
    end
  end

`ifdef DEBUG_PRINT
  function void print_mt_entry(int arch_idx, PREG entry);
    $write("%0d \t| ", arch_idx);
    print_preg(entry);
    $write(" \t|");
  endfunction

  function void print_map_table();
    $display("MAP TABLE");
    $display("rs1: req_idx=%0d, preg_num=%0d, ready=%0d", arch_reg1_idx, preg1_out.reg_num,
             preg1_out.ready);
    $display("rs2: req_idx=%0d, preg_num=%0d, ready=%0d", arch_reg2_idx, preg2_out.reg_num,
             preg2_out.ready);
    $display("dst: req_idx=%0d, old_preg_num=%0d, ready=%0d, new_preg_num", arch_dest_idx,
             old_dest_pr.reg_num, old_dest_pr.ready, new_dest_pr_idx);

    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $display(" Arch \t| Phys \t| Arch \t| Phys \t|");
    for (int i = 0; i < (`REG_SZ / 8); i++) begin
      print_mt_entry(i, preg_entries[i]);
      print_mt_entry(i + (`REG_SZ / 8), preg_entries[i+(`REG_SZ/8)]);
      print_mt_entry(i + (`REG_SZ / 4), preg_entries[i+(`REG_SZ/4)]);
      print_mt_entry(i + 3 * (`REG_SZ / 8), preg_entries[i+3*(`REG_SZ/8)]);
      print_mt_entry(i + 4 * (`REG_SZ / 8), preg_entries[i+4*(`REG_SZ/8)]);
      print_mt_entry(i + 5 * (`REG_SZ / 8), preg_entries[i+5*(`REG_SZ/8)]);
      print_mt_entry(i + 6 * (`REG_SZ / 8), preg_entries[i+6*(`REG_SZ/8)]);
      print_mt_entry(i + 7 * (`REG_SZ / 8), preg_entries[i+7*(`REG_SZ/8)]);
      $display("");
    end
  endfunction

  function void print_arch_table();
    $display("ARCH MAP");
    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $write(" Arch \t| Phys \t| Arch \t| Phys \t|");
    $display(" Arch \t| Phys \t| Arch \t| Phys \t|");
    for (int i = 0; i < (`REG_SZ / 8); i++) begin
      print_mt_entry(i, retired_preg_entries[i]);
      print_mt_entry(i + (`REG_SZ / 8), retired_preg_entries[i+(`REG_SZ/8)]);
      print_mt_entry(i + (`REG_SZ / 4), retired_preg_entries[i+(`REG_SZ/4)]);
      print_mt_entry(i + 3 * (`REG_SZ / 8), retired_preg_entries[i+3*(`REG_SZ/8)]);
      print_mt_entry(i + 4 * (`REG_SZ / 8), retired_preg_entries[i+4*(`REG_SZ/8)]);
      print_mt_entry(i + 5 * (`REG_SZ / 8), retired_preg_entries[i+5*(`REG_SZ/8)]);
      print_mt_entry(i + 6 * (`REG_SZ / 8), retired_preg_entries[i+6*(`REG_SZ/8)]);
      print_mt_entry(i + 7 * (`REG_SZ / 8), retired_preg_entries[i+7*(`REG_SZ/8)]);
      $display("");
    end
  endfunction

  always @(negedge clk) begin
    print_map_table();
    print_arch_table();
  end
`endif


  always @(posedge clk) begin
    if (reset) begin
      for (int i = 0; i < `PHYS_REG_SZ; i++) begin
        preg_entries[i].reg_num <= 0;
        preg_entries[i].ready <= 1;

        retired_preg_entries[i].reg_num <= 0;
        retired_preg_entries[i].ready <= 1;
      end
    end else begin
      // Store for forwarding (in case new dest overwrites)
      temp_old_dest_pr <= preg_entries[arch_dest_idx];

      // SET new dest
      if (set_dest_enable && (arch_dest_idx != `ZERO_REG)) begin
        preg_entries[arch_dest_idx].reg_num <= new_dest_pr_idx;
        preg_entries[arch_dest_idx].ready   <= 0;
      end

      // Update from CDB
      if (set_ready_enable) begin
        for (int i = 0; i < `REG_SZ; i++) begin
          if (preg_entries[i].reg_num == ready_phys_idx) begin
            preg_entries[i].ready <= 1;
          end
        end
      end


      // If retiring, upate retired preg entries 
      if (retire_enable) begin
        retired_preg_entries[retire_arch_reg].reg_num <= retire_phys_reg.reg_num;
        // If retiring and broadcast to CDB, update ready bit
        if (set_ready_enable && retire_phys_reg.reg_num == ready_phys_idx) begin
          retired_preg_entries[retire_arch_reg].ready <= 1;
        end else begin
          retired_preg_entries[retire_arch_reg].ready <= retire_phys_reg.ready;
        end
      end

      // If restoring, copy all retired_pregs back to preg
      if (restore_enable) begin
        for (int i = 0; i < `PHYS_REG_SZ; i++) begin
          if (i == immune_reg_idx) begin  // For JAL/JALR instructions
            preg_entries[i] <= immune_preg;
          end else begin
            preg_entries[i] <= retired_preg_entries[i];
          end
        end
      end

    end
  end
endmodule
