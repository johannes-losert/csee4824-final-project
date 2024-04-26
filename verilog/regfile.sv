/////////////////////////////////////////////////////////////////////////
//                                                                     //
//   Modulename :  regfile.sv                                          //
//                                                                     //
//  Description :  This module creates the Regfile used by the ID and  //
//                 WB Stages of the Pipeline.                          //
//                                                                     //
/////////////////////////////////////////////////////////////////////////

`include "verilog/sys_defs.svh"

// P4 TODO: update this with the new parameters from sys_defs

module regfile (
    input             clock, // system clock
    // note: no system reset, register values must be written before they can be read
    input [`PHYS_REG_IDX_SZ:0] read_idx_1, read_idx_2, write_idx,
    input             write_en,
    input [`XLEN-1:0] write_data,

    output logic [`XLEN-1:0] read_out_1, read_out_2
);

    logic [`PHYS_REG_SZ:1] [`XLEN-1:0] registers; // 31 XLEN-length Registers (0 is known)

    // Read port 1
    always_comb begin
        if (read_idx_1 == `ZERO_REG) begin
            read_out_1 = 0;
        end else if (write_en && (write_idx == read_idx_1)) begin
            read_out_1 = write_data; // internal forwarding
        end else begin
            read_out_1 = registers[read_idx_1];
        end
    end

    // Read port 2
    always_comb begin
        if (read_idx_2 == `ZERO_REG) begin
            read_out_2 = 0;
        end else if (write_en && (write_idx == read_idx_2)) begin
            read_out_2 = write_data; // internal forwarding
        end else begin
            read_out_2 = registers[read_idx_2];
        end
    end 

    function void print_reg(int i);
        $write("%0d \t| %h\t|", i, registers[i]);
    endfunction
    function void print_regfile();
        $display("REGFILE");
        $display("Read 1: idx=%0d, val=%h", read_idx_1, read_out_1);
        $display("Read 2: idx=%0d, val=%h", read_idx_2, read_out_2);
        $write(" Idx \t| Val \t\t| Idx \t| Val \t\t| Idx \t| Val \t\t| Idx \t| Val \t\t|");
        $display("Idx \t| Val \t\t| Idx \t| Val \t\t| Idx \t| Val \t\t| Idx \t| Val \t\t|");

        for (int i = 0; i < (`PHYS_REG_SZ/8); i++) begin
                print_reg(i);
                print_reg(i + (`PHYS_REG_SZ/8));
                print_reg(i + (`PHYS_REG_SZ/4));
                print_reg(i + 3*(`PHYS_REG_SZ/8));
                print_reg(i + (`PHYS_REG_SZ/2));
                print_reg(i + 5*(`PHYS_REG_SZ/8));
                print_reg(i + 3*(`PHYS_REG_SZ/4));
                print_reg(i + 7*(`PHYS_REG_SZ/8));
                $display("");
        end
    endfunction

    always_ff @(negedge clock) begin 
        print_regfile();
    end

    // Write port
    always_ff @(posedge clock) begin
        if (write_en && write_idx != `ZERO_REG) begin
            $display("[REG] writing to register %0d: %h", write_idx, write_data);
            registers[write_idx] <= write_data;
        end
    end

endmodule // regfile
