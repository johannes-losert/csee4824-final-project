module load (
    //input logic start,
    input IS_EX_PACKET is_ex_reg,
    input [`XLEN-1:0] address,
    input [`XLEN-1:0]   Dmem2proc_data,
    input [3:0]         Dmem2proc_response,

    output logic [1:0]       proc2Dmem_command, // The memory command
    output MEM_SIZE          proc2Dmem_size,    // Size of data to read or write
    output logic [`XLEN-1:0] proc2Dmem_addr,    // Address sent to Data memory
    output logic [`XLEN-1:0] proc2Dmem_data,     // Data sent to Data memory
    output logic [`XLEN-1:0] result,
    output logic done
); 

    logic [`XLEN-1:0] read_data;
    logic rd_unsigned;
    MEM_SIZE memory_size;

    assign result = read_data;
    
    assign rd_unsigned  = is_ex_reg.inst.r.funct3[2]; // 1 if unsigned, 0 if signed
    assign memory_size     = MEM_SIZE'(is_ex_reg.inst.r.funct3[1:0]);

    // Outputs from the processor to memory
    assign proc2Dmem_command = (is_ex_reg.valid && is_ex_reg.wr_mem) ? BUS_STORE :
                               (is_ex_reg.valid && is_ex_reg.rd_mem) ? BUS_LOAD : BUS_NONE;
    assign proc2Dmem_size = memory_size;
    assign proc2Dmem_data = is_ex_reg.opb_value;
    assign proc2Dmem_addr = address; // Memory address is calculated by the ALU

    // Read data from memory and sign extend the proper bits
    always_comb begin
        read_data = Dmem2proc_data;
        if (rd_unsigned) begin
	    // unsigned: zero-extend the data
	    if (memory_size == BYTE) begin
	        read_data[`XLEN-1:8] = 0;
	    end else if (memory_size == HALF) begin
	        read_data[`XLEN-1:16] = 0;
	    end
        end else begin
	    // signed: sign-extend the data
	    if (memory_size[1:0] == BYTE) begin
	        read_data[`XLEN-1:8] = {(`XLEN-8){Dmem2proc_data[7]}};
	    end else if (memory_size == HALF) begin
	        read_data[`XLEN-1:16] = {(`XLEN-16){Dmem2proc_data[15]}};
	    end
        end
        done = 1;
    end

endmodule
