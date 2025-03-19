module hazard_detection (
    input id_ex_mem_read,
    input [4:0] id_ex_rd,
    input [4:0] if_id_rs1,
    input [4:0] if_id_rs2,
    input [4:0] id_ex_rs1,
    input [4:0] id_ex_rs2,
    input [4:0] ex_mem_rd,
    input [4:0] mem_wb_rd,
    input ex_mem_reg_write,
    input mem_wb_reg_write,
    output reg stall,
    output reg [1:0] forward_a,
    output reg [1:0] forward_b,
    output reg [1:0] forward_rs1_decode, // New: Forwarding for rs1 in Decode stage
    output reg [1:0] forward_rs2_decode  // New: Forwarding for rs2 in Decode stage
);

    always @(*) begin
        // Default values
        stall = 0;
        forward_a = 2'b00;
        forward_b = 2'b00;
        forward_rs1_decode = 2'b00;
        forward_rs2_decode = 2'b00;

        // Load-Use Hazard (stall if ID/EX is a load and uses rs1 or rs2 in IF/ID)
        if (id_ex_mem_read && (id_ex_rd != 0) && (id_ex_rd == if_id_rs1 || id_ex_rd == if_id_rs2)) begin
            stall = 1;
        end

        // Forwarding for rs1 in Execute stage (alu_in1)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs1)) begin
            forward_a = 2'b01; // Forward from EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs1)) begin
            forward_a = 2'b10; // Forward from MEM/WB
        end

        // Forwarding for rs2 in Execute stage (alu_in2)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs2)) begin
            forward_b = 2'b01; // Forward from EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs2)) begin
            forward_b = 2'b10; // Forward from MEM/WB
        end

        // Forwarding for rs1 in Decode stage (for jump target)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == if_id_rs1)) begin
            forward_rs1_decode = 2'b01; // Forward from EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == if_id_rs1)) begin
            forward_rs1_decode = 2'b10; // Forward from MEM/WB
        end

        // Forwarding for rs2 in Decode stage (not used for JALR, but added for completeness)
        if (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == if_id_rs2)) begin
            forward_rs2_decode = 2'b01; // Forward from EX/MEM
        end else if (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == if_id_rs2)) begin
            forward_rs2_decode = 2'b10; // Forward from MEM/WB
        end
    end

endmodule