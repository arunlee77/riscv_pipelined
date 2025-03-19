module reg_file (
    input clk,
    input rst_n,
    input [4:0] rs1_addr,    // Source register 1 address
    input [4:0] rs2_addr,    // Source register 2 address
    input [4:0] rd_addr,     // Destination register address
    input [31:0] rd_data,    // Data to write
    input we,                // Write enable
    output [31:0] rs1_data,  // Source register 1 data
    output [31:0] rs2_data   // Source register 2 data
);
    reg [31:0] registers [31:0];
    integer i;

    // Read (combinational)
    assign rs1_data = (rs1_addr == 0) ? 0 : registers[rs1_addr];
    assign rs2_data = (rs2_addr == 0) ? 0 : registers[rs2_addr];

    // Write and reset (sequential)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 0;
        end else if (we && rd_addr != 0) begin
            registers[rd_addr] <= rd_data;
        end
    end
endmodule
