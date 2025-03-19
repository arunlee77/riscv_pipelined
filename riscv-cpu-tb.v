module riscv_cpu_tb;

    reg clk, rst_n;
    wire [31:0] imem_addr, imem_data;
    wire [31:0] dmem_addr, dmem_data_write, dmem_data_read;
    wire dmem_write_en;
    wire imem_valid; // New: valid signal from IMEM

    // Instantiate the instruction memory
    imem imem_inst (
        .clk(clk),
        .rst_n(rst_n),
        .addr(imem_addr),
        .data(imem_data),
        .valid(imem_valid)
    );

    // Instantiate the data memory (unchanged)
    reg [31:0] dmem [0:255];
    assign dmem_data_read = dmem[dmem_addr[9:2]];

    always @(posedge clk) begin
        if (dmem_write_en) begin
            dmem[dmem_addr[9:2]] <= dmem_data_write;
        end
    end

    // Instantiate the CPU
    riscv_cpu cpu (
        .clk(clk),
        .rst_n(rst_n),
        .imem_addr(imem_addr),
        .imem_data(imem_data),
        .imem_valid(imem_valid), // Connect the valid signal
        .dmem_addr(dmem_addr),
        .dmem_data_write(dmem_data_write),
        .dmem_data_read(dmem_data_read),
        .dmem_write_en(dmem_write_en)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ps period
    end

    // Test stimulus
    initial begin
        $dumpfile("riscv_cpu.vcd");
        $dumpvars(0, riscv_cpu_tb);

        // Reset
        rst_n = 0;
        #20 rst_n = 1;

        // Run for a while to observe the pipeline
        #50000 $finish;
    end

endmodule