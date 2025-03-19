module imem (
    input clk,
    input rst_n,
    input [31:0] addr,        // 32-bit address from CPU
    output reg [31:0] data,   // 32-bit instruction output to CPU
    output reg valid          // Indicates when the 32-bit instruction is ready
);

    // Memory to store 8-bit chunks (byte-addressable)
    reg [7:0] mem [0:1023]; // 1KB memory (1024 bytes)
	reg [31:0] memory [0:1023]; // 3GB = 3 * 2^20 words
	
    // Internal state for fetching 4 bytes
    reg [1:0] fetch_state;  // State machine: 0, 1, 2, 3 (one state per byte)
    reg [31:0] addr_reg;    // Store the address during the fetch process
    reg [31:0] data_reg;    // Assemble the 32-bit instruction

    // Declare loop variable outside the for loop
    integer i, byte_index;

    // Initialize memory (for simulation purposes)
    initial begin
	/*
        // Instruction at 0x00: 00000293 (ADDI x5, x0, 0)
        mem[0] = 8'h93;  // LSB
        mem[1] = 8'h02;
        mem[2] = 8'h00;
        mem[3] = 8'h00;  // MSB
        // Instruction at 0x04: 00000313 (ADDI x6, x0, 0)
        mem[4] = 8'h13;
        mem[5] = 8'h03;
        mem[6] = 8'h00;
        mem[7] = 8'h00;
        // Instruction at 0x08: c00003b7 (LUI x7, 0xc0000)
        mem[8] = 8'hb7;
        mem[9] = 8'h03;
        mem[10] = 8'h00;
        mem[11] = 8'hc0;
        // Instruction at 0x0c: 40038393 (ADDI x7, x7, 0x400)
        mem[12] = 8'h93;
        mem[13] = 8'h83;
        mem[14] = 8'h03;
        mem[15] = 8'h40;
        // Instruction at 0x10: 00a00e13 (ADDI x28, x0, 0xa)
        mem[16] = 8'h13;
        mem[17] = 8'he0;
        mem[18] = 8'h00;
        mem[19] = 8'h0a;
        // Instruction at 0x14: 03c2d263 (BLT x5, x28, 0x3c)
        mem[20] = 8'h63;
        mem[21] = 8'hd2;
        mem[22] = 8'hc2;
        mem[23] = 8'h03;
        // Instruction at 0x18: 00030513 (MV x10, x6)
        mem[24] = 8'h13;
        mem[25] = 8'h05;
        mem[26] = 8'h03;
        mem[27] = 8'h00;
        // Instruction at 0x1c: 00100593 (ADDI x11, x0, 1)
        mem[28] = 8'h93;
        mem[29] = 8'h05;
        mem[30] = 8'h00;
        mem[31] = 8'h01;
        // Instruction at 0x20: 03c000ef (JAL x1, 0x3c)
        mem[32] = 8'hef;
        mem[33] = 8'h00;
        mem[34] = 8'hc0;
        mem[35] = 8'h03;
        // Instruction at 0x24: 00050313 (MV x6, x10)
        mem[36] = 8'h13;
        mem[37] = 8'h03;
        mem[38] = 8'h05;
        mem[39] = 8'h00;
        // Instruction at 0x5c: 00b50533 (ADD x10, x10, x11)
        mem[92] = 8'h33;
        mem[93] = 8'h05;
        mem[94] = 8'hb5;
        mem[95] = 8'h00;
        // Instruction at 0x60: 00008067 (JALR x0, x1, 0)
        mem[96] = 8'h67;
        mem[97] = 8'h80;
        mem[98] = 8'h00;
        mem[99] = 8'h00;
        // Fill remaining memory with NOPs (00000013)
        for (i = 100; i < 1024; i = i + 1) begin
            mem[i] = (i % 4 == 0) ? 8'h13 : 8'h00;
        end
	*/
		$readmemh("pointers.mem", memory); // Pointer arithmetic
		byte_index = 0;

		// Parse memory into bytes
		for (i = 0; i < 1024; i = i + 1) begin
			// Break each 32-bit memory word into 4 bytes
			mem[byte_index]     = memory[i][7:0];    // Byte 0 (LSB)
			mem[byte_index + 1] = memory[i][15:8];   // Byte 1
			mem[byte_index + 2] = memory[i][23:16];  // Byte 2
			mem[byte_index + 3] = memory[i][31:24];  // Byte 3 (MSB)

			// Advance the byte index by 4
			byte_index = byte_index + 4;
		end

		// Display the parsed bytes
		$display("Byte Memory Contents:");
		for (i = 0; i < byte_index; i = i + 1) begin
			$display("Address %0d: %h", i, mem[i]);
		end		
    end

    // Fetch state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fetch_state <= 0;
            addr_reg <= 0;
            data_reg <= 0;
            data <= 0;
            valid <= 0;
        end else begin
            case (fetch_state)
                0: begin
                    // Start fetching: latch the address
                    addr_reg <= addr;
                    data_reg[7:0] <= mem[addr]; // Read first byte
                    fetch_state <= 1;
                    valid <= 0;
                end
                1: begin
                    data_reg[15:8] <= mem[addr_reg + 1]; // Read second byte
                    fetch_state <= 2;
                    valid <= 0;
                end
                2: begin
                    data_reg[23:16] <= mem[addr_reg + 2]; // Read third byte
                    fetch_state <= 3;
                    valid <= 0;
                end
                3: begin
                    data_reg[31:24] <= mem[addr_reg + 3]; // Read fourth byte
                    // Assemble the 32-bit instruction directly into data
                    data[31:24] <= mem[addr_reg + 3];
                    data[23:16] <= mem[addr_reg + 2];
                    data[15:8] <= mem[addr_reg + 1];
                    data[7:0] <= mem[addr_reg];
                    fetch_state <= 0;
                    valid <= 1; // Signal that the instruction is ready
                end
                default: begin
                    fetch_state <= 0;
                    valid <= 0;
                end
            endcase
        end
    end

endmodule