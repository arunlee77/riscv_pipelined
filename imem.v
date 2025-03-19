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
	
		//$readmemh("pointers.mem", memory);
		$readmemh("rv32i.mem", memory);
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