module riscv_cpu (
    input clk,
    input rst_n,
    output [31:0] imem_addr,
    input  [31:0] imem_data,
    input         imem_valid, // New input: valid signal from IMEM
    output [31:0] dmem_addr,
    output [31:0] dmem_data_write,
    input  [31:0] dmem_data_read,
    output        dmem_write_en
);

    // Pipeline registers
    reg [31:0] if_id_pc, if_id_inst;
    reg [31:0] id_ex_pc, id_ex_rd1, id_ex_rd2, id_ex_imm;
    reg [4:0]  id_ex_rs1, id_ex_rs2, id_ex_rd;
    reg [3:0]  id_ex_alu_op;
    reg [2:0]  id_ex_funct3;
    reg        id_ex_reg_write, id_ex_mem_write, id_ex_mem_read, id_ex_branch, id_ex_jump;
    reg [31:0] id_ex_pc_plus_4;
    reg [31:0] ex_mem_alu_result, ex_mem_rd2, ex_mem_pc_plus_4;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_reg_write, ex_mem_mem_write, ex_mem_mem_read;
    reg [31:0] mem_wb_alu_result, mem_wb_mem_data, mem_wb_pc_plus_4;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_reg_write, mem_wb_mem_read;

    // Program Counter
    reg [31:0] pc;
    wire [31:0] pc_next;

    // Flush and Stall control
    wire flush_if_id;
    wire stall, stall_jump, stall_fetch; // New: stall_fetch for IMEM latency
    reg  flush_if_id_prev;
    reg [1:0] stall_jump_counter;
    reg  jump_taken; // Track if a jump was taken
    reg  post_jump;  // Track if we are in the cycle after a jump

    // Fetch state machine for 4-cycle IMEM read
    reg [1:0] fetch_state; // 0: idle, 1-3: waiting for IMEM valid
    reg fetch_active;      // Indicates if a fetch is in progress

    // Hazard Detection
    wire [1:0] forward_a, forward_b;
    wire [1:0] forward_rs1_decode, forward_rs2_decode;

    // Write Back Result
    wire [31:0] mem_wb_result;

    // Decode Stage Signals
    wire [6:0] opcode = if_id_inst[6:0];
    wire [4:0] rs1 = if_id_inst[19:15];
    wire [4:0] rs2 = if_id_inst[24:20];
    wire [4:0] rd = if_id_inst[11:7];
    wire [2:0] funct3 = if_id_inst[14:12];
    wire [6:0] funct7 = if_id_inst[31:25];
    wire [31:0] imm_i = {{20{if_id_inst[31]}}, if_id_inst[31:20]};
    wire [31:0] imm_s = {{20{if_id_inst[31]}}, if_id_inst[31:25], if_id_inst[11:7]};
    wire [31:0] imm_b = {{19{if_id_inst[31]}}, if_id_inst[31], if_id_inst[7], if_id_inst[30:25], if_id_inst[11:8], 1'b0};
    wire [31:0] imm_j = {{11{if_id_inst[31]}}, if_id_inst[31], if_id_inst[19:12], if_id_inst[20], if_id_inst[30:21], 1'b0};
    wire is_nop = (if_id_inst == 32'h00000013);
    wire decode_jump = !is_nop && (opcode == 7'h6f || (opcode == 7'h67 && funct3 == 3'b000)) && !jump_taken && !post_jump;
    wire decode_branch = !is_nop && (opcode == 7'h63) && !jump_taken && !post_jump;

    // Register File
    wire [31:0] rs1_data_raw, rs2_data_raw;
    reg_file rf (
        .clk(clk),
        .rst_n(rst_n),
        .rs1_addr(if_id_inst[19:15]),
        .rs2_addr(if_id_inst[24:20]),
        .rd_addr(mem_wb_rd),
        .rd_data(mem_wb_result),
        .we(mem_wb_reg_write),
        .rs1_data(rs1_data_raw),
        .rs2_data(rs2_data_raw)
    );

    // Forwarding for rs1_data and rs2_data in Decode stage
    wire [31:0] rs1_data = (forward_rs1_decode == 2'b01) ? ex_mem_alu_result :
                           (forward_rs1_decode == 2'b10) ? mem_wb_result : rs1_data_raw;
    wire [31:0] rs2_data = (forward_rs2_decode == 2'b01) ? ex_mem_alu_result :
                           (forward_rs2_decode == 2'b10) ? mem_wb_result : rs2_data_raw;

    // Fetch Stage
    assign imem_addr = pc;

    // Stall the pipeline while fetching an instruction (4 cycles)
    assign stall_fetch = fetch_active && !imem_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 0;
            if_id_pc <= 0;
            if_id_inst <= 0;
            flush_if_id_prev <= 0;
            stall_jump_counter <= 0;
            jump_taken <= 0;
            post_jump <= 0;
            fetch_state <= 0;
            fetch_active <= 0;
        end else begin
            // Fetch state machine
            if (!stall && !stall_jump) begin
                if (!fetch_active) begin
                    // Start a new fetch
                    fetch_active <= 1;
                    fetch_state <= 0;
                end else if (imem_valid) begin
                    // IMEM has provided a valid instruction
                    if (!flush_if_id) begin
                        if_id_inst <= imem_data; // Update IF/ID with the new instruction
                        if_id_pc <= pc;
                    end else begin
                        if_id_inst <= 32'h00000013; // NOP on flush
                        if_id_pc <= pc;
                    end
                    pc <= pc_next; // Update PC to fetch the next instruction
                    fetch_active <= 0; // Reset fetch state
                    fetch_state <= 0;
                    jump_taken <= decode_jump && !jump_taken && !post_jump;
                    post_jump <= jump_taken;
                end
            end else if (jump_taken) begin
                // After a jump is taken, force the Fetch stage to update with the new instruction
                if (imem_valid) begin
                    if_id_inst <= imem_data;
                    if_id_pc <= pc;
                    jump_taken <= 0;
                    post_jump <= 1;
                    fetch_active <= 0;
                    fetch_state <= 0;
                end
            end else if (post_jump) begin
                // Ensure normal operation resumes after a jump
                if (imem_valid) begin
                    if_id_inst <= imem_data;
                    if_id_pc <= pc;
                    post_jump <= 0;
                    fetch_active <= 0;
                    fetch_state <= 0;
                end
            end

            // Debug: Print Fetch stage values
            $display("Fetch @ %0t: pc = %h, imem_data = %h, imem_valid = %b, if_id_inst = %h, flush_if_id = %b, stall = %b, stall_jump = %b, stall_fetch = %b, stall_jump_counter = %d, jump_taken = %b, post_jump = %b, fetch_active = %b", 
                     $time, pc, imem_data, imem_valid, if_id_inst, flush_if_id, stall, stall_jump, stall_fetch, stall_jump_counter, jump_taken, post_jump, fetch_active);
        end
    end

    // Decode Stage
    wire [31:0] jump_target = (opcode == 7'h6f) ? (if_id_pc + imm_j) : (rs1_data + imm_i);
    wire [31:0] decode_jump_target = decode_jump ? jump_target : 32'h0;

    // Stall for jump hazards (if JALR depends on a register being written)
    wire jump_dependency = decode_jump && (
        (id_ex_reg_write && (id_ex_rd != 0) && (id_ex_rd == rs1)) || // Check ID/EX stage
        (ex_mem_reg_write && (ex_mem_rd != 0) && (ex_mem_rd == rs1)) || // Check EX/MEM stage
        (mem_wb_reg_write && (mem_wb_rd != 0) && (mem_wb_rd == rs1)) // Check MEM/WB stage
    );
	/*
    // Debug: Print jump dependency
    always @(posedge clk) begin
        if (decode_jump) begin
            $display("Jump Dependency @ %0t: jump_dependency = %b, id_ex_rd = %h, ex_mem_rd = %h, mem_wb_rd = %h, rs1 = %h", 
                     $time, jump_dependency, id_ex_rd, ex_mem_rd, mem_wb_rd, rs1);
        end
    end
	*/
    // Stall for two cycles if there's a jump dependency
    assign stall_jump = (jump_dependency || stall_jump_counter > 0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stall_jump_counter <= 0;
        end else begin
            if (jump_dependency && stall_jump_counter == 0) begin
                stall_jump_counter <= 2; // Stall for 2 cycles
            end else if (stall_jump_counter > 0) begin
                stall_jump_counter <= stall_jump_counter - 1; // Decrement counter
            end else if (jump_taken || post_jump) begin
                stall_jump_counter <= 0; // Clear stall after jump is taken
            end
        end
    end
	/*
    // Debug: Print pipeline stage values
    always @(posedge clk) begin
        $display("ID/EX @ %0t: id_ex_rd = %h, id_ex_reg_write = %b", $time, id_ex_rd, id_ex_reg_write);
        $display("EX/MEM @ %0t: ex_mem_rd = %h, ex_mem_reg_write = %b, ex_mem_alu_result = %h", $time, ex_mem_rd, ex_mem_reg_write, ex_mem_alu_result);
        $display("MEM/WB @ %0t: mem_wb_rd = %h, mem_wb_reg_write = %b, mem_wb_result = %h", $time, mem_wb_rd, mem_wb_reg_write, mem_wb_result);
    end

    // Debug: Print jump target details and forwarding
    always @(posedge clk) begin
        if (decode_jump) begin
            $display("Jump @ %0t: opcode = %h, rs1 = %h, rs1_data = %h, imm_i = %h, jump_target = %h, decode_jump_target = %h, forward_rs1_decode = %b", 
                     $time, opcode, rs1, rs1_data, imm_i, jump_target, decode_jump_target, forward_rs1_decode);
        end
    end
	*/
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_pc <= 0;
            id_ex_rd1 <= 0;
            id_ex_rd2 <= 0;
            id_ex_imm <= 0;
            id_ex_rs1 <= 0;
            id_ex_rs2 <= 0;
            id_ex_rd <= 0;
            id_ex_alu_op <= 0;
            id_ex_funct3 <= 0;
            id_ex_reg_write <= 0;
            id_ex_mem_write <= 0;
            id_ex_mem_read <= 0;
            id_ex_branch <= 0;
            id_ex_jump <= 0;
            id_ex_pc_plus_4 <= 0;
        end else if (!stall && !stall_jump && !stall_fetch) begin
            id_ex_pc <= if_id_pc;
            id_ex_rd1 <= rs1_data;
            id_ex_rd2 <= rs2_data;
            id_ex_rs1 <= rs1;
            id_ex_rs2 <= rs2;
            id_ex_rd <= rd;
            id_ex_imm <= (opcode == 7'h63) ? imm_b : (opcode == 7'h6f) ? imm_j : (opcode == 7'h23) ? imm_s : imm_i;
            id_ex_alu_op <= (opcode == 7'h33 && funct7 == 7'h01) ? {1'b1, funct3} : {1'b0, funct3};
            id_ex_funct3 <= funct3;
            id_ex_reg_write <= (opcode == 7'h33 || opcode == 7'h13 || opcode == 7'h03 || opcode == 7'h6f || opcode == 7'h67);
            id_ex_mem_write <= (opcode == 7'h23);
            id_ex_mem_read <= (opcode == 7'h03);
            id_ex_branch <= decode_branch;
            id_ex_jump <= decode_jump;
            id_ex_pc_plus_4 <= if_id_pc + 4;
        end
    end

    // Execute Stage
    wire [31:0] alu_in1 = (forward_a == 2'b01) ? ex_mem_alu_result :
                          (forward_a == 2'b10) ? mem_wb_result : id_ex_rd1;
    wire [31:0] alu_in2 = id_ex_alu_op[3] ? id_ex_rd2 :
                          (forward_b == 2'b01) ? ex_mem_alu_result :
                          (forward_b == 2'b10) ? mem_wb_result : id_ex_imm;
    wire [31:0] alu_result;
    alu alu_inst (.op(id_ex_alu_op), .a(alu_in1), .b(alu_in2), .result(alu_result));

    wire branch_taken = id_ex_branch && (
        (id_ex_funct3 == 3'b000 && (alu_in1 == alu_in2)) ||        // BEQ
        (id_ex_funct3 == 3'b001 && (alu_in1 != alu_in2)) ||        // BNE
        (id_ex_funct3 == 3'b100 && ($signed(alu_in1) < $signed(alu_in2))) || // BLT
        (id_ex_funct3 == 3'b101 && ($signed(alu_in1) >= $signed(alu_in2))) || // BGE
        (id_ex_funct3 == 3'b110 && (alu_in1 < alu_in2)) ||         // BLTU
        (id_ex_funct3 == 3'b111 && (alu_in1 >= alu_in2))          // BGEU
    );

    wire [31:0] branch_target = id_ex_pc + id_ex_imm;
    assign pc_next = (decode_jump && !jump_taken && !post_jump) ? decode_jump_target :
                     (branch_taken && !jump_taken && !post_jump) ? branch_target : (pc + 4);
    assign flush_if_id = (decode_jump || branch_taken) && !jump_taken && !post_jump;

    // Debug: Print PC jump
    always @(posedge clk) begin
        if (decode_jump || branch_taken) begin
            $display("Jump/Branch @ %0t: decode_jump = %b, branch_taken = %b, pc_next = %h", $time, decode_jump, branch_taken, pc_next);
        end
    end

    wire [31:0] ex_result = id_ex_jump ? id_ex_pc_plus_4 : alu_result;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_mem_alu_result <= 0;
            ex_mem_rd2 <= 0;
            ex_mem_pc_plus_4 <= 0;
            ex_mem_rd <= 0;
            ex_mem_reg_write <= 0;
            ex_mem_mem_write <= 0;
            ex_mem_mem_read <= 0;
        end else if (!stall && !stall_jump && !stall_fetch) begin
            ex_mem_alu_result <= ex_result;
            ex_mem_rd2 <= id_ex_rd2;
            ex_mem_pc_plus_4 <= id_ex_pc_plus_4;
            ex_mem_rd <= id_ex_rd;
            ex_mem_reg_write <= id_ex_reg_write;
            ex_mem_mem_write <= id_ex_mem_write;
            ex_mem_mem_read <= id_ex_mem_read;
        end
    end

    // Memory Stage
    assign dmem_addr = ex_mem_alu_result;
    assign dmem_data_write = ex_mem_rd2;
    assign dmem_write_en = ex_mem_mem_write;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_alu_result <= 0;
            mem_wb_mem_data <= 0;
            mem_wb_pc_plus_4 <= 0;
            mem_wb_rd <= 0;
            mem_wb_reg_write <= 0;
            mem_wb_mem_read <= 0;
        end else if (!stall && !stall_jump && !stall_fetch) begin
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_data <= dmem_data_read;
            mem_wb_rd <= ex_mem_rd;
            mem_wb_reg_write <= ex_mem_reg_write;
            mem_wb_mem_read <= ex_mem_mem_read;
        end
    end

    // Write Back Stage
    assign mem_wb_result = mem_wb_mem_read ? mem_wb_mem_data : mem_wb_alu_result;

    // Hazard Detection Unit
    hazard_detection hd (
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex_rd(id_ex_rd),
        .if_id_rs1(rs1),
        .if_id_rs2(rs2),
        .id_ex_rs1(id_ex_rs1),
        .id_ex_rs2(id_ex_rs2),
        .ex_mem_rd(ex_mem_rd),
        .mem_wb_rd(mem_wb_rd),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .stall(stall),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .forward_rs1_decode(forward_rs1_decode),
        .forward_rs2_decode(forward_rs2_decode)
    );

    // Debug: Print hazard detection signals
    always @(posedge clk) begin
        $display("Hazard @ %0t: stall = %b, id_ex_mem_read = %b, id_ex_rd = %h, if_id_rs1 = %h, if_id_rs2 = %h", 
                 $time, stall, id_ex_mem_read, id_ex_rd, rs1, rs2);
    end

endmodule