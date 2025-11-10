/*
 * Copyright (c) 2025 Aaron Libokuohai Bovensiepen
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_libokuohai_asap_cpu_v1 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // -----------------------------
    // Clock divide (slow CPU + scan)
    // -----------------------------
    reg [23:0] div;
    always @(posedge clk or posedge reset) begin
        if (reset)
            div <= 24'd0;
        else
            div <= div + 1;
    end

 // wire cpu_clk  = div[12]; // faster CPU in simulation
 // wire scan_clk = div[8];
    wire cpu_clk  = div[18];
    wire scan_clk = div[10];

    // -----------------------------
    // Reset (SAP-1 expects active-high)
    // ui_in[0] can be a manual reset (optional)
    // -----------------------------
    wire reset = (~rst_n) | ui_in[0];

    // -----------------------------
    // SAP-1 core
    // -----------------------------
    wire [7:0] out;
    cpu cpu0(
        .clk   (cpu_clk),
        .reset (reset),
        .out   (out)
    );

    // -----------------------------
    // Binary to BCD (3 digits), 7-seg encoding
    // -----------------------------
    wire [11:0] bcd;
    bin_to_bcd u_b2d (.bin(out), .bcd(bcd));

    wire [6:0] seg_ones;
    wire [6:0] seg_tens;
    wire [6:0] seg_hundreds;

    seven_seg u_ones     (.bcd(bcd[3:0]),   .segments(seg_ones));
    seven_seg u_tens     (.bcd(bcd[7:4]),   .segments(seg_tens));
    seven_seg u_hundreds (.bcd(bcd[11:8]),  .segments(seg_hundreds));

    // -----------------------------
    // Scan three digits (active-low "cathode" enables)
    // -----------------------------
    reg  [3:0] cathode = 4'b1110; // D0 on first
    reg  [6:0] seg_cur;

    always @(posedge scan_clk or posedge reset) begin
        if (reset) begin
            cathode <= 4'b1110;
            seg_cur <= 7'b0000000;
        end else begin
            case (cathode)
                4'b1110: begin
                    cathode <= 4'b1011;     // next digit
                    seg_cur <= seg_hundreds;
                end
                4'b1011: begin
                    cathode <= 4'b1101;
                    seg_cur <= seg_tens;
                end
                4'b1101: begin
                    cathode <= 4'b1110;
                    seg_cur <= seg_ones;
                end
                default: begin
                    cathode <= 4'b1111;     // all off
                    seg_cur <= 7'b0000000;
                end
            endcase
        end
    end

    // -----------------------------
    // Pin mapping to TinyTapeout
    // -----------------------------
    // uo_out[6:0] = 7-seg segments (A..G), active-high like your original
    // uo_out[7]   = heartbeat (optional)
    assign uo_out[6:0] = seg_cur;
    assign uo_out[7]   = cpu_clk;

    // Put the 4 digit enable lines on bidirectional IO and drive them
    assign uio_out[3:0] = cathode;   // active-low digit enables
    assign uio_out[7:4] = 4'b0000;

    assign uio_oe[3:0]  = 4'b1111;   // drive these 4 pins out
    assign uio_oe[7:4]  = 4'b0000;   // leave the rest as inputs

    // -----------------------------
    // Silence unused warnings
    // -----------------------------
    wire _unused = &{ena, uio_in, ui_in[7:1], 1'b0};

endmodule

module bin_to_bcd(
    input wire[7:0] bin,
    output reg[11:0] bcd);

integer i;

always @(bin) begin
    bcd = 0;

    for (i = 0; i < 8; i = i+1) begin
        if (bcd[3:0] > 4)
            bcd[3:0] = bcd[3:0] + 3;

        if (bcd[7:4] > 4)
            bcd[7:4] = bcd[7:4] + 3;

        if (bcd[11:8] > 4)
            bcd[11:8] = bcd[11:8] + 3;

        // Concatenate acts as a shift
        bcd = {bcd[10:0], bin[7-i]};
    end
end

endmodule

module seven_seg(
    input wire[3:0] bcd,
    output wire[6:0] segments
);

assign segments =
    //              ABCDEFG
    (bcd == 0) ? 7'b1111110 :
    (bcd == 1) ? 7'b0110000 :
    (bcd == 2) ? 7'b1101101 :
    (bcd == 3) ? 7'b1111001 :
    (bcd == 4) ? 7'b0110011 :
    (bcd == 5) ? 7'b1011011 :
    (bcd == 6) ? 7'b1011111 :
    (bcd == 7) ? 7'b1110000 :
    (bcd == 8) ? 7'b1111111 :
    (bcd == 9) ? 7'b1110011 :
    7'b0000000;
endmodule

module cpu(
    input wire clk,
    input wire reset,
    output reg[7:0] out
    );

///////////////////////////////////////////////////////////////////////////////
// Opcodes
///////////////////////////////////////////////////////////////////////////////

parameter OP_NOP = 4'b0000;
parameter OP_LDA = 4'b0001;
parameter OP_ADD = 4'b0010;
parameter OP_SUB = 4'b0011;
parameter OP_STA = 4'b0100;
parameter OP_LDI = 4'b0101;
parameter OP_JMP = 4'b0110;
parameter OP_JC  = 4'b0111;
parameter OP_JZ  = 4'b1000;
//parameter OP_STI = 4'b1001;
parameter OP_OUT = 4'b1110;
parameter OP_HLT = 4'b1111;

// Registers
reg[3:0] pc;
reg[2:0] stage;
reg[3:0] mar;
reg[7:0] ir;
reg[7:0] a_reg;
reg[7:0] b_reg;
reg[1:0] flags;

parameter FLAG_C = 1;
parameter FLAG_Z = 0;

///////////////////////////////////////////////////////////////////////////////
// Control Signals (FIXED: Combinational Logic)
///////////////////////////////////////////////////////////////////////////////

// Halt
wire ctrl_ht = (ir[7:4] == OP_HLT && stage == 2);

// Memory Address Register In
wire ctrl_mi = (stage == 0) |
               (ir[7:4] == OP_LDA && stage == 2) |
               (ir[7:4] == OP_ADD && stage == 2) |
               (ir[7:4] == OP_SUB && stage == 2) |
               (ir[7:4] == OP_STA && stage == 2);

// RAM In
wire ctrl_ri = (ir[7:4] == OP_STA && stage == 3);

// RAM Out
wire ctrl_ro = (stage == 1) |
               (ir[7:4] == OP_LDA && stage == 3) |
               (ir[7:4] == OP_ADD && stage == 3) |
               (ir[7:4] == OP_SUB && stage == 3);

// Instruction Register Out
wire ctrl_io = (ir[7:4] == OP_LDA && stage == 2) |
               (ir[7:4] == OP_LDI && stage == 2) |
               (ir[7:4] == OP_ADD && stage == 2) |
               (ir[7:4] == OP_SUB && stage == 2) |
               (ir[7:4] == OP_STA && stage == 2) |
               (ir[7:4] == OP_JMP && stage == 2) |
               (ir[7:4] == OP_JC  && stage == 2) |
               (ir[7:4] == OP_JZ  && stage == 2);

// Instruction Register In
wire ctrl_ii = (stage == 1);

// A Register In
wire ctrl_ai = (ir[7:4] == OP_LDI && stage == 2) |
               (ir[7:4] == OP_LDA && stage == 3) |
               (ir[7:4] == OP_ADD && stage == 4) |
               (ir[7:4] == OP_SUB && stage == 4);

// A Register Out
wire ctrl_ao = (ir[7:4] == OP_STA && stage == 3) |
               (ir[7:4] == OP_OUT && stage == 2);

// Sum Out
wire ctrl_eo = (ir[7:4] == OP_ADD && stage == 4) |
               (ir[7:4] == OP_SUB && stage == 4);

// Subtract
wire ctrl_su = (ir[7:4] == OP_SUB && stage == 4);

// B Register In
wire ctrl_bi = (ir[7:4] == OP_ADD && stage == 3) |
               (ir[7:4] == OP_SUB && stage == 3);

// Output Register In
wire ctrl_oi = (ir[7:4] == OP_OUT && stage == 2);

// Counter Enable
wire ctrl_ce = (stage == 1);

// Counter Out
wire ctrl_co = (stage == 0);

// Jump
wire ctrl_jp = (ir[7:4] == OP_JMP && stage == 2) |
               (ir[7:4] == OP_JC  && stage == 2 && flags[FLAG_C] == 1) |
               (ir[7:4] == OP_JZ  && stage == 2 && flags[FLAG_Z] == 1);

// Flags Register In
wire ctrl_fi; // Assigned below, near ALU

///////////////////////////////////////////////////////////////////////////////
// Bus
///////////////////////////////////////////////////////////////////////////////
wire [7:0] bus;
wire [7:0] pc_zext = {4'b0000, pc};
wire [7:0] ir_zext = {4'b0000, ir[3:0]};
wire [7:0] alu_lo; // Assigned below

// RAM data out (combinational read)
wire [7:0] mem_out;
reg[7:0] mem[16];

assign bus =
    ctrl_co ? pc_zext :
    ctrl_ro ? mem_out : // Use combinational read
    ctrl_io ? ir_zext :
    ctrl_ao ? a_reg :
    ctrl_eo ? alu_lo :
    8'h00;

///////////////////////////////////////////////////////////////////////////////
// Program Counter
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        pc <= 0;
    else if (ctrl_ce)
        pc <= pc + 1;
    else if (ctrl_jp)
        pc <= bus[3:0];
end

///////////////////////////////////////////////////////////////////////////////
// Instruction Step Counter
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        stage <= 0;
    else if (stage == 5 || ctrl_jp) // 5 T-states per instruction (0-4), 5th rolls over
        stage <= 0;
    else if (ctrl_ht || stage == 6)
        // For a halt, put it into a stage it can never get out of
        stage <= 6;
    else
        stage <= stage + 1;
end

///////////////////////////////////////////////////////////////////////////////
// Memory Address Register
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        mar <= 0;
    else if (ctrl_mi)
        mar <= bus[3:0];
end

///////////////////////////////////////////////////////////////////////////////
// Memory (FIXED: Synthesizable RAM with Reset)
///////////////////////////////////////////////////////////////////////////////

assign mem_out = mem[mar]; // Combinational read

always @(posedge clk or posedge reset) begin
    if (reset) begin
        // Load program on reset (synthesizable)
        mem[0]  <= {OP_LDA,  4'b1111};   
        mem[1]  <= {OP_ADD,  4'b1110};
        mem[2]  <= {OP_STA,  4'b1111};   // store result of calculation #1
        mem[3]  <= {OP_JC,   4'b1011};   // jump to halt if overflow
        mem[4]  <= {OP_OUT,  4'b0000};   // output current fibonacci number
        mem[5]  <= {OP_LDA,  4'b1110};   
        mem[6]  <= {OP_ADD,  4'b1111};
        mem[7]  <= {OP_STA,  4'b1110};   // store result of calculation #2
        mem[8]  <= {OP_JC,   4'b1011};   // jump to halt if overflow
        mem[9]  <= {OP_OUT,  4'b0000};   // output current fibonacci number
        mem[10] <= {OP_JMP,  4'b0000};   // jump to beginning to calculate next number
        mem[11] <= {OP_HLT,  4'b0000};
        mem[12] <= {OP_NOP,  4'b0000};
        mem[13] <= {OP_NOP,  4'b0000};
        mem[14] <= {8'b00000001};    // storage #2
        mem[15] <= {8'b00000000};    // storage #1
    end
    else if (ctrl_ri) begin
        mem[mar] <= bus;
    end
end

///////////////////////////////////////////////////////////////////////////////
// Instruction Register
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        ir <= 0;
    else if (ctrl_ii)
        ir <= bus;
end

///////////////////////////////////////////////////////////////////////////////
// ALU
///////////////////////////////////////////////////////////////////////////////

wire[7:0] b_reg_out;
wire[8:0] alu;
wire flag_z, flag_c;

always @(posedge clk or posedge reset) begin
    if (reset)
        a_reg <= 0;
    else if (ctrl_ai)
        a_reg <= bus;
end

always @(posedge clk or posedge reset) begin
    if (reset)
        b_reg <= 0;
    else if (ctrl_bi)
        b_reg <= bus;
end

// Use twos-complement for subtraction
assign b_reg_out = ctrl_su ? ~b_reg + 1 : b_reg;

// ALU operation
assign alu = a_reg + b_reg_out;
assign alu_lo = alu[7:0];

// Zero flag is set if ALU is zero
assign flag_z = (alu[7:0] == 0) ? 1 : 0;

// Carry flag is set if there's an overflow into bit 8 of the ALU
assign flag_c = alu[8];

// Control signal for flags register (must be defined after flags)
assign ctrl_fi = (ir[7:4] == OP_ADD && stage == 4) |
                 (ir[7:4] == OP_SUB && stage == 4);

///////////////////////////////////////////////////////////////////////////////
// Flags Register
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        flags <= 0;
    else if (ctrl_fi)
        flags <= {flag_c, flag_z};
end

///////////////////////////////////////////////////////////////////////////////
// Output Register
///////////////////////////////////////////////////////////////////////////////

always @(posedge clk or posedge reset) begin
    if (reset)
        out <= 0;
    else if (ctrl_oi)
        out <= bus;
end

// REMOVED `initial` block as it's not synthesizable
// Program is now loaded in the `mem` always block on reset.

endmodule