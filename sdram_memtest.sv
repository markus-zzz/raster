`default_nettype none

// Self-checking SDRAM memory test (bring-up diagnostic), varied-data version.
//
// Writes an address-derived pattern  pat(a) = a[15:0] ^ {a[23:16], a[23:16]}
// across [TEST_BASE, TEST_BASE+TEST_WORDS), reads it back and compares. Because
// the data varies with the address (unlike an all-1s/all-0s constant test), it
// exposes address-line, adjacent-bit-coupling and byte/bit-swap faults that a
// constant test passes. This matches the kind of highly-varied data the real
// geometry pipeline stores.
//
// Result on the framebuffer (vertical layout -> immune to the panel's
// horizontal mirror):
//   rows   0..127 : 16 per-bit bands (8 rows each, 1-px white separator on top).
//                   Top band = bit 15 (top half = upper byte, bottom = lower).
//                   green = bit never mismatched, red = bit mismatched.
//   rows 128..163 : low-address region (< 0x20000)   green ok / red error
//   rows 164..199 : high-address region (>= 0x20000) green ok / red error
//
// Read+write burst bus master, same handshake as geom_front / gpu_top.

module sdram_memtest #(
    parameter int          MEM_AW     = 24,
    parameter int          FRAME_W    = 320,
    parameter int          FRAME_H    = 200,
    parameter [MEM_AW-1:0] TEST_BASE  = 24'h00_0000,
    parameter int          TEST_WORDS = 32'h0002_8000, // covers FB + input region
    parameter [MEM_AW-1:0] FB_BASE    = 24'h00_A000
) (
    input  wire                clk,
    input  wire                rst,
    input  wire                start,
    output logic               done,
    // burst master interface (to arbiter)
    output logic [MEM_AW-1:0]  mem_addr,
    output logic               mem_req,
    output logic               mem_we,
    output logic [15:0]        mem_wr_data,
    input  wire                mem_wr_data_req,
    input  wire  [15:0]        mem_rd_data,
    input  wire                mem_rd_valid,
    input  wire                mem_ready
);
    localparam int BURST       = 8;
    localparam int BSEL        = 3;
    localparam int TEST_BURSTS = TEST_WORDS / BURST;
    localparam int FB_WORDS    = FRAME_W * FRAME_H;
    localparam int FB_BURSTS   = FB_WORDS / BURST;
    localparam int TBW         = $clog2(TEST_BURSTS + 1);
    localparam int FBW         = $clog2(FB_BURSTS + 1);
    localparam int XW          = $clog2(FRAME_W);
    localparam int YW          = $clog2(FRAME_H);

    localparam [15:0] C_GREEN = 16'h07E0;
    localparam [15:0] C_RED   = 16'hF800;
    localparam [15:0] C_WHITE = 16'hFFFF;
    localparam [15:0] C_BLACK = 16'h0000;

    function automatic [15:0] pat(input [MEM_AW-1:0] a);
        pat = a[15:0] ^ {a[23:16], a[23:16]};
    endfunction

    // ---- read+write burst engine (mirrors geom_front) ----
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_RD, M_WR } mst_t;
    mst_t              mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic              burst_we, burst_go, burst_ack;
    logic [15:0]       rdbuf [0:BURST-1];
    logic [15:0]       wbuf  [0:BURST-1];
    logic [BSEL-1:0]   beat, wptr;
    logic [15:0]       wdata_r;

    assign mem_addr    = burst_addr;
    assign mem_we      = burst_we;
    assign mem_req     = (mstate == M_REQ);
    assign mem_wr_data = wdata_r;

    always_ff @(posedge clk) begin
        if (rst) begin
            mstate <= M_IDLE; burst_ack <= 0; beat <= 0; wptr <= 0;
        end else begin
            burst_ack <= 0;
            wdata_r <= wbuf[wptr];
            if (mstate == M_WR && mem_wr_data_req) wptr <= wptr + 1'b1;
            case (mstate)
                M_IDLE: if (burst_go) begin beat <= 0; wptr <= 0; mstate <= M_REQ; end
                M_REQ:  if (mem_ready) begin beat <= 0; mstate <= burst_we ? M_WR : M_RD; end
                M_RD:   if (mem_rd_valid) begin
                            rdbuf[beat] <= mem_rd_data;
                            if (beat == BSEL'(BURST-1)) begin burst_ack <= 1; mstate <= M_IDLE; end
                            else beat <= beat + 1'b1;
                        end
                M_WR:   if (mem_wr_data_req) begin
                            if (beat == BSEL'(BURST-1)) begin burst_ack <= 1; mstate <= M_IDLE; end
                            else beat <= beat + 1'b1;
                        end
                default: mstate <= M_IDLE;
            endcase
        end
    end
    wire eng_idle = (mstate == M_IDLE) && !burst_go && !burst_ack;

    // ---- results ----
    logic [15:0]       err_bits;   // sticky OR of (got ^ expected)
    logic              err_low;    // any error at addr <  0x20000
    logic              err_high;   // any error at addr >= 0x20000

    // ---- high-level FSM ----
    typedef enum logic [3:0] {
        T_IDLE, T_WFILL, T_WISS, T_WWAIT,
        T_RISS, T_RWAIT, T_PFILL, T_PISS, T_PWAIT, T_DONE
    } tst_t;
    tst_t              st;
    logic [TBW-1:0]    bidx;
    logic [MEM_AW-1:0] baddr;
    logic [FBW-1:0]    pidx;
    logic [XW-1:0]     px;
    logic [YW-1:0]     py;

    integer k;
    function automatic [15:0] vizcol(input [XW-1:0] x, input [YW-1:0] y);
        logic [3:0] band, b;
        if (y < 128) begin
            if (y[2:0] == 3'd0) begin
                vizcol = C_WHITE;                 // separator line
            end else begin
                band = y[6:3];
                b    = 4'd15 - band;              // top band = bit 15
                vizcol = err_bits[b] ? C_RED : C_GREEN;
            end
        end else if (y < 164) begin
            vizcol = err_low  ? C_RED : C_GREEN;  // low-address region
        end else begin
            vizcol = err_high ? C_RED : C_GREEN;  // high-address region
        end
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            st <= T_IDLE; done <= 0; burst_go <= 0; burst_we <= 0;
            err_bits <= 0; err_low <= 0; err_high <= 0;
            bidx <= 0; baddr <= 0; pidx <= 0; px <= 0; py <= 0;
        end else begin
            burst_go <= 0; done <= 0;
            case (st)
                T_IDLE: if (start) begin
                    err_bits <= 0; err_low <= 0; err_high <= 0;
                    bidx <= 0; baddr <= TEST_BASE; st <= T_WFILL;
                end

                // ---- write phase (address-derived pattern) ----
                T_WFILL: begin
                    for (k = 0; k < BURST; k++) wbuf[k] <= pat(baddr + k[MEM_AW-1:0]);
                    st <= T_WISS;
                end
                T_WISS: if (eng_idle) begin
                    burst_addr <= baddr; burst_we <= 1; burst_go <= 1; st <= T_WWAIT;
                end
                T_WWAIT: if (burst_ack) begin
                    if (bidx == TBW'(TEST_BURSTS-1)) begin
                        bidx <= 0; baddr <= TEST_BASE; st <= T_RISS;
                    end else begin
                        bidx <= bidx + 1'b1; baddr <= baddr + BURST; st <= T_WFILL;
                    end
                end

                // ---- read + check phase ----
                T_RISS: if (eng_idle) begin
                    burst_addr <= baddr; burst_we <= 0; burst_go <= 1; st <= T_RWAIT;
                end
                T_RWAIT: if (burst_ack) begin
                    logic [15:0] xoracc; logic found;
                    xoracc = 16'b0; found = 1'b0;
                    for (k = 0; k < BURST; k++) begin
                        logic [15:0] e; e = pat(baddr + k[MEM_AW-1:0]);
                        if (rdbuf[k] != e) begin
                            xoracc = xoracc | (rdbuf[k] ^ e);
                            found  = 1'b1;
                        end
                    end
                    err_bits <= err_bits | xoracc;
                    if (found) begin
                        if (baddr >= 24'h02_0000) err_high <= 1'b1;
                        else                      err_low  <= 1'b1;
                    end
                    if (bidx == TBW'(TEST_BURSTS-1)) begin
                        pidx <= 0; px <= 0; py <= 0; st <= T_PFILL;
                    end else begin
                        bidx <= bidx + 1'b1; baddr <= baddr + BURST; st <= T_RISS;
                    end
                end

                // ---- paint result to framebuffer ----
                T_PFILL: begin
                    for (k = 0; k < BURST; k++) wbuf[k] <= vizcol(px + k[XW-1:0], py);
                    st <= T_PISS;
                end
                T_PISS: if (eng_idle) begin
                    burst_addr <= FB_BASE + (MEM_AW'(pidx) << BSEL);
                    burst_we <= 1; burst_go <= 1; st <= T_PWAIT;
                end
                T_PWAIT: if (burst_ack) begin
                    if (pidx == FBW'(FB_BURSTS-1)) st <= T_DONE;
                    else begin
                        pidx <= pidx + 1'b1;
                        if (px + BURST >= FRAME_W) begin px <= 0; py <= py + 1'b1; end
                        else px <= px + BURST[XW-1:0];
                        st <= T_PFILL;
                    end
                end

                T_DONE: begin done <= 1; st <= T_IDLE; end
                default: st <= T_IDLE;
            endcase
        end
    end
endmodule
