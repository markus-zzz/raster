`default_nettype none

// Diagnostic framebuffer test-pattern writer.
//
// Fills the framebuffer region of SDRAM with vertical colour bars, once, at
// startup. With the geometry/raster pipeline bypassed (system_top TEST_PATTERN)
// the display controller then streams this pattern straight back out, so the
// LCD shows a known image that exercises ONLY the SDRAM write + read + display
// path. A marginal SDRAM interface degrades the bars gracefully (noise / a few
// wrong columns) instead of the total havoc a corrupted geometry input causes,
// which makes it easy to judge how close the read timing is.
//
// Burst-write bus master, same handshake as sdram_loader / gpu_top.

module fb_pattern #(
    parameter int          MEM_AW   = 24,
    parameter int          FRAME_W  = 320,           // must be a multiple of BURST
    parameter int          FRAME_H  = 200,
    parameter [MEM_AW-1:0] FB_BASE  = 24'h00_A000    // halfword base, burst-aligned
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
    input  wire  [15:0]        mem_rd_data,   // unused (write-only)
    input  wire                mem_rd_valid,  // unused (write-only)
    input  wire                mem_ready
);
    localparam int BURST      = 8;
    localparam int BSEL       = 3;
    localparam int FB_WORDS   = FRAME_W * FRAME_H;   // one 16-bit pixel per word
    localparam int NUM_BURSTS = FB_WORDS / BURST;
    localparam int CW         = $clog2(NUM_BURSTS + 1);
    localparam int XW         = $clog2(FRAME_W);

    // RGB565 colour bar palette (32-px wide bars: bar index = x[7:5]).
    function automatic [15:0] barcol(input [2:0] idx);
        case (idx)
            3'd0: barcol = 16'hFFFF; // white
            3'd1: barcol = 16'hFFE0; // yellow
            3'd2: barcol = 16'h07FF; // cyan
            3'd3: barcol = 16'h07E0; // green
            3'd4: barcol = 16'hF81F; // magenta
            3'd5: barcol = 16'hF800; // red
            3'd6: barcol = 16'h001F; // blue
            default: barcol = 16'h0000; // black
        endcase
    endfunction

    // ---- burst write engine (mirrors sdram_loader / geom_front) ----
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_WR } mst_t;
    mst_t              mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic              burst_go, burst_ack;
    logic [15:0]       wbuf [0:BURST-1];
    logic [BSEL-1:0]   beat, wptr;
    logic [15:0]       wdata_r;

    assign mem_addr    = burst_addr;
    assign mem_we      = 1'b1;               // write-only
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
                M_REQ:  if (mem_ready) begin beat <= 0; mstate <= M_WR; end
                M_WR:   if (mem_wr_data_req) begin
                            if (beat == BSEL'(BURST-1)) begin burst_ack <= 1; mstate <= M_IDLE; end
                            else beat <= beat + 1'b1;
                        end
                default: mstate <= M_IDLE;
            endcase
        end
    end
    wire eng_idle = (mstate == M_IDLE) && !burst_go && !burst_ack;

    // ---- high-level fill FSM ----
    typedef enum logic [2:0] { P_IDLE, P_FILL, P_ISSUE, P_WAIT, P_DONE } pst_t;
    pst_t          st;
    logic [CW-1:0] bidx;         // current burst index
    logic [XW-1:0] px;           // x of the burst's first pixel (0..FRAME_W-1)

    integer k;
    always_ff @(posedge clk) begin
        if (rst) begin
            st <= P_IDLE; done <= 0; burst_go <= 0; bidx <= 0; px <= 0;
        end else begin
            burst_go <= 0; done <= 0;
            case (st)
                P_IDLE: if (start) begin bidx <= 0; px <= 0; st <= P_FILL; end

                // Compute this burst's 8 pixels (same row: FRAME_W % BURST == 0).
                P_FILL: begin
                    for (k = 0; k < BURST; k++)
                        wbuf[k] <= barcol(3'((px + k[XW-1:0]) >> 5));
                    st <= P_ISSUE;
                end

                P_ISSUE: if (eng_idle) begin
                    burst_addr <= FB_BASE + (MEM_AW'(bidx) << BSEL);
                    burst_go   <= 1;
                    st         <= P_WAIT;
                end

                P_WAIT: if (burst_ack) begin
                    if (bidx == CW'(NUM_BURSTS-1)) st <= P_DONE;
                    else begin
                        bidx <= bidx + 1'b1;
                        px   <= (px + BURST >= FRAME_W) ? '0 : (px + BURST[XW-1:0]);
                        st   <= P_FILL;
                    end
                end

                P_DONE: begin done <= 1; st <= P_IDLE; end
                default: st <= P_IDLE;
            endcase
        end
    end
endmodule
