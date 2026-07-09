`default_nettype none

// Startup DMA loader.
//
// External SDRAM is volatile, so the read-only geometry inputs (mesh, 90
// animation matrices, light) cannot be baked into it at bitstream load like
// the old on-chip behavioural model was. Instead they live in an on-chip block
// ROM (initialised via $readmemh) and this loader copies them into SDRAM once,
// after the controller finishes its power-up initialisation and before the
// first geometry pass. It runs in both simulation and on the FPGA, so the sim
// exercises the same load path.
//
// It is a burst-write bus master (BURST-word writes) using the same handshake
// with sdram_ctrl as gpu_top / geom_front.

module sdram_loader #(
    parameter int          MEM_AW      = 24,
    parameter int          INPUT_WORDS = 23680,        // halfwords (multiple of BURST)
    parameter [MEM_AW-1:0] INPUT_BASE  = 24'h02_0000,  // SDRAM dest base (burst-aligned)
    parameter              ROM_FILE    = ""
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
    localparam int NUM_BURSTS = INPUT_WORDS / BURST;
    localparam int RAW        = $clog2(INPUT_WORDS);
    localparam int CW         = $clog2(NUM_BURSTS + 1);

    // On-chip ROM holding the input image (inferred as block RAM).
    (* rom_style = "block" *) logic [15:0] rom [0:INPUT_WORDS-1];
    initial if (ROM_FILE != "") $readmemh(ROM_FILE, rom);
    logic [RAW-1:0] rom_ra;
    logic [15:0]    rom_q;
    always_ff @(posedge clk) rom_q <= rom[rom_ra];

    // ---- burst write engine (mirrors geom_front / gpu_top) ----
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_WR } mst_t;
    mst_t              mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic              burst_go, burst_ack;
    logic [15:0]       wbuf [0:BURST-1];
    logic [BSEL-1:0]   beat, wptr;
    logic [15:0]       wdata_r;

    assign mem_addr    = burst_addr;
    assign mem_we      = 1'b1;               // loader only ever writes
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

    // ---- high-level copy FSM ----
    typedef enum logic [2:0] { L_IDLE, L_FILL, L_ISSUE, L_WAIT, L_DONE } lst_t;
    lst_t          st;
    logic [CW-1:0] bidx;    // current burst index
    logic [3:0]    fj;      // ROM prefetch index 0..BURST

    // ROM read address: prefetch words bidx*BURST + fj (clamp on the extra tick)
    assign rom_ra = RAW'((bidx << BSEL) + ((fj < BURST) ? fj : (BURST-1)));

    always_ff @(posedge clk) begin
        if (rst) begin
            st <= L_IDLE; done <= 0; burst_go <= 0; bidx <= 0; fj <= 0;
        end else begin
            burst_go <= 0; done <= 0;
            case (st)
                L_IDLE: if (start) begin bidx <= 0; fj <= 0; st <= L_FILL; end

                // Prefetch this burst's 8 words from the ROM. rom_q lags the
                // address by one cycle, so word (fj-1) lands as fj advances.
                L_FILL: begin
                    if (fj >= 1) wbuf[fj-1] <= rom_q;
                    if (fj == BURST) begin fj <= 0; st <= L_ISSUE; end
                    else fj <= fj + 1'b1;
                end

                L_ISSUE: if (eng_idle) begin
                    burst_addr <= INPUT_BASE + (MEM_AW'(bidx) << BSEL);
                    burst_go   <= 1;
                    st         <= L_WAIT;
                end

                L_WAIT: if (burst_ack) begin
                    if (bidx == CW'(NUM_BURSTS-1)) st <= L_DONE;
                    else begin bidx <= bidx + 1'b1; fj <= 0; st <= L_FILL; end
                end

                L_DONE: begin done <= 1; st <= L_IDLE; end
                default: st <= L_IDLE;
            endcase
        end
    end
endmodule
