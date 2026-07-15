`default_nettype none

//===========================================================================
// geom_front (burst master): geometry pass on the shared SDRAM bus.
//
// For every face it burst-reads the face record + the 3 vertices, runs
// geom_engine, burst-writes the 16-word triangle record (2 bursts), and bins
// the triangle into per-tile buckets. Bin entries are accumulated 8 at a time
// per tile and flushed as aligned bursts (no read-modify-write). Finally it
// writes the per-tile counts. Produces the TRI/BINLIST/BIN layout gpu_top reads.
//
// Burst master interface identical to gpu_top's (one 8-beat burst per req;
// 8-aligned addresses). Input layout (halfword addrs; 32-bit = lo,hi):
//   MATRIX_BASE : 12 words row-major 3x4, 2 hw each              (24 hw, 3 bursts)
//   LIGHT_BASE  : 3 words (negated light dir)                     (6 hw, 1 burst)
//   VTX_BASE    : vertex i at +i*8 : x,y,z (2 hw each) + 2 pad    (1 burst)
//   FACE_BASE   : face f at +f*16 : idx0,idx1,idx2, normal xyz,
//                 colour xyz (see decode)                          (2 bursts)
//===========================================================================
module geom_front #(
    parameter int MEM_AW    = 24,
    parameter int W         = 320,   // render viewport (projection center = W/2, H/2)
    parameter int H         = 200,
    parameter int NTX       = 5,
    parameter int NTY       = 4,
    parameter int TILE_W    = 64,
    parameter int TILE_H    = 64,
    parameter int MAX_FACES_PER_TILE = 1024,
    parameter int MATRIX_STRIDE = 24,   // halfwords per matrix (12 words x 2)
    parameter int MATRIX_BASE = 0,
    parameter int LIGHT_BASE  = 0,
    parameter int VTX_BASE    = 0,
    parameter int FACE_BASE   = 0,
    parameter int TRI_BASE    = 0,
    parameter int BIN_BASE    = 0,
    parameter int BINLIST_BASE= 0
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire  [15:0]         nfaces,
    input  wire  [15:0]         mat_index,   // which matrix to use this frame
    output logic                done,
    // burst master interface
    output logic [MEM_AW-1:0]   mem_addr,
    output logic                mem_req,
    output logic                mem_we,
    output logic [15:0]         mem_wr_data,
    input  wire                 mem_wr_data_req,
    input  wire  [15:0]         mem_rd_data,
    input  wire                 mem_rd_valid,
    input  wire                 mem_ready
);
    localparam int BURST     = 8;
    localparam int BSEL      = 3;
    localparam int NUM_TILES = NTX * NTY;
    localparam int CNT_BURSTS = (NUM_TILES + 7) / 8;   // 8 per-tile counts per burst
    localparam int TIDXW     = $clog2(NUM_TILES);
    localparam int TWSH      = $clog2(TILE_W);
    localparam int THSH      = $clog2(TILE_H);

    // ---------------- geom_engine ----------------
    logic               g_start, g_done, g_valid;
    logic signed [31:0] g_mat [0:11], g_vtx [0:8], g_nrm [0:2], g_col [0:2], g_light [0:2];
    logic signed [31:0] g_bbminx, g_bbminy, g_bbmaxx, g_bbmaxy;
    logic [15:0]        g_rec [0:10];

    geom_engine #(.W(W), .H(H)) u_geom (
        .clk(clk), .rst(rst), .start(g_start),
        .mat(g_mat), .vtx(g_vtx), .nrm(g_nrm), .col(g_col), .light(g_light),
        .done(g_done), .valid(g_valid),
        .bbminx(g_bbminx), .bbminy(g_bbminy), .bbmaxx(g_bbmaxx), .bbmaxy(g_bbmaxy),
        .rec(g_rec)
    );

    // ---------------- burst engine (mirrors gpu_top) ----------------
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_RD, M_WR } mst_t;
    mst_t               mstate;
    logic [MEM_AW-1:0]  burst_addr;
    logic               burst_we, burst_go, burst_ack;
    logic [15:0]        rdbuf [0:BURST-1];
    logic [15:0]        wbuf  [0:BURST-1];
    logic [BSEL-1:0]    beat, wptr;
    logic [15:0]        wdata_r;

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

    // ---------------- high-level FSM ----------------
    typedef enum logic [4:0] {
        S_IDLE, S_MAT, S_LIGHT, S_FACE_HEAD, S_FACE, S_FACE_DEC, S_VTX,
        S_GEOM_W, S_WREC, S_BINP, S_BIN, S_BINFLUSH,
        S_FLUSHT, S_WCNT, S_DONE
    } st_t;
    st_t st;

    logic [15:0] fcnt, tri_id, gidx [0:2];
    logic [15:0] fblk [0:15];
    logic [1:0]  mbi, fbi, vi;
    logic        recbi;
    logic [15:0] tcount [0:NUM_TILES-1];
    logic [15:0] tacc [0:NUM_TILES*BURST-1];
    // binning cursor
    logic signed [31:0] txmin, txmax, tymin, tymax, cur_tx, cur_ty;
    logic [TIDXW-1:0]   flush_tile;
    logic [15:0]        flush_base;
    logic               bin_last;
    logic [15:0]        ft;      // end-flush tile index
    logic [$clog2(CNT_BURSTS+1)-1:0] wci;     // count-burst index

    function automatic signed [31:0] divp(input signed [31:0] x, input int sh);
        return (x >= 0) ? (x >>> sh) : -((-x) >>> sh);
    endfunction
    function automatic signed [31:0] clamp(input signed [31:0] v,
                                           input signed [31:0] lo, input signed [31:0] hi);
        return (v < lo) ? lo : (v > hi) ? hi : v;
    endfunction

    integer k;
    always_ff @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; done <= 0; g_start <= 0; burst_go <= 0; burst_we <= 0;
        end else begin
            g_start <= 0; burst_go <= 0; done <= 0;
            case (st)
                S_IDLE: if (start) begin mbi <= 0; st <= S_MAT; end

                // ---- read matrix (3 bursts) ----
                S_MAT: begin
                    if (eng_idle) begin
                        burst_addr <= MATRIX_BASE[MEM_AW-1:0]
                                    + MEM_AW'(mat_index) * MATRIX_STRIDE + (mbi << 3);
                        burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<4;k++)
                            g_mat[mbi*4+k] <= $signed({rdbuf[2*k+1], rdbuf[2*k]});
                        if (mbi == 2) begin st <= S_LIGHT; end
                        else mbi <= mbi + 1'b1;
                    end
                end
                // ---- read light (1 burst) ----
                S_LIGHT: begin
                    if (eng_idle) begin
                        burst_addr <= LIGHT_BASE[MEM_AW-1:0]; burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<3;k++) g_light[k] <= $signed({rdbuf[2*k+1], rdbuf[2*k]});
                        for (k=0;k<NUM_TILES;k++) tcount[k] <= 0;
                        fcnt <= 0; tri_id <= 0; st <= S_FACE_HEAD;
                    end
                end

                S_FACE_HEAD: begin
                    if (fcnt >= nfaces) begin ft <= 0; st <= S_FLUSHT; end
                    else begin fbi <= 0; st <= S_FACE; end
                end
                // ---- read face record (2 bursts) ----
                S_FACE: begin
                    if (eng_idle) begin
                        burst_addr <= FACE_BASE[MEM_AW-1:0] + (fcnt << 4) + (fbi << 3);
                        burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<8;k++) fblk[fbi*8+k] <= rdbuf[k];
                        if (fbi == 1) st <= S_FACE_DEC;
                        else fbi <= fbi + 1'b1;
                    end
                end
                // decode face record now that both bursts are captured
                S_FACE_DEC: begin
                    gidx[0] <= fblk[0]; gidx[1] <= fblk[1]; gidx[2] <= fblk[2];
                    g_nrm[0] <= $signed({fblk[4],  fblk[3]});
                    g_nrm[1] <= $signed({fblk[6],  fblk[5]});
                    g_nrm[2] <= $signed({fblk[8],  fblk[7]});
                    g_col[0] <= $signed({fblk[10], fblk[9]});
                    g_col[1] <= $signed({fblk[12], fblk[11]});
                    g_col[2] <= $signed({fblk[14], fblk[13]});
                    vi <= 0; st <= S_VTX;
                end
                // ---- read vertices (1 burst each) ----
                S_VTX: begin
                    if (eng_idle) begin
                        burst_addr <= VTX_BASE[MEM_AW-1:0] + (gidx[vi] << 3);
                        burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        g_vtx[vi*3+0] <= $signed({rdbuf[1], rdbuf[0]});
                        g_vtx[vi*3+1] <= $signed({rdbuf[3], rdbuf[2]});
                        g_vtx[vi*3+2] <= $signed({rdbuf[5], rdbuf[4]});
                        if (vi == 2) begin g_start <= 1; st <= S_GEOM_W; end
                        else vi <= vi + 1'b1;
                    end
                end

                S_GEOM_W: if (g_done) begin
                    if (!g_valid) begin fcnt <= fcnt + 1'b1; st <= S_FACE_HEAD; end
                    else begin recbi <= 0; st <= S_WREC; end
                end

                // ---- write 16-word tri record (2 bursts) ----
                S_WREC: begin
                    if (eng_idle) begin
                        if (!recbi) for (k=0;k<8;k++) wbuf[k] <= g_rec[k];
                        else begin
                            wbuf[0] <= g_rec[8]; wbuf[1] <= g_rec[9]; wbuf[2] <= g_rec[10];
                            for (k=3;k<8;k++) wbuf[k] <= 16'b0;
                        end
                        burst_addr <= TRI_BASE[MEM_AW-1:0] + (tri_id << 4) + (recbi ? BURST : 0);
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        if (recbi) st <= S_BINP;
                        else recbi <= 1;
                    end
                end

                S_BINP: begin
                    txmin  <= clamp(divp(g_bbminx,TWSH), 0, NTX-1);
                    txmax  <= clamp(divp(g_bbmaxx,TWSH), 0, NTX-1);
                    tymin  <= clamp(divp(g_bbminy,THSH), 0, NTY-1);
                    tymax  <= clamp(divp(g_bbmaxy,THSH), 0, NTY-1);
                    cur_tx <= clamp(divp(g_bbminx,TWSH), 0, NTX-1);
                    cur_ty <= clamp(divp(g_bbminy,THSH), 0, NTY-1);
                    st <= S_BIN;
                end
                S_BIN: begin
                    logic [31:0] tile; logic [2:0] pos; logic last;
                    tile = cur_ty * NTX + cur_tx;
                    pos  = tcount[tile[TIDXW-1:0]][2:0];
                    last = (cur_tx == txmax) && (cur_ty == tymax);
                    tacc[tile[TIDXW-1:0]*BURST + pos] <= tri_id;
                    tcount[tile[TIDXW-1:0]] <= tcount[tile[TIDXW-1:0]] + 1'b1;
                    // advance cursor
                    if (cur_tx == txmax) begin cur_tx <= txmin; cur_ty <= cur_ty + 1; end
                    else cur_tx <= cur_tx + 1;
                    if (pos == 3'd7) begin
                        flush_tile <= tile[TIDXW-1:0];
                        flush_base <= tcount[tile[TIDXW-1:0]] & ~16'd7;
                        bin_last   <= last;
                        st <= S_BINFLUSH;
                    end else if (last) begin
                        tri_id <= tri_id + 1'b1; fcnt <= fcnt + 1'b1; st <= S_FACE_HEAD;
                    end
                end
                // flush a full 8-entry group for one tile
                S_BINFLUSH: begin
                    if (eng_idle) begin
                        for (k=0;k<8;k++) wbuf[k] <= tacc[flush_tile*BURST + k];
                        burst_addr <= BINLIST_BASE[MEM_AW-1:0]
                                    + (flush_tile * MAX_FACES_PER_TILE) + {2'b0, flush_base};
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        if (bin_last) begin
                            tri_id <= tri_id + 1'b1; fcnt <= fcnt + 1'b1; st <= S_FACE_HEAD;
                        end else st <= S_BIN;
                    end
                end

                // ---- end: flush partial per-tile groups ----
                S_FLUSHT: begin
                    if (ft >= NUM_TILES) begin wci <= 0; st <= S_WCNT; end
                    else if ((tcount[ft[TIDXW-1:0]] & 16'd7) == 0) begin
                        ft <= ft + 1'b1;   // nothing partial to flush
                    end else if (eng_idle) begin
                        for (k=0;k<8;k++) wbuf[k] <= tacc[ft[TIDXW-1:0]*BURST + k];
                        burst_addr <= BINLIST_BASE[MEM_AW-1:0]
                                    + (ft[TIDXW-1:0] * MAX_FACES_PER_TILE)
                                    + {2'b0, (tcount[ft[TIDXW-1:0]] & ~16'd7)};
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        ft <= ft + 1'b1;
                    end
                end

                // ---- write per-tile counts (3 bursts = 24 words) ----
                S_WCNT: begin
                    if (eng_idle) begin
                        for (k=0;k<8;k++)
                            wbuf[k] <= ((wci*8+k) < NUM_TILES) ? tcount[(wci*8+k) % NUM_TILES] : 16'b0;
                        burst_addr <= BIN_BASE[MEM_AW-1:0] + (wci << 3);
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        if (wci == CNT_BURSTS - 1) st <= S_DONE;
                        else wci <= wci + 1'b1;
                    end
                end

                S_DONE: begin done <= 1; st <= S_IDLE; end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
