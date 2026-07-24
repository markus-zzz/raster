`default_nettype none

//===========================================================================
// geom_front (burst master): geometry pass on the shared SDRAM bus.
//
// Walks a null-terminated linked list of descriptors starting at desc_head.
// Each descriptor is 16 halfwords (2 aligned bursts):
//   hw 0-1 : command       (0 = SET_OUTPUT, 1 = OBJECT)
//   hw 2-3 : next_desc      (halfword address; 0 => end of list)
//   SET_OUTPUT payload:                 OBJECT payload:
//     hw 4-5 : tri_base                   hw 4-5 : in_vertex_base
//     hw 6-7 : binlist_base               hw 6-7 : in_faces_base
//     hw 8-9 : bin_base                   hw 8-9 : in_matrix_base
//     hw10-11: light_x (neg dir)          hw10   : in_nbr_faces
//     hw12-13: light_y                    hw11-15: pad
//     hw14-15: light_z
//
// SET_OUTPUT delimits a frame: it latches the output bases + light and, if a
// region was already active, first finalizes it (flush partial bins + write
// counts) to the *old* bases. OBJECT transforms/bins one mesh, accumulating
// into the shared on-chip per-tile buckets (no reset, no finalize). At the end
// of the list the current region is finalized. So one list can carry several
// frames' worth of geometry. The list must start with a SET_OUTPUT.
//
// Per face it burst-reads the face record + 3 vertices, runs geom_engine,
// burst-writes the 16-word triangle record (2 bursts), and bins the triangle
// into per-tile buckets flushed 8 at a time as aligned bursts. Produces the
// TRI/BINLIST/BIN layout gpu_top reads.
//===========================================================================
module geom_front #(
    parameter int MEM_AW    = 24,
    parameter int W         = 320,   // render viewport (projection center = W/2, H/2)
    parameter int H         = 200,
    parameter int NTX       = 5,
    parameter int NTY       = 4,
    parameter int TILE_W    = 64,
    parameter int TILE_H    = 64,
    parameter int MAX_FACES_PER_TILE = 1024
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 start,
    input  wire  [MEM_AW-1:0]   desc_head,   // head of the descriptor list
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

    localparam [15:0] CMD_SET = 16'd0;   // set output bases + light (frame delimiter)
    localparam [15:0] CMD_OBJ = 16'd1;   // transform + bin one mesh

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
        S_IDLE, S_DESC0, S_DESC1, S_DECODE, S_SETAPPLY,
        S_MAT, S_FACE_HEAD, S_FACE, S_FACE_DEC, S_VTX, S_GEOM_W, S_WREC,
        S_BINP, S_BIN, S_BINFLUSH, S_ADVANCE, S_FLUSHT, S_WCNT, S_DONE
    } st_t;
    st_t st;

    // descriptor walk
    logic [MEM_AW-1:0] desc_ptr, next_desc_r;
    logic [MEM_AW-1:0] tri_base_r, binlist_base_r, bin_base_r;   // current output region
    logic [MEM_AW-1:0] ovtx, ofaces, omatrix;                    // current object inputs
    logic [15:0]       onfaces;
    logic [15:0]       dbuf [0:15];    // descriptor buffer (2 bursts)
    logic              region_active;  // an output region has been set up
    logic              fin_done;       // finalize target: 1=>S_DONE, 0=>S_SETAPPLY

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
            region_active <= 0;
        end else begin
            g_start <= 0; burst_go <= 0; done <= 0;
            case (st)
                S_IDLE: if (start) begin
                    desc_ptr <= desc_head; region_active <= 0; st <= S_DESC0;
                end

                // ---- fetch descriptor (2 bursts into dbuf) ----
                S_DESC0: begin
                    if (eng_idle) begin
                        burst_addr <= desc_ptr; burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<8;k++) dbuf[k] <= rdbuf[k];
                        st <= S_DESC1;
                    end
                end
                S_DESC1: begin
                    if (eng_idle) begin
                        burst_addr <= desc_ptr + MEM_AW'(BURST); burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<8;k++) dbuf[8+k] <= rdbuf[k];
                        st <= S_DECODE;
                    end
                end
                // decode command and dispatch
                S_DECODE: begin
                    next_desc_r <= MEM_AW'({dbuf[3], dbuf[2]});
                    if (dbuf[0] == CMD_SET) begin
                        // SET_OUTPUT: finalize the previous region first (if any),
                        // then apply the new bases/light (dbuf is preserved meanwhile).
                        if (region_active) begin fin_done <= 0; ft <= 0; st <= S_FLUSHT; end
                        else st <= S_SETAPPLY;
                    end else begin
                        // OBJECT
                        ovtx    <= MEM_AW'({dbuf[5], dbuf[4]});
                        ofaces  <= MEM_AW'({dbuf[7], dbuf[6]});
                        omatrix <= MEM_AW'({dbuf[9], dbuf[8]});
                        onfaces <= dbuf[10];
                        fcnt <= 0; mbi <= 0; st <= S_MAT;
                    end
                end
                // apply latched SET_OUTPUT: new bases + light, reset accumulators
                S_SETAPPLY: begin
                    tri_base_r     <= MEM_AW'({dbuf[5],  dbuf[4]});
                    binlist_base_r <= MEM_AW'({dbuf[7],  dbuf[6]});
                    bin_base_r     <= MEM_AW'({dbuf[9],  dbuf[8]});
                    g_light[0]     <= $signed({dbuf[11], dbuf[10]});
                    g_light[1]     <= $signed({dbuf[13], dbuf[12]});
                    g_light[2]     <= $signed({dbuf[15], dbuf[14]});
                    for (k=0;k<NUM_TILES;k++) tcount[k] <= 0;
                    tri_id <= 0; region_active <= 1;
                    st <= S_ADVANCE;
                end

                // ---- read matrix (3 bursts) from the object's matrix base ----
                S_MAT: begin
                    if (eng_idle) begin
                        burst_addr <= omatrix + (mbi << 3);
                        burst_we <= 0; burst_go <= 1;
                    end else if (burst_ack) begin
                        for (k=0;k<4;k++)
                            g_mat[mbi*4+k] <= $signed({rdbuf[2*k+1], rdbuf[2*k]});
                        if (mbi == 2) st <= S_FACE_HEAD;
                        else mbi <= mbi + 1'b1;
                    end
                end

                S_FACE_HEAD: begin
                    if (fcnt >= onfaces) st <= S_ADVANCE;
                    else begin fbi <= 0; st <= S_FACE; end
                end
                // ---- read face record (2 bursts) ----
                S_FACE: begin
                    if (eng_idle) begin
                        burst_addr <= ofaces + (fcnt << 4) + (fbi << 3);
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
                        burst_addr <= ovtx + (gidx[vi] << 3);
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
                        burst_addr <= tri_base_r + (tri_id << 4) + (recbi ? BURST : 0);
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
                        burst_addr <= binlist_base_r
                                    + (flush_tile * MAX_FACES_PER_TILE) + {2'b0, flush_base};
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        if (bin_last) begin
                            tri_id <= tri_id + 1'b1; fcnt <= fcnt + 1'b1; st <= S_FACE_HEAD;
                        end else st <= S_BIN;
                    end
                end

                // ---- advance to next descriptor (or finalize at end of list) ----
                S_ADVANCE: begin
                    if (next_desc_r == 0) begin
                        if (region_active) begin fin_done <= 1; ft <= 0; st <= S_FLUSHT; end
                        else st <= S_DONE;
                    end else begin
                        desc_ptr <= next_desc_r; st <= S_DESC0;
                    end
                end

                // ---- finalize region: flush partial per-tile groups ----
                S_FLUSHT: begin
                    if (ft >= NUM_TILES) begin wci <= 0; st <= S_WCNT; end
                    else if ((tcount[ft[TIDXW-1:0]] & 16'd7) == 0) begin
                        ft <= ft + 1'b1;   // nothing partial to flush
                    end else if (eng_idle) begin
                        for (k=0;k<8;k++) wbuf[k] <= tacc[ft[TIDXW-1:0]*BURST + k];
                        burst_addr <= binlist_base_r
                                    + (ft[TIDXW-1:0] * MAX_FACES_PER_TILE)
                                    + {2'b0, (tcount[ft[TIDXW-1:0]] & ~16'd7)};
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        ft <= ft + 1'b1;
                    end
                end

                // ---- write per-tile counts, then continue or finish ----
                S_WCNT: begin
                    if (eng_idle) begin
                        for (k=0;k<8;k++)
                            wbuf[k] <= ((wci*8+k) < NUM_TILES) ? tcount[(wci*8+k) % NUM_TILES] : 16'b0;
                        burst_addr <= bin_base_r + (wci << 3);
                        burst_we <= 1; burst_go <= 1;
                    end else if (burst_ack) begin
                        if (wci == CNT_BURSTS - 1) st <= fin_done ? S_DONE : S_SETAPPLY;
                        else wci <= wci + 1'b1;
                    end
                end

                S_DONE: begin done <= 1; st <= S_IDLE; end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
