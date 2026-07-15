`default_nettype none

module gpu_top #(
    parameter FRAME_W      = 320,
    parameter FRAME_H      = 200,
    parameter TILE_W       = 64,
    parameter TILE_H       = 64,
    parameter SUBPIXEL     = 4,
    parameter MEM_AW       = 24,
    // Compact memory map fits in 256 KB:
    //   TRI_BASE     0x00000
    //   BIN_BASE     0x04000
    //   BINLIST_BASE 0x05000  (tile T at +T*MAX_FACES_PER_TILE)
    //   FB_BASE      0x30000
    parameter TRI_BASE          = 24'h00_0000,
    parameter BIN_BASE          = 24'h00_4000,
    parameter BINLIST_BASE      = 24'h00_5000,
    parameter MAX_FACES_PER_TILE = 1024,
    parameter FB_BASE           = 24'h03_0000
) (
    input  wire  clk,
    input  wire  rst,
    input  wire  start, // frame_start
    output logic done,  // frame_done
    // External memory handshake interface
    output logic [MEM_AW-1:0] mem_addr,
    output logic              mem_req,
    output logic              mem_we,
    output logic [15:0]       mem_wr_data,
    input  wire               mem_wr_data_req,
    input  wire  [15:0]       mem_rd_data,
    input  wire               mem_rd_valid,
    input  wire               mem_ready
);

    localparam BURST = 8;
    localparam NTX = (FRAME_W + TILE_W - 1) / TILE_W;
    localparam NTY = (FRAME_H + TILE_H - 1) / TILE_H;
    localparam NUM_TILES = NTX * NTY;
    localparam TILE_PIX = TILE_W * TILE_H;
    localparam ADDR_WIDTH = $clog2((TILE_W/2) * (TILE_H/2));
    localparam CW = $clog2(FRAME_W);
    localparam CH = $clog2(FRAME_H);
    localparam VW = CW + SUBPIXEL;
    localparam VH = CH + SUBPIXEL;

    // Tile coordinates
    logic [$clog2(NUM_TILES)-1:0] tile_idx;
    logic [CW-1:0] tile_x;
    logic [CH-1:0] tile_y;

    // FSM
    typedef enum logic [3:0] {
        S_IDLE,
        S_LOAD_BIN_COUNT,
        S_CLEAR,
        S_FETCH_TRI_IDX,
        S_FETCH_TRI,
        S_FETCH_TRI2,
        S_RASTERIZE,
        S_DUMP_WR,
        S_NEXT_TILE,
        S_DONE
    } state_t;

    state_t state;

    // Bin info
    logic [15:0]       bin_count;
    logic [15:0]       tri_n;

    // Triangle fetch
    logic [15:0] tri_id;

    // Triangle registers
    logic [VW-1:0] v0_x_r, v1_x_r, v2_x_r;
    logic [VH-1:0] v0_y_r, v1_y_r, v2_y_r;
    logic signed [15:0] iz_init_r, iz_dx_r, iz_dy_r;
    logic [23:0] color_r;

    // Rasterizer interface
    logic rast_clear, rast_clear_done;
    logic rast_start, rast_done;
    logic [ADDR_WIDTH+1:0] fb_rd_pixel_addr;
    logic [15:0] fb_rd_data;

    // Dump pixel counter (group base, advances by BURST per write burst).
    logic [$clog2(TILE_PIX)-1:0] dump_pix;

    localparam PIXW = $clog2(TILE_PIX);
    localparam TWB  = $clog2(TILE_W);
    localparam BSEL = $clog2(BURST);

    // During a dump write-burst the tile-BRAM read index leads the controller's
    // data latch by one cycle: wptr is the beat whose address is presented now;
    // its (registered) data is latched by the controller next cycle. wptr
    // advances on each mem_wr_data_req.
    logic [BSEL-1:0] wptr;
    logic [PIXW-1:0] ridx;
    assign ridx = dump_pix + PIXW'(wptr);
    wire [TWB-1:0]        rx = ridx[TWB-1:0];
    wire [PIXW-TWB-1:0]   ry = ridx[PIXW-1:TWB];

    // FB read address for dump (tile-internal swizzled 2x2 layout)
    always_comb begin
        fb_rd_pixel_addr = ((ADDR_WIDTH+2)'(ry >> 1) * (TILE_W/2)
                          + (ADDR_WIDTH+2)'(rx >> 1)) * 4
                          + (ADDR_WIDTH+2)'({ry[0], rx[0]});
    end

    // Group base coordinates for the FB write burst address.
    wire [TWB-1:0]      dump_x = dump_pix[TWB-1:0];
    wire [PIXW-TWB-1:0] dump_y = dump_pix[PIXW-1:TWB];

    //=====================================================================
    // Generic burst engine. The main FSM presents an 8-aligned address and a
    // direction, pulses burst_go, and waits for burst_ack. Reads land in
    // rdbuf[]; writes stream directly from the tile BRAM (mem_wr_data =
    // fb_rd_data), with wptr selecting the beat.
    //=====================================================================
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_RD, M_WR } mstate_t;
    mstate_t mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic              burst_we;
    logic              burst_go;
    logic              burst_ack;
    logic [15:0]       rdbuf [0:BURST-1];
    logic [BSEL-1:0]   beat;

    assign mem_addr    = burst_addr;
    assign mem_we      = burst_we;
    assign mem_req     = (mstate == M_REQ);
    assign mem_wr_data = fb_rd_data;   // streamed directly from tile BRAM

    always_ff @(posedge clk) begin
        if (rst) begin
            mstate    <= M_IDLE;
            burst_ack <= 0;
            beat      <= 0;
            wptr      <= 0;
        end else begin
            burst_ack <= 0;
            // Advance the dump read pointer each time the controller consumes a
            // write beat (leads the data latch by one cycle).
            if (mstate == M_WR && mem_wr_data_req)
                wptr <= wptr + 1'b1;
            case (mstate)
                M_IDLE: if (burst_go) begin
                    beat   <= 0;
                    wptr   <= 0;
                    mstate <= M_REQ;
                end
                M_REQ: if (mem_ready) begin
                    beat   <= 0;
                    mstate <= burst_we ? M_WR : M_RD;
                end
                M_RD: if (mem_rd_valid) begin
                    rdbuf[beat] <= mem_rd_data;
                    if (beat == BSEL'(BURST-1)) begin
                        burst_ack <= 1;
                        mstate    <= M_IDLE;
                    end else
                        beat <= beat + 1'b1;
                end
                M_WR: if (mem_wr_data_req) begin
                    if (beat == BSEL'(BURST-1)) begin
                        burst_ack <= 1;
                        mstate    <= M_IDLE;
                    end else
                        beat <= beat + 1'b1;
                end
                default: mstate <= M_IDLE;
            endcase
        end
    end

    // Unaligned single-word accesses: read the aligned 8-word block and select.
    wire [MEM_AW-1:0] bin_full  = BIN_BASE[MEM_AW-1:0] + MEM_AW'(tile_idx);
    wire [MEM_AW-1:0] tidx_full = BINLIST_BASE[MEM_AW-1:0]
                                + MEM_AW'(tile_idx) * MEM_AW'(MAX_FACES_PER_TILE)
                                + MEM_AW'(tri_n);
    logic [BSEL-1:0] sel_off;  // word offset within the burst block

    // FSM logic
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            done <= 0;
            tile_idx <= 0;
            tile_x <= 0;
            tile_y <= 0;
            rast_clear <= 0;
            rast_start <= 0;
            burst_go <= 0;
        end else begin
            rast_clear <= 0;
            rast_start <= 0;
            burst_go <= 0;

            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        tile_idx <= 0;
                        tile_x <= 0;
                        tile_y <= 0;
                        state <= S_LOAD_BIN_COUNT;
                    end
                end

                // Read the 8-word block containing the bin count and select it.
                S_LOAD_BIN_COUNT: begin
                    if (mstate == M_IDLE && !burst_go && !burst_ack) begin
                        burst_addr <= {bin_full[MEM_AW-1:BSEL], {BSEL{1'b0}}};
                        burst_we   <= 1'b0;
                        sel_off    <= bin_full[BSEL-1:0];
                        burst_go   <= 1'b1;
                    end else if (burst_ack) begin
                        bin_count <= rdbuf[sel_off];
                        tri_n <= 0;
                        rast_clear <= 1;
                        state <= S_CLEAR;
                    end
                end

                S_CLEAR: begin
                    if (rast_clear_done) begin
                        if (bin_count == 0) begin
                            dump_pix <= 0;
                            state <= S_DUMP_WR;
                        end else begin
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                // Read the 8-word block containing the triangle index.
                S_FETCH_TRI_IDX: begin
                    if (mstate == M_IDLE && !burst_go && !burst_ack) begin
                        burst_addr <= {tidx_full[MEM_AW-1:BSEL], {BSEL{1'b0}}};
                        burst_we   <= 1'b0;
                        sel_off    <= tidx_full[BSEL-1:0];
                        burst_go   <= 1'b1;
                    end else if (burst_ack) begin
                        tri_id <= rdbuf[sel_off];
                        state  <= S_FETCH_TRI;
                    end
                end

                // Triangle record is 16 words = one 8-aligned burst (low 11
                // words used). tri_id*16 is 8-aligned.
                S_FETCH_TRI: begin
                    if (mstate == M_IDLE && !burst_go && !burst_ack) begin
                        burst_addr <= TRI_BASE[MEM_AW-1:0] + (MEM_AW'(tri_id) << 4);
                        burst_we   <= 1'b0;
                        burst_go   <= 1'b1;
                    end else if (burst_ack) begin
                        // Words 0..7 are in rdbuf; 8..10 need a second burst.
                        v0_x_r <= rdbuf[0][VW-1:0];
                        v0_y_r <= rdbuf[1][VH-1:0];
                        v1_x_r <= rdbuf[2][VW-1:0];
                        v1_y_r <= rdbuf[3][VH-1:0];
                        v2_x_r <= rdbuf[4][VW-1:0];
                        v2_y_r <= rdbuf[5][VH-1:0];
                        iz_init_r <= $signed(rdbuf[6]);
                        iz_dx_r   <= $signed(rdbuf[7]);
                        state <= S_FETCH_TRI2;
                    end
                end

                S_FETCH_TRI2: begin
                    if (mstate == M_IDLE && !burst_go && !burst_ack) begin
                        burst_addr <= TRI_BASE[MEM_AW-1:0] + (MEM_AW'(tri_id) << 4) + MEM_AW'(BURST);
                        burst_we   <= 1'b0;
                        burst_go   <= 1'b1;
                    end else if (burst_ack) begin
                        iz_dy_r       <= $signed(rdbuf[0]);  // word 8
                        color_r[15:0] <= rdbuf[1];           // word 9
                        color_r[23:16]<= rdbuf[2][7:0];      // word 10
                        rast_start <= 1;
                        state <= S_RASTERIZE;
                    end
                end

                S_RASTERIZE: begin
                    if (rast_done) begin
                        tri_n <= tri_n + 1;
                        if (tri_n + 1 >= bin_count) begin
                            dump_pix <= 0;
                            state <= S_DUMP_WR;
                        end else begin
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                // Stream BURST tile pixels directly from the raster BRAM to the
                // SDRAM write port. The burst engine's wptr presents the read
                // address one cycle ahead of the controller's data latch, so no
                // prefetch buffer is needed.
                S_DUMP_WR: begin
                    if (mstate == M_IDLE && !burst_go && !burst_ack) begin
                        burst_addr <= FB_BASE[MEM_AW-1:0]
                                    + MEM_AW'(tile_y + CH'(dump_y)) * MEM_AW'(FRAME_W)
                                    + MEM_AW'(tile_x + CW'(dump_x));
                        burst_we   <= 1'b1;
                        burst_go   <= 1'b1;
                    end else if (burst_ack) begin
                        if (dump_pix >= PIXW'(TILE_PIX - BURST)) begin
                            state <= S_NEXT_TILE;
                        end else begin
                            dump_pix <= dump_pix + PIXW'(BURST);
                            state    <= S_DUMP_WR;
                        end
                    end
                end

                S_NEXT_TILE: begin
                    tile_idx <= tile_idx + 1;
                    if (tile_x + CW'(TILE_W) >= CW'(FRAME_W)) begin
                        tile_x <= 0;
                        tile_y <= tile_y + CH'(TILE_H);
                    end else begin
                        tile_x <= tile_x + CW'(TILE_W);
                    end
                    if (tile_idx + 1 >= $clog2(NUM_TILES)'(NUM_TILES)) begin
                        state <= S_DONE;
                    end else begin
                        state <= S_LOAD_BIN_COUNT;
                    end
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Raster top instance
    raster_top #(
        .FRAME_W(FRAME_W),
        .FRAME_H(FRAME_H),
        .TILE_W(TILE_W),
        .TILE_H(TILE_H),
        .SUBPIXEL(SUBPIXEL)
    ) rast (
        .clk(clk),
        .rst(rst),
        .clear(rast_clear),
        .clear_done(rast_clear_done),
        .tile_x(tile_x),
        .tile_y(tile_y),
        .start(rast_start),
        .v0_x(v0_x_r), .v1_x(v1_x_r), .v2_x(v2_x_r),
        .v0_y(v0_y_r), .v1_y(v1_y_r), .v2_y(v2_y_r),
        .iz_init(iz_init_r),
        .iz_dx(iz_dx_r),
        .iz_dy(iz_dy_r),
        .color(color_r),
        .done(rast_done),
        .fb_rd_pixel_addr(fb_rd_pixel_addr),
        .fb_rd_data(fb_rd_data)
    );

endmodule
