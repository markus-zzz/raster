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
    //   FB_BASE      0x0A000
    parameter TRI_BASE          = 24'h00_0000,
    parameter BIN_BASE          = 24'h00_4000,
    parameter BINLIST_BASE      = 24'h00_5000,
    parameter MAX_FACES_PER_TILE = 1024,
    parameter FB_BASE           = 24'h00_A000
) (
    input  logic clk,
    input  logic rst,
    input  logic start, // frame_start
    output logic done,  // frame_done
    // External memory handshake interface
    output logic [MEM_AW-1:0] mem_addr,
    output logic              mem_req,
    output logic              mem_we,
    output logic [15:0]       mem_wr_data,
    input  logic [15:0]       mem_rd_data,
    input  logic              mem_rd_valid,
    input  logic              mem_ready
);

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
        S_RASTERIZE,
        S_DUMP_RD,
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
    logic [3:0]  tri_word;

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

    // Dump pixel counter
    logic [$clog2(TILE_PIX)-1:0] dump_pix;
    logic [$clog2(TILE_W)-1:0] dump_x;
    logic [$clog2(TILE_H)-1:0] dump_y;
    assign dump_x = dump_pix[$clog2(TILE_W)-1:0];
    assign dump_y = dump_pix[$clog2(TILE_PIX)-1:$clog2(TILE_W)];

    // FB read address for dump
    always_comb begin
        fb_rd_pixel_addr = ((ADDR_WIDTH+2)'(dump_y >> 1) * (TILE_W/2)
                          + (ADDR_WIDTH+2)'(dump_x >> 1)) * 4
                          + (ADDR_WIDTH+2)'({dump_y[0], dump_x[0]});
    end

    // Request tracking: only issue mem_req once per read, then wait for rd_valid
    logic req_sent;

    // Memory address/request generation
    always_comb begin
        mem_addr = '0;
        mem_req = 0;
        mem_we = 0;
        mem_wr_data = '0;

        case (state)
            S_LOAD_BIN_COUNT: begin
                mem_addr = BIN_BASE[MEM_AW-1:0] + MEM_AW'(tile_idx);
                mem_req = mem_ready && !req_sent;
            end
            S_FETCH_TRI_IDX: begin
                mem_addr = BINLIST_BASE[MEM_AW-1:0]
                         + MEM_AW'(tile_idx) * MEM_AW'(MAX_FACES_PER_TILE)
                         + MEM_AW'(tri_n);
                mem_req = mem_ready && !req_sent;
            end
            S_FETCH_TRI: begin
                mem_addr = TRI_BASE[MEM_AW-1:0] + (MEM_AW'(tri_id) << 4) + MEM_AW'(tri_word);
                mem_req = mem_ready && !req_sent;
            end
            S_DUMP_WR: begin
                mem_addr = FB_BASE[MEM_AW-1:0]
                         + MEM_AW'(tile_y + CH'(dump_y)) * MEM_AW'(FRAME_W)
                         + MEM_AW'(tile_x + CW'(dump_x));
                mem_req = mem_ready && !req_sent;
                mem_we = 1;
                mem_wr_data = fb_rd_data;
            end
            default: ;
        endcase
    end

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
            req_sent <= 0;
        end else begin
            rast_clear <= 0;
            rast_start <= 0;

            // Track whether we've issued a request in the current state
            if (mem_req && mem_ready)
                req_sent <= 1;

            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        tile_idx <= 0;
                        tile_x <= 0;
                        tile_y <= 0;
                        req_sent <= 0;
                        state <= S_LOAD_BIN_COUNT;
                    end
                end

                S_LOAD_BIN_COUNT: begin
                    if (mem_rd_valid) begin
                        bin_count <= mem_rd_data;
                        tri_n <= 0;
                        rast_clear <= 1;
                        req_sent <= 0;
                        state <= S_CLEAR;
                    end
                end

                S_CLEAR: begin
                    if (rast_clear_done) begin
                        if (bin_count == 0) begin
                            dump_pix <= 0;
                            req_sent <= 0;
                            state <= S_DUMP_RD;
                        end else begin
                            req_sent <= 0;
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                S_FETCH_TRI_IDX: begin
                    if (mem_rd_valid) begin
                        tri_id <= mem_rd_data;
                        tri_word <= 0;
                        req_sent <= 0;
                        state <= S_FETCH_TRI;
                    end
                end

                S_FETCH_TRI: begin
                    if (mem_rd_valid) begin
                        case (tri_word)
                            4'd0:  v0_x_r <= mem_rd_data[VW-1:0];
                            4'd1:  v0_y_r <= mem_rd_data[VH-1:0];
                            4'd2:  v1_x_r <= mem_rd_data[VW-1:0];
                            4'd3:  v1_y_r <= mem_rd_data[VH-1:0];
                            4'd4:  v2_x_r <= mem_rd_data[VW-1:0];
                            4'd5:  v2_y_r <= mem_rd_data[VH-1:0];
                            4'd6:  iz_init_r <= $signed(mem_rd_data);
                            4'd7:  iz_dx_r <= $signed(mem_rd_data);
                            4'd8:  iz_dy_r <= $signed(mem_rd_data);
                            4'd9:  color_r[15:0] <= mem_rd_data;
                            4'd10: color_r[23:16] <= mem_rd_data[7:0];
                            default: ;
                        endcase
                        if (tri_word == 4'd10) begin
                            rast_start <= 1;
                            req_sent <= 0;
                            state <= S_RASTERIZE;
                        end else begin
                            tri_word <= tri_word + 1;
                            req_sent <= 0;  // allow next word fetch
                        end
                    end
                end

                S_RASTERIZE: begin
                    if (rast_done) begin
                        tri_n <= tri_n + 1;
                        req_sent <= 0;
                        if (tri_n + 1 >= bin_count) begin
                            dump_pix <= 0;
                            state <= S_DUMP_RD;
                        end else begin
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                S_DUMP_RD: begin
                    // BRAM read latency: 1 cycle
                    state <= S_DUMP_WR;
                end

                S_DUMP_WR: begin
                    // Write accepted when mem_req && mem_ready (req_sent goes high)
                    if (req_sent) begin
                        req_sent <= 0;
                        if (dump_pix == $clog2(TILE_PIX)'(TILE_PIX - 1)) begin
                            state <= S_NEXT_TILE;
                        end else begin
                            dump_pix <= dump_pix + 1;
                            state <= S_DUMP_RD;
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
                    req_sent <= 0;
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
