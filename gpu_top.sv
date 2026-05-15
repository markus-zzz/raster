module gpu_top #(
    parameter WIDTH       = 320,
    parameter HEIGHT      = 200,
    parameter TILE_W      = 64,
    parameter TILE_H      = 64,
    parameter SUBPIXEL    = 4,
    parameter MEM_AW      = 24,
    parameter TRI_BASE     = 0,
    parameter BIN_BASE     = 24'h08_0000,
    parameter BINLIST_BASE = 24'h08_1000,
    parameter FB_BASE      = 24'h10_0000
) (
    input  logic clk,
    input  logic rst,
    input  logic start,
    output logic done,
    // External memory (16-bit, 1-cycle read latency)
    output logic [MEM_AW-1:0] mem_addr,
    output logic              mem_we,
    output logic [15:0]       mem_wr_data,
    input  logic [15:0]       mem_rd_data
);

    localparam NTX = (WIDTH + TILE_W - 1) / TILE_W;
    localparam NTY = (HEIGHT + TILE_H - 1) / TILE_H;
    localparam NUM_TILES = NTX * NTY;
    localparam TILE_PIX = TILE_W * TILE_H;
    localparam ADDR_WIDTH = $clog2((TILE_W/2) * (TILE_H/2));
    localparam CW = $clog2(WIDTH);
    localparam CH = $clog2(HEIGHT);
    localparam VW = CW + SUBPIXEL;
    localparam VH = CH + SUBPIXEL;

    // Tile index and coordinates
    logic [$clog2(NUM_TILES)-1:0] tile_idx;
    logic [CW-1:0] tile_x;
    logic [CH-1:0] tile_y;

    // Compute tile_x/tile_y from tile_idx (registered for timing)
    always_ff @(posedge clk) begin
        if (rst) begin
            tile_x <= 0;
            tile_y <= 0;
        end else if (state == S_IDLE && start) begin
            tile_x <= 0;
            tile_y <= 0;
        end else if (state == S_NEXT_TILE) begin
            if (tile_x + CW'(TILE_W) >= CW'(WIDTH)) begin
                tile_x <= 0;
                tile_y <= tile_y + CH'(TILE_H);
            end else begin
                tile_x <= tile_x + CW'(TILE_W);
            end
        end
    end

    // FSM
    typedef enum logic [3:0] {
        S_IDLE,
        S_LOAD_BIN_OFFSET,
        S_LOAD_BIN_OFFSET_W,
        S_LOAD_BIN_COUNT,
        S_LOAD_BIN_COUNT_W,
        S_CLEAR,
        S_FETCH_TRI_IDX,
        S_FETCH_TRI_IDX_W,
        S_FETCH_TRI,
        S_RASTERIZE,
        S_DUMP_RD,
        S_DUMP_WR,
        S_NEXT_TILE,
        S_DONE
    } state_t;

    state_t state;

    // Bin info
    logic [MEM_AW-1:0] bin_offset;
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

    // Dump pixel coordinates (tile-local)
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

    // Memory address/write mux
    always_comb begin
        mem_addr = '0;
        mem_we = 0;
        mem_wr_data = '0;
        case (state)
            S_LOAD_BIN_OFFSET: mem_addr = BIN_BASE[MEM_AW-1:0] + (MEM_AW'(tile_idx) << 1);
            S_LOAD_BIN_COUNT:  mem_addr = BIN_BASE[MEM_AW-1:0] + (MEM_AW'(tile_idx) << 1) + 1;
            S_FETCH_TRI_IDX:   mem_addr = BINLIST_BASE[MEM_AW-1:0] + bin_offset + MEM_AW'(tri_n);
            S_FETCH_TRI:       mem_addr = TRI_BASE[MEM_AW-1:0] + (MEM_AW'(tri_id) << 4) + MEM_AW'(tri_word);
            S_DUMP_WR: begin
                mem_addr = FB_BASE[MEM_AW-1:0]
                         + MEM_AW'(tile_y + CH'(dump_y)) * MEM_AW'(WIDTH)
                         + MEM_AW'(tile_x + CW'(dump_x));
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
            rast_clear <= 0;
            rast_start <= 0;
        end else begin
            rast_clear <= 0;
            rast_start <= 0;

            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        tile_idx <= 0;
                        state <= S_LOAD_BIN_OFFSET;
                    end
                end

                S_LOAD_BIN_OFFSET: state <= S_LOAD_BIN_OFFSET_W;
                S_LOAD_BIN_OFFSET_W: begin
                    bin_offset <= MEM_AW'(mem_rd_data);
                    state <= S_LOAD_BIN_COUNT;
                end
                S_LOAD_BIN_COUNT: state <= S_LOAD_BIN_COUNT_W;
                S_LOAD_BIN_COUNT_W: begin
                    bin_count <= mem_rd_data;
                    tri_n <= 0;
                    rast_clear <= 1;
                    state <= S_CLEAR;
                end

                S_CLEAR: begin
                    if (rast_clear_done) begin
                        if (bin_count == 0) begin
                            dump_pix <= 0;
                            state <= S_DUMP_RD;
                        end else begin
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                S_FETCH_TRI_IDX: state <= S_FETCH_TRI_IDX_W;
                S_FETCH_TRI_IDX_W: begin
                    tri_id <= mem_rd_data;
                    tri_word <= 0;
                    state <= S_FETCH_TRI;
                end

                S_FETCH_TRI: begin
                    // Capture data from previous cycle's address
                    if (tri_word > 0) begin
                        case (tri_word - 4'd1)
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
                    end
                    if (tri_word == 4'd11) begin
                        rast_start <= 1;
                        state <= S_RASTERIZE;
                    end else begin
                        tri_word <= tri_word + 1;
                    end
                end

                S_RASTERIZE: begin
                    if (rast_done) begin
                        tri_n <= tri_n + 1;
                        if (tri_n + 1 >= bin_count) begin
                            dump_pix <= 0;
                            state <= S_DUMP_RD;
                        end else begin
                            state <= S_FETCH_TRI_IDX;
                        end
                    end
                end

                // Dump: 2 cycles per pixel (read FB, then write to ext mem)
                S_DUMP_RD: state <= S_DUMP_WR;  // present read addr, wait 1 cycle
                S_DUMP_WR: begin
                    // fb_rd_data is valid now; mem_we writes it
                    if (dump_pix == $clog2(TILE_PIX)'(TILE_PIX - 1)) begin
                        state <= S_NEXT_TILE;
                    end else begin
                        dump_pix <= dump_pix + 1;
                        state <= S_DUMP_RD;
                    end
                end

                S_NEXT_TILE: begin
                    tile_idx <= tile_idx + 1;
                    if (tile_idx + 1 >= $clog2(NUM_TILES)'(NUM_TILES)) begin
                        state <= S_DONE;
                    end else begin
                        state <= S_LOAD_BIN_OFFSET;
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
        .WIDTH(WIDTH),
        .HEIGHT(HEIGHT),
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
