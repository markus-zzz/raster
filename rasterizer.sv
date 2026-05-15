module rasterizer #(
    parameter RES_W = 320,
    parameter RES_H = 200,
    parameter TILE_W = 320,    // tile width (BRAM size)
    parameter TILE_H = 200,    // tile height (BRAM size)
    parameter SUBPIXEL = 4,
    parameter ADDR_WIDTH = $clog2((TILE_W/2) * (TILE_H/2))
) (
    input  logic clk,
    input  logic rst,
    // Tile offset in screen coordinates (pixel-aligned)
    input  logic [$clog2(RES_W)-1:0] tile_x,
    input  logic [$clog2(RES_H)-1:0] tile_y,
    // Triangle input (fixed-point: SUBPIXEL fractional bits, screen-space)
    input  logic start,
    input  logic [$clog2(RES_W)+SUBPIXEL-1:0] v0_x,
    input  logic [$clog2(RES_H)+SUBPIXEL-1:0] v0_y,
    input  logic [$clog2(RES_W)+SUBPIXEL-1:0] v1_x,
    input  logic [$clog2(RES_H)+SUBPIXEL-1:0] v1_y,
    input  logic [$clog2(RES_W)+SUBPIXEL-1:0] v2_x,
    input  logic [$clog2(RES_H)+SUBPIXEL-1:0] v2_y,
    input  logic signed [15:0] iz_init,  // 1/z at pixel (0,0)
    input  logic signed [15:0] iz_dx,    // d(1/z)/dx per pixel
    input  logic signed [15:0] iz_dy,    // d(1/z)/dy per pixel
    input  logic [23:0] color,
    output logic done,
    // Framebuffer write (tile-local addressing)
    output logic [ADDR_WIDTH-1:0] fb_addr,
    output logic [63:0]           fb_data,
    output logic [3:0]            fb_mask,
    // Z-buffer interface (tile-local addressing)
    output logic [ADDR_WIDTH-1:0] zb_rd_addr,
    input  logic [15:0]           zb_rd_data [4],
    output logic [ADDR_WIDTH-1:0] zb_wr_addr,
    output logic [15:0]           zb_wr_data [4],
    output logic [3:0]            zb_wr_mask
);

    localparam CW = $clog2(RES_W);   // pixel coordinate width
    localparam CH = $clog2(RES_H);
    localparam VW = CW + SUBPIXEL;    // vertex coordinate width (with sub-pixel)
    localparam VH = CH + SUBPIXEL;
    localparam EW = VW + VH + 1;      // edge function width
    localparam TWB = $clog2(TILE_W); // tile width in bits (for BRAM addressing)
    localparam THB = $clog2(TILE_H);

    typedef enum logic [2:0] {
        IDLE,
        SETUP,
        INIT_ROW,
        TRAVERSE
    } state_t;

    state_t state;

    // Bounding box (pixel coordinates)
    logic [CW-1:0] minx, maxx;
    logic [CH-1:0] miny, maxy;
    // qx/qy widened by 1 bit to avoid wraparound when qy + 2 exceeds maxy near 2^CH
    logic [CW:0] qx;
    logic [CH:0] qy;

    // Edge function values (sub-pixel precision)
    logic signed [EW-1:0] e0_row, e1_row, e2_row;
    logic signed [EW-1:0] e0_col, e1_col, e2_col;
    logic signed [EW-1:0] e0_dx, e1_dx, e2_dx;  // per-pixel step in x
    logic signed [EW-1:0] e0_dy, e1_dy, e2_dy;  // per-pixel step in y

    // 1/z interpolation (pre-computed plane, 16-bit fixed-point)
    logic signed [15:0] iz_row, iz_col;
    logic signed [15:0] iz_dx_r, iz_dy_r;

    // Top-left rule
    logic tl0, tl1, tl2;

    // Convert 24-bit RGB to 16-bit RGB565
    logic [15:0] color_565;
    assign color_565 = {color[23:19], color[15:10], color[7:3]};

    // Edge function at a sub-pixel point
    function automatic logic signed [EW-1:0] edge_func(
        input logic [VW-1:0] v0x, input logic [VH-1:0] v0y,
        input logic [VW-1:0] v1x, input logic [VH-1:0] v1y,
        input logic [VW-1:0] px,  input logic [VH-1:0] py
    );
        logic signed [VW:0] dx, dpx;
        logic signed [VH:0] dy, dpy;
        dx  = $signed({1'b0, v1x}) - $signed({1'b0, v0x});
        dy  = $signed({1'b0, v1y}) - $signed({1'b0, v0y});
        dpx = $signed({1'b0, px})  - $signed({1'b0, v0x});
        dpy = $signed({1'b0, py})  - $signed({1'b0, v0y});
        return EW'(dx * dpy) - EW'(dy * dpx);
    endfunction

    function automatic logic [CW-1:0] min3x(input logic [CW-1:0] a, b, c);
        return (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    endfunction
    function automatic logic [CW-1:0] max3x(input logic [CW-1:0] a, b, c);
        return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    endfunction
    function automatic logic [CH-1:0] min3y(input logic [CH-1:0] a, b, c);
        return (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    endfunction
    function automatic logic [CH-1:0] max3y(input logic [CH-1:0] a, b, c);
        return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    endfunction

    // --- Combinational setup signals ---
    // Edge deltas per pixel: stepping 1 pixel = stepping (1<<SUBPIXEL) in sub-pixel coords
    // e_dx = -(v1_y - v0_y) is the edge function change per +1 sub-pixel in x
    // Per pixel step = e_dx * (1 << SUBPIXEL), but we compute it directly as:
    // e_dx_pixel = -(v1_y - v0_y) << SUBPIXEL... No!
    // Actually: edge_func uses sub-pixel coords. The per-pixel-step of the edge function is:
    // delta_x = (v1_y - v0_y) applied to a 1-pixel step = -(v1_y - v0_y) * 1_pixel
    // But since coords are in sub-pixel units, 1 pixel = (1 << SUBPIXEL) sub-pixel units.
    // So: e_dx_per_pixel = -(v1_y - v0_y) * (1 << SUBPIXEL)... No, that's wrong too.
    //
    // The edge function is: E(px,py) = (v1x-v0x)*(py-v0y) - (v1y-v0y)*(px-v0x)
    // dE/dpx = -(v1y - v0y)  (in sub-pixel units)
    // When we step 1 pixel in x, px changes by (1 << SUBPIXEL), so:
    // delta_E_per_pixel_x = -(v1y - v0y) * (1 << SUBPIXEL)
    //
    // Similarly: delta_E_per_pixel_y = (v1x - v0x) * (1 << SUBPIXEL)
    //
    // But it's simpler: just compute the edge function at pixel centers.
    // Pixel center (px, py) in sub-pixel coords = (px << SUBPIXEL) + (1 << (SUBPIXEL-1))
    // i.e., sample at the center of the pixel.

    logic signed [EW-1:0] setup_e0_dx, setup_e1_dx, setup_e2_dx;
    logic signed [EW-1:0] setup_e0_dy, setup_e1_dy, setup_e2_dy;
    logic signed [EW-1:0] setup_e0_init, setup_e1_init, setup_e2_init;
    logic [CW-1:0] bbox_minx, bbox_maxx;
    logic [CH-1:0] bbox_miny, bbox_maxy;

    // Pixel center in sub-pixel coords for the bounding box origin
    logic [VW-1:0] origin_x;
    logic [VH-1:0] origin_y;

    always_comb begin
        // Per-pixel edge deltas (sub-pixel scale)
        // dE/dx_pixel = -(v1y - v0y) * (1 << SUBPIXEL)
        setup_e0_dx = -(EW'($signed({1'b0, v1_y}) - $signed({1'b0, v0_y}))) <<< SUBPIXEL;
        setup_e0_dy =  (EW'($signed({1'b0, v1_x}) - $signed({1'b0, v0_x}))) <<< SUBPIXEL;
        setup_e1_dx = -(EW'($signed({1'b0, v2_y}) - $signed({1'b0, v1_y}))) <<< SUBPIXEL;
        setup_e1_dy =  (EW'($signed({1'b0, v2_x}) - $signed({1'b0, v1_x}))) <<< SUBPIXEL;
        setup_e2_dx = -(EW'($signed({1'b0, v0_y}) - $signed({1'b0, v2_y}))) <<< SUBPIXEL;
        setup_e2_dy =  (EW'($signed({1'b0, v0_x}) - $signed({1'b0, v2_x}))) <<< SUBPIXEL;

        // Bounding box in pixel coords (truncate sub-pixel bits)
        bbox_minx = min3x(v0_x[VW-1:SUBPIXEL], v1_x[VW-1:SUBPIXEL], v2_x[VW-1:SUBPIXEL]);
        bbox_miny = min3y(v0_y[VH-1:SUBPIXEL], v1_y[VH-1:SUBPIXEL], v2_y[VH-1:SUBPIXEL]);
        bbox_maxx = max3x(v0_x[VW-1:SUBPIXEL], v1_x[VW-1:SUBPIXEL], v2_x[VW-1:SUBPIXEL]);
        bbox_maxy = max3y(v0_y[VH-1:SUBPIXEL], v1_y[VH-1:SUBPIXEL], v2_y[VH-1:SUBPIXEL]);

        // Pixel center of bounding box origin in sub-pixel coords
        origin_x = {minx, {SUBPIXEL{1'b0}}} | VW'(1 << (SUBPIXEL - 1));
        origin_y = {miny, {SUBPIXEL{1'b0}}} | VH'(1 << (SUBPIXEL - 1));

        // Edge function at the pixel center of the bounding box origin
        setup_e0_init = edge_func(v0_x, v0_y, v1_x, v1_y, origin_x, origin_y);
        setup_e1_init = edge_func(v1_x, v1_y, v2_x, v2_y, origin_x, origin_y);
        setup_e2_init = edge_func(v2_x, v2_y, v0_x, v0_y, origin_x, origin_y);
    end

    // --- Pipeline stage 1 registers ---
    logic p1_valid;
    logic signed [EW-1:0] p1_e0, p1_e1, p1_e2;
    logic signed [EW-1:0] p1_e0_dx, p1_e1_dx, p1_e2_dx;
    logic signed [EW-1:0] p1_e0_dy, p1_e1_dy, p1_e2_dy;
    logic signed [EW-1:0] p1_e0_dxy, p1_e1_dxy, p1_e2_dxy;
    logic p1_tl0, p1_tl1, p1_tl2;
    logic signed [15:0] p1_iz [4];
    logic [CW-1:0] p1_qx;
    logic [CH-1:0] p1_qy;
    logic [CW-1:0] p1_maxx;
    logic [CH-1:0] p1_maxy;
    logic [15:0]   p1_color;

    // --- Pipeline stage 2 registers ---
    logic p2_valid;
    logic [3:0] p2_inside;
    logic [15:0] p2_iz [4];
    logic [ADDR_WIDTH-1:0] p2_addr;
    logic [15:0] p2_color;

    // Tile bbox clamping (combinational)
    logic [CW-1:0] tile_xmax_p1;
    logic [CH-1:0] tile_ymax_p1;
    logic [CW-1:0] clamped_minx, clamped_maxx;
    logic [CH-1:0] clamped_miny, clamped_maxy;
    logic          bbox_empty;

    always_comb begin
        tile_xmax_p1 = tile_x + CW'(TILE_W);
        tile_ymax_p1 = tile_y + CH'(TILE_H);
        clamped_minx = (bbox_minx < tile_x) ? tile_x : bbox_minx;
        clamped_miny = (bbox_miny < tile_y) ? tile_y : bbox_miny;
        clamped_maxx = (bbox_maxx >= tile_xmax_p1) ? tile_xmax_p1 - 1 : bbox_maxx;
        clamped_maxy = (bbox_maxy >= tile_ymax_p1) ? tile_ymax_p1 - 1 : bbox_maxy;
        bbox_empty = (clamped_minx > clamped_maxx) || (clamped_miny > clamped_maxy);
    end

    // Traversal FSM
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            p1_valid <= 0;
        end else begin
            p1_valid <= 0;

            case (state)
                IDLE: begin
                    done <= 0;
                    if (start) begin
                        if (bbox_empty) begin
                            done <= 1;
                        end else begin
                            minx <= {clamped_minx[CW-1:1], 1'b0};
                            miny <= {clamped_miny[CH-1:1], 1'b0};
                            maxx <= clamped_maxx;
                            maxy <= clamped_maxy;
                            state <= SETUP;
                        end
                    end
                end

                SETUP: begin
                    e0_dx <= setup_e0_dx; e0_dy <= setup_e0_dy;
                    e1_dx <= setup_e1_dx; e1_dy <= setup_e1_dy;
                    e2_dx <= setup_e2_dx; e2_dy <= setup_e2_dy;
                    e0_row <= setup_e0_init;
                    e1_row <= setup_e1_init;
                    e2_row <= setup_e2_init;

                    // Top-left rule (based on sub-pixel edge direction)
                    tl0 <= (setup_e0_dx > 0) || (setup_e0_dx == 0 && setup_e0_dy < 0);
                    tl1 <= (setup_e1_dx > 0) || (setup_e1_dx == 0 && setup_e1_dy < 0);
                    tl2 <= (setup_e2_dx > 0) || (setup_e2_dx == 0 && setup_e2_dy < 0);

                    // iz_init is 1/z at pixel (0,0); compute value at bbox origin
                    iz_dx_r <= iz_dx;
                    iz_dy_r <= iz_dy;
                    iz_row <= iz_init + iz_dx * $signed({1'b0, minx}) + iz_dy * $signed({1'b0, miny});

                    qx <= minx;
                    qy <= miny;
                    state <= INIT_ROW;
                end

                INIT_ROW: begin
                    e0_col <= e0_row;
                    e1_col <= e1_row;
                    e2_col <= e2_row;
                    iz_col <= iz_row;
                    state <= TRAVERSE;
                end

                TRAVERSE: begin
                    if (qy > maxy) begin
                        done <= 1;
                        state <= IDLE;
                    end else begin
                        p1_valid <= 1;
                        p1_e0 <= e0_col;
                        p1_e1 <= e1_col;
                        p1_e2 <= e2_col;
                        p1_e0_dx <= e0_col + e0_dx;
                        p1_e1_dx <= e1_col + e1_dx;
                        p1_e2_dx <= e2_col + e2_dx;
                        p1_e0_dy <= e0_col + e0_dy;
                        p1_e1_dy <= e1_col + e1_dy;
                        p1_e2_dy <= e2_col + e2_dy;
                        p1_e0_dxy <= e0_col + e0_dx + e0_dy;
                        p1_e1_dxy <= e1_col + e1_dx + e1_dy;
                        p1_e2_dxy <= e2_col + e2_dx + e2_dy;

                        p1_tl0 <= tl0;
                        p1_tl1 <= tl1;
                        p1_tl2 <= tl2;

                        p1_iz[0] <= iz_col;
                        p1_iz[1] <= iz_col + iz_dx_r;
                        p1_iz[2] <= iz_col + iz_dy_r;
                        p1_iz[3] <= iz_col + iz_dx_r + iz_dy_r;

                        p1_qx <= qx;
                        p1_qy <= qy;
                        p1_maxx <= maxx;
                        p1_maxy <= maxy;
                        p1_color <= color_565;

                        if (qx + 2 > maxx) begin
                            qx <= minx;
                            qy <= qy + 2;
                            e0_row <= e0_row + (e0_dy << 1);
                            e1_row <= e1_row + (e1_dy << 1);
                            e2_row <= e2_row + (e2_dy << 1);
                            iz_row <= iz_row + (iz_dy_r <<< 1);
                            state <= INIT_ROW;
                        end else begin
                            qx <= qx + 2;
                            e0_col <= e0_col + (e0_dx << 1);
                            e1_col <= e1_col + (e1_dx << 1);
                            e2_col <= e2_col + (e2_dx << 1);
                            iz_col <= iz_col + (iz_dx_r <<< 1);
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // --- Pipeline stage 1: inside mask + z-buffer read ---
    // Tile-local quad address: ((qy - tile_y) >> 1) * (TILE_W >> 1) + ((qx - tile_x) >> 1)
    logic [ADDR_WIDTH-1:0] p1_addr;
    logic [CW-1:0] p1_qx_local;
    logic [CH-1:0] p1_qy_local;
    assign p1_qx_local = p1_qx - tile_x;
    assign p1_qy_local = p1_qy - tile_y;
    assign p1_addr = ADDR_WIDTH'(p1_qy_local >> 1) * ADDR_WIDTH'(TILE_W >> 1) + ADDR_WIDTH'(p1_qx_local >> 1);
    assign zb_rd_addr = p1_addr;

    always_ff @(posedge clk) begin
        if (rst) begin
            p2_valid <= 0;
        end else begin
            p2_valid <= p1_valid;
            if (p1_valid) begin
                p2_inside[0] <= (p1_qy <= p1_maxy && p1_qx <= p1_maxx &&
                                 (p1_e0 > 0 || (p1_e0 == 0 && p1_tl0)) &&
                                 (p1_e1 > 0 || (p1_e1 == 0 && p1_tl1)) &&
                                 (p1_e2 > 0 || (p1_e2 == 0 && p1_tl2)));
                p2_inside[1] <= (p1_qy <= p1_maxy && (p1_qx + 1) <= p1_maxx &&
                                 (p1_e0_dx > 0 || (p1_e0_dx == 0 && p1_tl0)) &&
                                 (p1_e1_dx > 0 || (p1_e1_dx == 0 && p1_tl1)) &&
                                 (p1_e2_dx > 0 || (p1_e2_dx == 0 && p1_tl2)));
                p2_inside[2] <= ((p1_qy + 1) <= p1_maxy && p1_qx <= p1_maxx &&
                                 (p1_e0_dy > 0 || (p1_e0_dy == 0 && p1_tl0)) &&
                                 (p1_e1_dy > 0 || (p1_e1_dy == 0 && p1_tl1)) &&
                                 (p1_e2_dy > 0 || (p1_e2_dy == 0 && p1_tl2)));
                p2_inside[3] <= ((p1_qy + 1) <= p1_maxy && (p1_qx + 1) <= p1_maxx &&
                                 (p1_e0_dxy > 0 || (p1_e0_dxy == 0 && p1_tl0)) &&
                                 (p1_e1_dxy > 0 || (p1_e1_dxy == 0 && p1_tl1)) &&
                                 (p1_e2_dxy > 0 || (p1_e2_dxy == 0 && p1_tl2)));

                for (int i = 0; i < 4; i++)
                    p2_iz[i] <= p1_iz[i];

                p2_addr <= p1_addr;
                p2_color <= p1_color;
            end
        end
    end

    // --- Pipeline stage 2: z-compare and write ---
    always_ff @(posedge clk) begin
        if (rst) begin
            fb_mask <= 0;
            zb_wr_mask <= 0;
        end else if (p2_valid) begin
            fb_addr <= p2_addr;
            fb_data <= {p2_color, p2_color, p2_color, p2_color};
            zb_wr_addr <= p2_addr;
            zb_wr_data <= p2_iz;
            for (int i = 0; i < 4; i++) begin
                fb_mask[i] <= p2_inside[i] && (p2_iz[i] >= zb_rd_data[i]);
                zb_wr_mask[i] <= p2_inside[i] && (p2_iz[i] >= zb_rd_data[i]);
            end
        end else begin
            fb_mask <= 0;
            zb_wr_mask <= 0;
        end
    end

endmodule
