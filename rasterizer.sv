module rasterizer #(
    parameter RWIDTH = 320,
    parameter RHEIGHT = 200,
    parameter ADDR_WIDTH = $clog2((RWIDTH/2) * (RHEIGHT/2))
) (
    input  logic clk,
    input  logic rst,
    // Triangle input
    input  logic start,
    input  logic [$clog2(RWIDTH)-1:0] v0_x,
    input  logic [$clog2(RHEIGHT)-1:0] v0_y,
    input  logic [$clog2(RWIDTH)-1:0] v1_x,
    input  logic [$clog2(RHEIGHT)-1:0] v1_y,
    input  logic [$clog2(RWIDTH)-1:0] v2_x,
    input  logic [$clog2(RHEIGHT)-1:0] v2_y,
    input  logic [15:0] v0_iz, v1_iz, v2_iz,
    input  logic [23:0] color,
    output logic done,
    // Framebuffer write
    output logic [ADDR_WIDTH-1:0] fb_addr,
    output logic [63:0]           fb_data,
    output logic [3:0]            fb_mask,
    // Z-buffer interface
    output logic [ADDR_WIDTH-1:0] zb_rd_addr,
    input  logic [15:0]           zb_rd_data [4],
    output logic [ADDR_WIDTH-1:0] zb_wr_addr,
    output logic [15:0]           zb_wr_data [4],
    output logic [3:0]            zb_wr_mask
);

    localparam CW = $clog2(RWIDTH);
    localparam CH = $clog2(RHEIGHT);
    localparam EW = CW + CH + 1;
    localparam IZ_FRAC = 16 + EW;

    typedef enum logic [2:0] {
        IDLE,
        SETUP,
        INIT_ROW,
        TRAVERSE
    } state_t;

    state_t state;

    // Bounding box
    logic [CW-1:0] minx, maxx;
    logic [CH-1:0] miny, maxy;
    logic [CW-1:0] qx;
    logic [CH-1:0] qy;

    // Edge function values
    logic signed [EW-1:0] e0_row, e1_row, e2_row;
    logic signed [EW-1:0] e0_col, e1_col, e2_col;
    logic signed [EW-1:0] e0_dx, e1_dx, e2_dx;
    logic signed [EW-1:0] e0_dy, e1_dy, e2_dy;

    // 1/z interpolation
    logic signed [IZ_FRAC-1:0] iz_row, iz_col;
    logic signed [IZ_FRAC-1:0] iz_dx, iz_dy;

    // Convert 24-bit RGB to 16-bit RGB565
    logic [15:0] color_565;
    assign color_565 = {color[23:19], color[15:10], color[7:3]};

    function automatic logic signed [EW-1:0] edge_func(
        input logic [CW-1:0] v0x, input logic [CH-1:0] v0y,
        input logic [CW-1:0] v1x, input logic [CH-1:0] v1y,
        input logic [CW-1:0] px,  input logic [CH-1:0] py
    );
        logic signed [CW:0] dx, dpx;
        logic signed [CH:0] dy, dpy;
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

    // --- Combinational setup signals (used in SETUP state) ---
    logic signed [EW-1:0] setup_e0_dx, setup_e1_dx, setup_e2_dx;
    logic signed [EW-1:0] setup_e0_dy, setup_e1_dy, setup_e2_dy;
    logic signed [EW-1:0] setup_e0_init, setup_e1_init, setup_e2_init;
    logic [CW-1:0] bbox_minx, bbox_maxx;
    logic [CH-1:0] bbox_miny, bbox_maxy;

    always_comb begin
        setup_e0_dx = -EW'($signed({1'b0, v1_y}) - $signed({1'b0, v0_y}));
        setup_e0_dy =  EW'($signed({1'b0, v1_x}) - $signed({1'b0, v0_x}));
        setup_e1_dx = -EW'($signed({1'b0, v2_y}) - $signed({1'b0, v1_y}));
        setup_e1_dy =  EW'($signed({1'b0, v2_x}) - $signed({1'b0, v1_x}));
        setup_e2_dx = -EW'($signed({1'b0, v0_y}) - $signed({1'b0, v2_y}));
        setup_e2_dy =  EW'($signed({1'b0, v0_x}) - $signed({1'b0, v2_x}));

        setup_e0_init = edge_func(v0_x, v0_y, v1_x, v1_y, minx, miny);
        setup_e1_init = edge_func(v1_x, v1_y, v2_x, v2_y, minx, miny);
        setup_e2_init = edge_func(v2_x, v2_y, v0_x, v0_y, minx, miny);

        bbox_minx = min3x(v0_x, v1_x, v2_x);
        bbox_miny = min3y(v0_y, v1_y, v2_y);
        bbox_maxx = max3x(v0_x, v1_x, v2_x);
        bbox_maxy = max3y(v0_y, v1_y, v2_y);
    end

    // --- Pipeline stage 1 registers ---
    logic p1_valid;
    logic signed [EW-1:0] p1_e0, p1_e1, p1_e2;
    logic signed [EW-1:0] p1_e0_dx, p1_e1_dx, p1_e2_dx;
    logic signed [EW-1:0] p1_e0_dy, p1_e1_dy, p1_e2_dy;
    logic signed [EW-1:0] p1_e0_dxy, p1_e1_dxy, p1_e2_dxy;
    logic signed [IZ_FRAC-1:0] p1_iz [4];
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
                        minx <= {bbox_minx[CW-1:1], 1'b0};
                        miny <= {bbox_miny[CH-1:1], 1'b0};
                        maxx <= (bbox_maxx >= CW'(RWIDTH))
                                ? CW'(RWIDTH - 1) : bbox_maxx;
                        maxy <= (bbox_maxy >= CH'(RHEIGHT))
                                ? CH'(RHEIGHT - 1) : bbox_maxy;
                        state <= SETUP;
                    end
                end

                SETUP: begin
                    e0_dx <= setup_e0_dx; e0_dy <= setup_e0_dy;
                    e1_dx <= setup_e1_dx; e1_dy <= setup_e1_dy;
                    e2_dx <= setup_e2_dx; e2_dy <= setup_e2_dy;
                    e0_row <= setup_e0_init;
                    e1_row <= setup_e1_init;
                    e2_row <= setup_e2_init;

                    // 1/z plane: iz = e1*v0_iz + e2*v1_iz + e0*v2_iz
                    // Multiplies are EW × 17 bits (one DSP each)
                    iz_dx <= IZ_FRAC'(setup_e1_dx * $signed({1'b0, v0_iz}))
                           + IZ_FRAC'(setup_e2_dx * $signed({1'b0, v1_iz}))
                           + IZ_FRAC'(setup_e0_dx * $signed({1'b0, v2_iz}));
                    iz_dy <= IZ_FRAC'(setup_e1_dy * $signed({1'b0, v0_iz}))
                           + IZ_FRAC'(setup_e2_dy * $signed({1'b0, v1_iz}))
                           + IZ_FRAC'(setup_e0_dy * $signed({1'b0, v2_iz}));
                    iz_row <= IZ_FRAC'(setup_e1_init * $signed({1'b0, v0_iz}))
                            + IZ_FRAC'(setup_e2_init * $signed({1'b0, v1_iz}))
                            + IZ_FRAC'(setup_e0_init * $signed({1'b0, v2_iz}));

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

                        p1_iz[0] <= iz_col;
                        p1_iz[1] <= iz_col + iz_dx;
                        p1_iz[2] <= iz_col + iz_dy;
                        p1_iz[3] <= iz_col + iz_dx + iz_dy;

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
                            iz_row <= iz_row + (iz_dy <<< 1);
                            state <= INIT_ROW;
                        end else begin
                            qx <= qx + 2;
                            e0_col <= e0_col + (e0_dx << 1);
                            e1_col <= e1_col + (e1_dx << 1);
                            e2_col <= e2_col + (e2_dx << 1);
                            iz_col <= iz_col + (iz_dx <<< 1);
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // --- Pipeline stage 1: inside mask + z-buffer read ---
    logic [ADDR_WIDTH-1:0] p1_addr;
    assign p1_addr = ADDR_WIDTH'(p1_qy >> 1) * ADDR_WIDTH'(RWIDTH >> 1) + ADDR_WIDTH'(p1_qx >> 1);
    assign zb_rd_addr = p1_addr;

    always_ff @(posedge clk) begin
        if (rst) begin
            p2_valid <= 0;
        end else begin
            p2_valid <= p1_valid;
            if (p1_valid) begin
                p2_inside[0] <= (p1_qy <= p1_maxy && p1_qx <= p1_maxx &&
                                 p1_e0 >= 0 && p1_e1 >= 0 && p1_e2 >= 0);
                p2_inside[1] <= (p1_qy <= p1_maxy && (p1_qx + 1) <= p1_maxx &&
                                 p1_e0_dx >= 0 && p1_e1_dx >= 0 && p1_e2_dx >= 0);
                p2_inside[2] <= ((p1_qy + 1) <= p1_maxy && p1_qx <= p1_maxx &&
                                 p1_e0_dy >= 0 && p1_e1_dy >= 0 && p1_e2_dy >= 0);
                p2_inside[3] <= ((p1_qy + 1) <= p1_maxy && (p1_qx + 1) <= p1_maxx &&
                                 p1_e0_dxy >= 0 && p1_e1_dxy >= 0 && p1_e2_dxy >= 0);

                for (int i = 0; i < 4; i++)
                    p2_iz[i] <= p1_iz[i][IZ_FRAC-1 -: 16];

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
