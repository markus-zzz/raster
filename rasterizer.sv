module rasterizer #(
    parameter RWIDTH = 320,
    parameter RHEIGHT = 240,
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
    input  logic [23:0] color,
    output logic done,
    // Framebuffer write (one 64-bit quad per cycle)
    output logic [ADDR_WIDTH-1:0] fb_addr,
    output logic [63:0]           fb_data,
    output logic [3:0]            fb_mask
);

    localparam CW = $clog2(RWIDTH);
    localparam CH = $clog2(RHEIGHT);
    localparam EW = CW + CH + 1;

    // --- Traversal FSM ---
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

    // --- Pipeline stage: write to FB (1 cycle behind traversal) ---
    logic pipe_valid;
    logic signed [EW-1:0] pipe_e0, pipe_e1, pipe_e2;           // p0 (top-left)
    logic signed [EW-1:0] pipe_e0_dx, pipe_e1_dx, pipe_e2_dx;  // p1 = p0 + dx
    logic signed [EW-1:0] pipe_e0_dy, pipe_e1_dy, pipe_e2_dy;  // p2 = p0 + dy
    logic signed [EW-1:0] pipe_e0_dxy, pipe_e1_dxy, pipe_e2_dxy; // p3 = p0 + dx + dy
    logic [CW-1:0] pipe_qx;
    logic [CH-1:0] pipe_qy;
    logic [CW-1:0] pipe_maxx;
    logic [CH-1:0] pipe_maxy;
    logic [15:0]   pipe_color;

    // Traversal FSM
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            pipe_valid <= 0;
        end else begin
            pipe_valid <= 0;

            case (state)
                IDLE: begin
                    done <= 0;
                    if (start) begin
                        logic [CW-1:0] tmp_minx, tmp_maxx;
                        logic [CH-1:0] tmp_miny, tmp_maxy;
                        tmp_minx = min3x(v0_x, v1_x, v2_x);
                        tmp_miny = min3y(v0_y, v1_y, v2_y);
                        tmp_maxx = max3x(v0_x, v1_x, v2_x);
                        tmp_maxy = max3y(v0_y, v1_y, v2_y);

                        minx <= {tmp_minx[CW-1:1], 1'b0};
                        miny <= {tmp_miny[CH-1:1], 1'b0};
                        maxx <= (tmp_maxx >= CW'(RWIDTH))  ? CW'(RWIDTH - 1)  : tmp_maxx;
                        maxy <= (tmp_maxy >= CH'(RHEIGHT)) ? CH'(RHEIGHT - 1) : tmp_maxy;
                        state <= SETUP;
                    end
                end

                SETUP: begin
                    e0_dx <= -EW'($signed({1'b0, v1_y}) - $signed({1'b0, v0_y}));
                    e0_dy <=  EW'($signed({1'b0, v1_x}) - $signed({1'b0, v0_x}));
                    e1_dx <= -EW'($signed({1'b0, v2_y}) - $signed({1'b0, v1_y}));
                    e1_dy <=  EW'($signed({1'b0, v2_x}) - $signed({1'b0, v1_x}));
                    e2_dx <= -EW'($signed({1'b0, v0_y}) - $signed({1'b0, v2_y}));
                    e2_dy <=  EW'($signed({1'b0, v0_x}) - $signed({1'b0, v2_x}));

                    e0_row <= edge_func(v0_x, v0_y, v1_x, v1_y, minx, miny);
                    e1_row <= edge_func(v1_x, v1_y, v2_x, v2_y, minx, miny);
                    e2_row <= edge_func(v2_x, v2_y, v0_x, v0_y, minx, miny);

                    qx <= minx;
                    qy <= miny;
                    state <= INIT_ROW;
                end

                INIT_ROW: begin
                    e0_col <= e0_row;
                    e1_col <= e1_row;
                    e2_col <= e2_row;
                    state <= TRAVERSE;
                end

                TRAVERSE: begin
                    if (qy > maxy) begin
                        done <= 1;
                        state <= IDLE;
                    end else begin
                        // Feed pipeline with current quad's edge values
                        pipe_valid <= 1;
                        pipe_e0 <= e0_col;
                        pipe_e1 <= e1_col;
                        pipe_e2 <= e2_col;
                        pipe_e0_dx <= e0_col + e0_dx;
                        pipe_e1_dx <= e1_col + e1_dx;
                        pipe_e2_dx <= e2_col + e2_dx;
                        pipe_e0_dy <= e0_col + e0_dy;
                        pipe_e1_dy <= e1_col + e1_dy;
                        pipe_e2_dy <= e2_col + e2_dy;
                        pipe_e0_dxy <= e0_col + e0_dx + e0_dy;
                        pipe_e1_dxy <= e1_col + e1_dx + e1_dy;
                        pipe_e2_dxy <= e2_col + e2_dx + e2_dy;
                        pipe_qx <= qx;
                        pipe_qy <= qy;
                        pipe_maxx <= maxx;
                        pipe_maxy <= maxy;
                        pipe_color <= color_565;

                        // Advance to next quad
                        if (qx + 2 > maxx) begin
                            qx <= minx;
                            qy <= qy + 2;
                            e0_row <= e0_row + (e0_dy << 1);
                            e1_row <= e1_row + (e1_dy << 1);
                            e2_row <= e2_row + (e2_dy << 1);
                            // Check if this was the last row
                            state <= INIT_ROW;
                        end else begin
                            qx <= qx + 2;
                            e0_col <= e0_col + (e0_dx << 1);
                            e1_col <= e1_col + (e1_dx << 1);
                            e2_col <= e2_col + (e2_dx << 1);
                        end
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // --- Pipeline output stage: compute mask & address, drive FB ---
    always_ff @(posedge clk) begin
        if (rst) begin
            fb_mask <= 0;
        end else if (pipe_valid) begin
            logic [3:0] mask;
            mask = {
                ((pipe_qy + 1) <= pipe_maxy && (pipe_qx + 1) <= pipe_maxx &&
                    pipe_e0_dxy >= 0 && pipe_e1_dxy >= 0 && pipe_e2_dxy >= 0),
                ((pipe_qy + 1) <= pipe_maxy && pipe_qx <= pipe_maxx &&
                    pipe_e0_dy >= 0 && pipe_e1_dy >= 0 && pipe_e2_dy >= 0),
                (pipe_qy <= pipe_maxy && (pipe_qx + 1) <= pipe_maxx &&
                    pipe_e0_dx >= 0 && pipe_e1_dx >= 0 && pipe_e2_dx >= 0),
                (pipe_qy <= pipe_maxy && pipe_qx <= pipe_maxx &&
                    pipe_e0 >= 0 && pipe_e1 >= 0 && pipe_e2 >= 0)
            };

            fb_addr <= ADDR_WIDTH'(pipe_qy >> 1) * ADDR_WIDTH'(RWIDTH >> 1) + ADDR_WIDTH'(pipe_qx >> 1);
            fb_data <= {pipe_color, pipe_color, pipe_color, pipe_color};
            fb_mask <= mask;
        end else begin
            fb_mask <= 0;
        end
    end

endmodule
