module rasterizer #(
    parameter WIDTH = 320,
    parameter ADDR_WIDTH = 16
) (
    input  logic clk,
    input  logic rst,
    // Triangle input
    input  logic start,
    input  logic [9:0] v0_x, v0_y,
    input  logic [9:0] v1_x, v1_y,
    input  logic [9:0] v2_x, v2_y,
    input  logic [23:0] color,
    output logic done,
    // Framebuffer write (one 64-bit quad per cycle)
    output logic                  fb_we,
    output logic [ADDR_WIDTH-1:0] fb_addr,
    output logic [63:0]           fb_data,
    output logic [3:0]            fb_mask  // byte enable for each pixel
);

    typedef enum logic [3:0] {
        IDLE,
        SETUP,
        INIT_QUAD,
        RASTER_QUAD,
        WRITE_QUAD
    } state_t;

    state_t state;

    // Bounding box
    logic [9:0] minx, miny, maxx, maxy;
    logic [9:0] qx, qy;

    // Edge function values
    logic signed [20:0] e0_row, e1_row, e2_row;
    logic signed [20:0] e0_col, e1_col, e2_col;
    logic signed [20:0] e0_dx, e1_dx, e2_dx;
    logic signed [20:0] e0_dy, e1_dy, e2_dy;

    // Quad pixel values
    logic signed [20:0] p0_e0, p0_e1, p0_e2;
    logic signed [20:0] p1_e0, p1_e1, p1_e2;
    logic signed [20:0] p2_e0, p2_e1, p2_e2;
    logic signed [20:0] p3_e0, p3_e1, p3_e2;

    // Convert 24-bit RGB to 16-bit RGB565
    logic [15:0] color_565;
    assign color_565 = {color[23:19], color[15:10], color[7:3]};

    function automatic logic signed [20:0] edge_func(
        input logic [9:0] v0x, v0y, v1x, v1y, px, py
    );
        logic signed [10:0] dx, dy, dpx, dpy;
        dx = $signed({1'b0, v1x}) - $signed({1'b0, v0x});
        dy = $signed({1'b0, v1y}) - $signed({1'b0, v0y});
        dpx = $signed({1'b0, px}) - $signed({1'b0, v0x});
        dpy = $signed({1'b0, py}) - $signed({1'b0, v0y});
        return dx * dpy - dy * dpx;
    endfunction

    function automatic logic [9:0] min3(input logic [9:0] a, b, c);
        return (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    endfunction

    function automatic logic [9:0] max3(input logic [9:0] a, b, c);
        return (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            fb_we <= 0;
        end else begin
            fb_we <= 0;
            
            
            case (state)
                IDLE: begin
                    done <= 0;
                    if (start) begin
                        // Compute bounding box aligned to 2x2 grid
                        logic [9:0] tmp_minx, tmp_miny, tmp_maxx, tmp_maxy;
                        tmp_minx = min3(v0_x, v1_x, v2_x);
                        tmp_miny = min3(v0_y, v1_y, v2_y);
                        tmp_maxx = max3(v0_x, v1_x, v2_x);
                        tmp_maxy = max3(v0_y, v1_y, v2_y);
                        
                        minx <= {tmp_minx[9:1], 1'b0};
                        miny <= {tmp_miny[9:1], 1'b0};
                        maxx <= (tmp_maxx >= WIDTH) ? (WIDTH - 1) : tmp_maxx;
                        maxy <= (tmp_maxy >= 10'd200) ? 10'd199 : tmp_maxy;
                        state <= SETUP;
                    end
                end

                SETUP: begin
                    // Compute edge deltas (signed 11-bit differences)
                    e0_dx <= -21'($signed({1'b0, v1_y}) - $signed({1'b0, v0_y}));
                    e0_dy <=  21'($signed({1'b0, v1_x}) - $signed({1'b0, v0_x}));
                    e1_dx <= -21'($signed({1'b0, v2_y}) - $signed({1'b0, v1_y}));
                    e1_dy <=  21'($signed({1'b0, v2_x}) - $signed({1'b0, v1_x}));
                    e2_dx <= -21'($signed({1'b0, v0_y}) - $signed({1'b0, v2_y}));
                    e2_dy <=  21'($signed({1'b0, v0_x}) - $signed({1'b0, v2_x}));

                    // Initial edge values at origin (minx, miny)
                    e0_row <= edge_func(v0_x, v0_y, v1_x, v1_y, minx, miny);
                    e1_row <= edge_func(v1_x, v1_y, v2_x, v2_y, minx, miny);
                    e2_row <= edge_func(v2_x, v2_y, v0_x, v0_y, minx, miny);

                    qx <= minx;
                    qy <= miny;
                    state <= INIT_QUAD;
                end
                
                INIT_QUAD: begin
                    // Initialize column values for first quad of row
                    e0_col <= e0_row;
                    e1_col <= e1_row;
                    e2_col <= e2_row;
                    state <= RASTER_QUAD;
                end

                RASTER_QUAD: begin
                    if (qy > maxy) begin
                        state <= IDLE;
                        done <= 1;
                    end else begin
                        // Compute all 4 pixels of quad
                        p0_e0 <= e0_col;
                        p0_e1 <= e1_col;
                        p0_e2 <= e2_col;
                        
                        p1_e0 <= e0_col + e0_dx;
                        p1_e1 <= e1_col + e1_dx;
                        p1_e2 <= e2_col + e2_dx;
                        
                        p2_e0 <= e0_col + e0_dy;
                        p2_e1 <= e1_col + e1_dy;
                        p2_e2 <= e2_col + e2_dy;
                        
                        p3_e0 <= e0_col + e0_dx + e0_dy;
                        p3_e1 <= e1_col + e1_dx + e1_dy;
                        p3_e2 <= e2_col + e2_dx + e2_dy;

                        state <= WRITE_QUAD;
                    end
                end

                WRITE_QUAD: begin
                    // Compute quad address (qx/2, qy/2)
                    logic [ADDR_WIDTH-1:0] quad_addr;
                    logic [3:0] mask;
                    
                    quad_addr = ADDR_WIDTH'(qy >> 1) * ADDR_WIDTH'(WIDTH >> 1) + ADDR_WIDTH'(qx >> 1);
                    
                    // Generate mask for valid pixels
                    mask = {
                        ((qy + 1) <= maxy && (qx + 1) <= maxx && p3_e0 >= 0 && p3_e1 >= 0 && p3_e2 >= 0),
                        ((qy + 1) <= maxy && qx <= maxx && p2_e0 >= 0 && p2_e1 >= 0 && p2_e2 >= 0),
                        (qy <= maxy && (qx + 1) <= maxx && p1_e0 >= 0 && p1_e1 >= 0 && p1_e2 >= 0),
                        (qy <= maxy && qx <= maxx && p0_e0 >= 0 && p0_e1 >= 0 && p0_e2 >= 0)
                    };
                    
                    // Pack 4 pixels
                    fb_data <= {color_565, color_565, color_565, color_565};
                    fb_addr <= quad_addr;
                    fb_mask <= mask;
                    fb_we <= |mask;  // Write if any pixel is valid

                    // Move to next quad
                    if (qx + 2 > maxx) begin
                        qx <= minx;
                        qy <= qy + 2;
                        e0_row <= e0_row + (e0_dy << 1);
                        e1_row <= e1_row + (e1_dy << 1);
                        e2_row <= e2_row + (e2_dy << 1);
                        state <= INIT_QUAD;
                    end else begin
                        qx <= qx + 2;
                        e0_col <= e0_col + (e0_dx << 1);
                        e1_col <= e1_col + (e1_dx << 1);
                        e2_col <= e2_col + (e2_dx << 1);
                        state <= RASTER_QUAD;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
