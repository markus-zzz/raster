module raster_top #(
    parameter WIDTH = 320,
    parameter HEIGHT = 200,
    parameter ADDR_WIDTH = 14  // (WIDTH/2) * (HEIGHT/2) = 160*100 = 16000 quads
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
    // Framebuffer read port (pixel addressing)
    input  logic [15:0] fb_rd_pixel_addr,
    output logic [15:0] fb_rd_data
);

    logic                   fb_we;
    logic [ADDR_WIDTH-1:0]  fb_wr_addr;
    logic [63:0]            fb_wr_data;
    logic [3:0]             fb_wr_mask;
    
    logic [15:0]            fb_rd_data_bank[4];

    rasterizer #(
        .WIDTH(WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) rast (
        .clk(clk),
        .rst(rst),
        .start(start),
        .v0_x(v0_x), .v0_y(v0_y),
        .v1_x(v1_x), .v1_y(v1_y),
        .v2_x(v2_x), .v2_y(v2_y),
        .color(color),
        .done(done),
        .fb_we(fb_we),
        .fb_addr(fb_wr_addr),
        .fb_data(fb_wr_data),
        .fb_mask(fb_wr_mask)
    );

    // Instantiate 4 BRAM banks, one per pixel in quad
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : fb_banks
            dpram #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(16),
                .DEPTH((WIDTH/2) * (HEIGHT/2))
            ) fb (
                .clk(clk),
                .wr_en(fb_we & fb_wr_mask[i]),
                .wr_addr(fb_wr_addr),
                .wr_data(fb_wr_data[i*16 +: 16]),
                .rd_addr(fb_rd_pixel_addr[15:2]),
                .rd_data(fb_rd_data_bank[i])
            );
        end
    endgenerate
    
    // Mux out the correct pixel from the quad
    assign fb_rd_data = fb_rd_data_bank[fb_rd_pixel_addr[1:0]];

endmodule
