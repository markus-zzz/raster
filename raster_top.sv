module raster_top #(
    parameter WIDTH = 320,
    parameter HEIGHT = 200,
    parameter ADDR_WIDTH = $clog2((WIDTH/2) * (HEIGHT/2))
) (
    input  logic clk,
    input  logic rst,
    // Triangle input
    input  logic start,
    input  logic [$clog2(WIDTH)-1:0] v0_x, v1_x, v2_x,
    input  logic [$clog2(HEIGHT)-1:0] v0_y, v1_y, v2_y,
    input  logic [15:0] v0_iz, v1_iz, v2_iz,
    input  logic [23:0] color,
    output logic done,
    // Framebuffer read port (pixel addressing)
    input  logic [ADDR_WIDTH+1:0] fb_rd_pixel_addr,
    output logic [15:0] fb_rd_data
);

    logic [ADDR_WIDTH-1:0]  fb_wr_addr;
    logic [63:0]            fb_wr_data;
    logic [3:0]             fb_wr_mask;

    logic [ADDR_WIDTH-1:0]  zb_rd_addr;
    logic [15:0]            zb_rd_data [4];
    logic [ADDR_WIDTH-1:0]  zb_wr_addr;
    logic [15:0]            zb_wr_data [4];
    logic [3:0]             zb_wr_mask;

    logic [15:0]            fb_rd_data_bank[4];

    rasterizer #(
        .RWIDTH(WIDTH),
        .RHEIGHT(HEIGHT),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) rast (
        .clk(clk),
        .rst(rst),
        .start(start),
        .v0_x(v0_x), .v0_y(v0_y),
        .v1_x(v1_x), .v1_y(v1_y),
        .v2_x(v2_x), .v2_y(v2_y),
        .v0_iz(v0_iz), .v1_iz(v1_iz), .v2_iz(v2_iz),
        .color(color),
        .done(done),
        .fb_addr(fb_wr_addr),
        .fb_data(fb_wr_data),
        .fb_mask(fb_wr_mask),
        .zb_rd_addr(zb_rd_addr),
        .zb_rd_data(zb_rd_data),
        .zb_wr_addr(zb_wr_addr),
        .zb_wr_data(zb_wr_data),
        .zb_wr_mask(zb_wr_mask)
    );

    // Color framebuffer: 4 BRAM banks
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : fb_banks
            dpram #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(16),
                .DEPTH((WIDTH/2) * (HEIGHT/2))
            ) fb (
                .clk(clk),
                .wr_en(fb_wr_mask[i]),
                .wr_addr(fb_wr_addr),
                .wr_data(fb_wr_data[i*16 +: 16]),
                .rd_addr(fb_rd_pixel_addr[ADDR_WIDTH+1:2]),
                .rd_data(fb_rd_data_bank[i])
            );
        end
    endgenerate

    // Z-buffer: 4 BRAM banks
    generate
        for (i = 0; i < 4; i++) begin : zb_banks
            dpram #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(16),
                .DEPTH((WIDTH/2) * (HEIGHT/2))
            ) zb (
                .clk(clk),
                .wr_en(zb_wr_mask[i]),
                .wr_addr(zb_wr_addr),
                .wr_data(zb_wr_data[i]),
                .rd_addr(zb_rd_addr),
                .rd_data(zb_rd_data[i])
            );
        end
    endgenerate

    // Mux out the correct pixel from the quad
    assign fb_rd_data = fb_rd_data_bank[fb_rd_pixel_addr[1:0]];

endmodule
