module raster_top #(
    parameter WIDTH = 320,
    parameter HEIGHT = 200,
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
    // Framebuffer read port
    input  logic [ADDR_WIDTH-1:0] fb_rd_addr,
    output logic [15:0] fb_rd_data
);

    logic                   fb_we0, fb_we1, fb_we2, fb_we3;
    logic [ADDR_WIDTH-1:0]  fb_wr_addr0, fb_wr_addr1, fb_wr_addr2, fb_wr_addr3;
    logic [15:0]            fb_wr_data0, fb_wr_data1, fb_wr_data2, fb_wr_data3;

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
        .fb_we0(fb_we0), .fb_we1(fb_we1), .fb_we2(fb_we2), .fb_we3(fb_we3),
        .fb_addr0(fb_wr_addr0), .fb_addr1(fb_wr_addr1), .fb_addr2(fb_wr_addr2), .fb_addr3(fb_wr_addr3),
        .fb_data0(fb_wr_data0), .fb_data1(fb_wr_data1), .fb_data2(fb_wr_data2), .fb_data3(fb_wr_data3)
    );

    // Instantiate 4 BRAM banks for parallel writes
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH(WIDTH * HEIGHT)
    ) fb0 (
        .clk(clk),
        .wr_en(fb_we0),
        .wr_addr(fb_wr_addr0),
        .wr_data(fb_wr_data0),
        .rd_addr(fb_rd_addr),
        .rd_data(fb_rd_data)
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH(WIDTH * HEIGHT)
    ) fb1 (
        .clk(clk),
        .wr_en(fb_we1),
        .wr_addr(fb_wr_addr1),
        .wr_data(fb_wr_data1),
        .rd_addr(fb_rd_addr),
        .rd_data()
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH(WIDTH * HEIGHT)
    ) fb2 (
        .clk(clk),
        .wr_en(fb_we2),
        .wr_addr(fb_wr_addr2),
        .wr_data(fb_wr_data2),
        .rd_addr(fb_rd_addr),
        .rd_data()
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH(WIDTH * HEIGHT)
    ) fb3 (
        .clk(clk),
        .wr_en(fb_we3),
        .wr_addr(fb_wr_addr3),
        .wr_data(fb_wr_data3),
        .rd_addr(fb_rd_addr),
        .rd_data()
    );

endmodule
