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
    
    logic [15:0]            fb_rd_data0, fb_rd_data1, fb_rd_data2, fb_rd_data3;

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
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH((WIDTH/2) * (HEIGHT/2))
    ) fb0 (
        .clk(clk),
        .wr_en(fb_we & fb_wr_mask[0]),
        .wr_addr(fb_wr_addr),
        .wr_data(fb_wr_data[15:0]),
        .rd_addr(fb_rd_pixel_addr[15:2]),
        .rd_data(fb_rd_data0)
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH((WIDTH/2) * (HEIGHT/2))
    ) fb1 (
        .clk(clk),
        .wr_en(fb_we & fb_wr_mask[1]),
        .wr_addr(fb_wr_addr),
        .wr_data(fb_wr_data[31:16]),
        .rd_addr(fb_rd_pixel_addr[15:2]),
        .rd_data(fb_rd_data1)
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH((WIDTH/2) * (HEIGHT/2))
    ) fb2 (
        .clk(clk),
        .wr_en(fb_we & fb_wr_mask[2]),
        .wr_addr(fb_wr_addr),
        .wr_data(fb_wr_data[47:32]),
        .rd_addr(fb_rd_pixel_addr[15:2]),
        .rd_data(fb_rd_data2)
    );
    
    dpram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(16),
        .DEPTH((WIDTH/2) * (HEIGHT/2))
    ) fb3 (
        .clk(clk),
        .wr_en(fb_we & fb_wr_mask[3]),
        .wr_addr(fb_wr_addr),
        .wr_data(fb_wr_data[63:48]),
        .rd_addr(fb_rd_pixel_addr[15:2]),
        .rd_data(fb_rd_data3)
    );
    
    // Mux out the correct pixel from the quad
    always_comb begin
        case (fb_rd_pixel_addr[1:0])
            2'b00: fb_rd_data = fb_rd_data0;
            2'b01: fb_rd_data = fb_rd_data1;
            2'b10: fb_rd_data = fb_rd_data2;
            2'b11: fb_rd_data = fb_rd_data3;
        endcase
    end

endmodule
