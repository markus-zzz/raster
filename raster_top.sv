module raster_top #(
    parameter WIDTH = 320,
    parameter HEIGHT = 200,
    parameter TILE_W = 64,
    parameter TILE_H = 64,
    parameter SUBPIXEL = 4,
    parameter ADDR_WIDTH = $clog2((TILE_W/2) * (TILE_H/2))
) (
    input  logic clk,
    input  logic rst,
    // Tile clear
    input  logic clear,
    output logic clear_done,
    // Tile offset
    input  logic [$clog2(WIDTH)-1:0] tile_x,
    input  logic [$clog2(HEIGHT)-1:0] tile_y,
    // Triangle input (sub-pixel fixed-point, screen-space)
    input  logic start,
    input  logic [$clog2(WIDTH)+SUBPIXEL-1:0] v0_x, v1_x, v2_x,
    input  logic [$clog2(HEIGHT)+SUBPIXEL-1:0] v0_y, v1_y, v2_y,
    input  logic signed [15:0] iz_init, iz_dx, iz_dy,
    input  logic [23:0] color,
    output logic done,
    // Framebuffer read port (tile-local)
    input  logic [ADDR_WIDTH+1:0] fb_rd_pixel_addr,
    output logic [15:0] fb_rd_data
);

    // --- Clear logic ---
    logic clearing;
    logic [ADDR_WIDTH-1:0] clear_addr;

    always_ff @(posedge clk) begin
        if (rst) begin
            clearing <= 0;
            clear_done <= 0;
        end else if (clear && !clearing) begin
            clearing <= 1;
            clear_addr <= 0;
            clear_done <= 0;
        end else if (clearing) begin
            if (clear_addr == ADDR_WIDTH'((TILE_W/2)*(TILE_H/2) - 1)) begin
                clearing <= 0;
                clear_done <= 1;
            end else begin
                clear_addr <= clear_addr + 1;
            end
        end else begin
            clear_done <= 0;
        end
    end

    // --- Rasterizer ---
    logic [ADDR_WIDTH-1:0]  rast_fb_addr;
    logic [63:0]            rast_fb_data;
    logic [3:0]             rast_fb_mask;
    logic [ADDR_WIDTH-1:0]  rast_zb_wr_addr;
    logic [15:0]            rast_zb_wr_data [4];
    logic [3:0]             rast_zb_wr_mask;
    logic [ADDR_WIDTH-1:0]  zb_rd_addr;
    logic [15:0]            zb_rd_data [4];

    logic [15:0]            fb_rd_data_bank[4];

    rasterizer #(
        .RES_W(WIDTH),
        .RES_H(HEIGHT),
        .TILE_W(TILE_W),
        .TILE_H(TILE_H),
        .SUBPIXEL(SUBPIXEL),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) rast (
        .clk(clk),
        .rst(rst),
        .tile_x(tile_x),
        .tile_y(tile_y),
        .start(start),
        .v0_x(v0_x), .v0_y(v0_y),
        .v1_x(v1_x), .v1_y(v1_y),
        .v2_x(v2_x), .v2_y(v2_y),
        .iz_init(iz_init), .iz_dx(iz_dx), .iz_dy(iz_dy),
        .color(color),
        .done(done),
        .fb_addr(rast_fb_addr),
        .fb_data(rast_fb_data),
        .fb_mask(rast_fb_mask),
        .zb_rd_addr(zb_rd_addr),
        .zb_rd_data(zb_rd_data),
        .zb_wr_addr(rast_zb_wr_addr),
        .zb_wr_data(rast_zb_wr_data),
        .zb_wr_mask(rast_zb_wr_mask)
    );

    // Mux BRAM write ports: clear vs rasterizer
    logic [ADDR_WIDTH-1:0] fb_wr_addr;
    logic [3:0]            fb_wr_mask;
    logic [15:0]           fb_wr_data;
    logic [ADDR_WIDTH-1:0] zb_wr_addr;
    logic [3:0]            zb_wr_mask;
    logic [15:0]           zb_wr_data;

    assign fb_wr_addr = clearing ? clear_addr : rast_fb_addr;
    assign fb_wr_mask = clearing ? 4'b1111 : rast_fb_mask;
    assign fb_wr_data = clearing ? 16'h0 : 16'h0; // unused, per-bank below
    assign zb_wr_addr = clearing ? clear_addr : rast_zb_wr_addr;
    assign zb_wr_mask = clearing ? 4'b1111 : rast_zb_wr_mask;

    // Color framebuffer: 4 BRAM banks (tile-sized)
    genvar i;
    generate
        for (i = 0; i < 4; i++) begin : fb_banks
            dpram #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(16),
                .DEPTH((TILE_W/2) * (TILE_H/2))
            ) fb (
                .clk(clk),
                .wr_en(fb_wr_mask[i]),
                .wr_addr(fb_wr_addr),
                .wr_data(clearing ? 16'h0 : rast_fb_data[i*16 +: 16]),
                .rd_addr(fb_rd_pixel_addr[ADDR_WIDTH+1:2]),
                .rd_data(fb_rd_data_bank[i])
            );
        end
    endgenerate

    // Z-buffer: 4 BRAM banks (tile-sized)
    generate
        for (i = 0; i < 4; i++) begin : zb_banks
            dpram #(
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(16),
                .DEPTH((TILE_W/2) * (TILE_H/2))
            ) zb (
                .clk(clk),
                .wr_en(zb_wr_mask[i]),
                .wr_addr(zb_wr_addr),
                .wr_data(clearing ? 16'h0 : rast_zb_wr_data[i]),
                .rd_addr(zb_rd_addr),
                .rd_data(zb_rd_data[i])
            );
        end
    endgenerate

    assign fb_rd_data = fb_rd_data_bank[fb_rd_pixel_addr[1:0]];

endmodule
