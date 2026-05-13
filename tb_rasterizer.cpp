#include <verilated.h>
#include "Vraster_top.h"
#include <cstdio>
#include <cstdint>
#include <cstring>

static const int W = 320, H = 240;
static uint32_t fb_hw[H][W];
static uint32_t fb_ref[H][W];

struct Vec2 { int x, y; };

static int edge_func(Vec2 v0, Vec2 v1, Vec2 p) {
    return (v1.x - v0.x) * (p.y - v0.y) - (v1.y - v0.y) * (p.x - v0.x);
}

struct EdgeStep {
    int val, dx, dy;
};

static EdgeStep make_edge(Vec2 v0, Vec2 v1, Vec2 origin) {
    return {
        edge_func(v0, v1, origin),
        -(v1.y - v0.y),
         (v1.x - v0.x),
    };
}

void draw_triangle_ref(Vec2 v0, Vec2 v1, Vec2 v2, uint32_t color) {
    int minx = std::max(0,   std::min({v0.x, v1.x, v2.x}));
    int miny = std::max(0,   std::min({v0.y, v1.y, v2.y}));
    int maxx = std::min(W-1, std::max({v0.x, v1.x, v2.x}));
    int maxy = std::min(H-1, std::max({v0.y, v1.y, v2.y}));

    minx &= ~1; miny &= ~1;

    Vec2 origin = {minx, miny};
    EdgeStep e0 = make_edge(v0, v1, origin);
    EdgeStep e1 = make_edge(v1, v2, origin);
    EdgeStep e2 = make_edge(v2, v0, origin);

    int row0 = e0.val, row1 = e1.val, row2 = e2.val;

    for (int qy = miny; qy <= maxy; qy += 2) {
        int col0 = row0, col1 = row1, col2 = row2;

        for (int qx = minx; qx <= maxx; qx += 2) {
            int a0 = col0,          a1 = col1,          a2 = col2;
            int b0 = col0 + e0.dx,  b1 = col1 + e1.dx,  b2 = col2 + e2.dx;
            int c0 = col0 + e0.dy,  c1 = col1 + e1.dy,  c2 = col2 + e2.dy;
            int d0 = b0   + e0.dy,  d1 = b1   + e1.dy,  d2 = b2   + e2.dy;

            if (qy   <= maxy && qx   <= maxx && a0>=0 && a1>=0 && a2>=0) fb_ref[qy  ][qx  ] = color;
            if (qy   <= maxy && qx+1 <= maxx && b0>=0 && b1>=0 && b2>=0) fb_ref[qy  ][qx+1] = color;
            if (qy+1 <= maxy && qx   <= maxx && c0>=0 && c1>=0 && c2>=0) fb_ref[qy+1][qx  ] = color;
            if (qy+1 <= maxy && qx+1 <= maxx && d0>=0 && d1>=0 && d2>=0) fb_ref[qy+1][qx+1] = color;

            col0 += e0.dx * 2;
            col1 += e1.dx * 2;
            col2 += e2.dx * 2;
        }

        row0 += e0.dy * 2;
        row1 += e1.dy * 2;
        row2 += e2.dy * 2;
    }
}

void draw_triangle_hw(Vraster_top *dut, Vec2 v0, Vec2 v1, Vec2 v2, uint32_t color) {
    dut->v0_x = v0.x; dut->v0_y = v0.y;
    dut->v1_x = v1.x; dut->v1_y = v1.y;
    dut->v2_x = v2.x; dut->v2_y = v2.y;
    dut->color = color;
    dut->start = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->start = 0;

    // Run until done
    int timeout = 100000;
    int writes = 0;
    while (!dut->done && timeout-- > 0) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
    }

    printf("  Total writes: %d\n", writes);
    if (timeout <= 0) {
        printf("ERROR: Hardware timeout\n");
    }
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vraster_top *dut = new Vraster_top;

    // Reset
    memset(fb_hw, 0, sizeof(fb_hw));
    memset(fb_ref, 0, sizeof(fb_ref));

    dut->rst = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->rst = 0;

    // Draw same triangles as C++ reference
    draw_triangle_hw(dut, {10, 10}, {200, 30}, {100, 180}, 0xFF0000);
    draw_triangle_hw(dut, {150, 50}, {310, 80}, {230, 190}, 0x00FF00);
    draw_triangle_hw(dut, {50, 100}, {160, 80}, {80, 190}, 0x0000FF);

    draw_triangle_ref({10, 10}, {200, 30}, {100, 180}, 0xFF0000);
    draw_triangle_ref({150, 50}, {310, 80}, {230, 190}, 0x00FF00);
    draw_triangle_ref({50, 100}, {160, 80}, {80, 190}, 0x0000FF);

    // Read back framebuffer and convert RGB565 to RGB888
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            // Convert pixel (x,y) to quad address and pixel index
            // Quad layout: [p0=00, p1=01, p2=10, p3=11]
            int quad_x = x / 2;
            int quad_y = y / 2;
            int pixel_idx = (y % 2) * 2 + (x % 2);
            dut->fb_rd_pixel_addr = (quad_y * (W/2) + quad_x) * 4 + pixel_idx;
            dut->clk = 0; dut->eval();
            dut->clk = 1; dut->eval();
            uint16_t rgb565 = dut->fb_rd_data;
            // Convert RGB565 to RGB888
            uint8_t r = (rgb565 >> 11) << 3;
            uint8_t g = ((rgb565 >> 5) & 0x3F) << 2;
            uint8_t b = (rgb565 & 0x1F) << 3;
            fb_hw[y][x] = (r << 16) | (g << 8) | b;
        }
    }

    // Compare (convert reference to RGB565 for fair comparison)
    int errors = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            // Convert reference to RGB565 and back
            uint32_t ref = fb_ref[y][x];
            uint8_t ref_r = (ref >> 16) & 0xFF;
            uint8_t ref_g = (ref >> 8) & 0xFF;
            uint8_t ref_b = ref & 0xFF;
            uint16_t ref_565 = ((ref_r >> 3) << 11) | ((ref_g >> 2) << 5) | (ref_b >> 3);
            uint8_t ref_r8 = (ref_565 >> 11) << 3;
            uint8_t ref_g8 = ((ref_565 >> 5) & 0x3F) << 2;
            uint8_t ref_b8 = (ref_565 & 0x1F) << 3;
            uint32_t ref_888 = (ref_r8 << 16) | (ref_g8 << 8) | ref_b8;
            
            if (fb_hw[y][x] != ref_888) {
                if (errors < 20) {
                    printf("MISMATCH at (%d,%d): hw=%06X ref_565=%06X\n", 
                           x, y, fb_hw[y][x], ref_888);
                }
                errors++;
            }
        }
    }

    if (errors == 0) {
        printf("PASS: Hardware matches reference (%d pixels)\n", W*H);
    } else {
        printf("FAIL: %d mismatches\n", errors);
    }

    // Write hardware output
    FILE *f = fopen("out_hw.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint32_t c = fb_hw[y][x];
            uint8_t rgb[3] = {(uint8_t)(c>>16), (uint8_t)(c>>8), (uint8_t)c};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);

    delete dut;
    return errors ? 1 : 0;
}
