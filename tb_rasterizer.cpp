#include <verilated.h>
#include "Vraster_top.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <algorithm>

static const int W = 320, H = 200;
static uint32_t fb_hw[H][W];
static uint32_t fb_ref[H][W];
static uint16_t zb_ref[H][W]; // reference z-buffer (stores 1/z, higher = closer)

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

void draw_triangle_ref(Vec2 v0, Vec2 v1, Vec2 v2,
                       uint16_t iz0, uint16_t iz1, uint16_t iz2,
                       uint32_t color) {
    int minx = std::max(0,   std::min({v0.x, v1.x, v2.x}));
    int miny = std::max(0,   std::min({v0.y, v1.y, v2.y}));
    int maxx = std::min(W-1, std::max({v0.x, v1.x, v2.x}));
    int maxy = std::min(H-1, std::max({v0.y, v1.y, v2.y}));

    minx &= ~1; miny &= ~1;

    Vec2 origin = {minx, miny};
    EdgeStep e0 = make_edge(v0, v1, origin);
    EdgeStep e1 = make_edge(v1, v2, origin);
    EdgeStep e2 = make_edge(v2, v0, origin);

    // Top-left rule: left edge (dx > 0) or top edge (dx == 0 && dy < 0)
    bool tl0 = (e0.dx > 0) || (e0.dx == 0 && e0.dy < 0);
    bool tl1 = (e1.dx > 0) || (e1.dx == 0 && e1.dy < 0);
    bool tl2 = (e2.dx > 0) || (e2.dx == 0 && e2.dy < 0);

    auto inside = [](int e, bool tl) { return e > 0 || (e == 0 && tl); };

    // 1/z plane: iz(p) = e1(p)*iz0 + e2(p)*iz1 + e0(p)*iz2 (unnormalized)
    // iz_dx = e1.dx*iz0 + e2.dx*iz1 + e0.dx*iz2
    // iz_dy = e1.dy*iz0 + e2.dy*iz1 + e0.dy*iz2
    int64_t iz_dx_val = (int64_t)e1.dx * iz0 + (int64_t)e2.dx * iz1 + (int64_t)e0.dx * iz2;
    int64_t iz_dy_val = (int64_t)e1.dy * iz0 + (int64_t)e2.dy * iz1 + (int64_t)e0.dy * iz2;
    int64_t iz_init   = (int64_t)e1.val * iz0 + (int64_t)e2.val * iz1 + (int64_t)e0.val * iz2;

    int row0 = e0.val, row1 = e1.val, row2 = e2.val;
    int64_t iz_row = iz_init;

    // Determine IZ_FRAC shift to match hardware (EW = clog2(W) + clog2(H) + 1)
    // Hardware takes upper 16 bits of IZ_FRAC-bit value
    int CW_val = 0, CH_val = 0;
    for (int v = W-1; v > 0; v >>= 1) CW_val++;
    for (int v = H-1; v > 0; v >>= 1) CH_val++;
    int EW_val = CW_val + CH_val + 1;
    int IZ_FRAC = 16 + EW_val;
    int shift = IZ_FRAC - 16;

    for (int qy = miny; qy <= maxy; qy += 2) {
        int col0 = row0, col1 = row1, col2 = row2;
        int64_t iz_col = iz_row;

        for (int qx = minx; qx <= maxx; qx += 2) {
            int a0 = col0,          a1 = col1,          a2 = col2;
            int b0 = col0 + e0.dx,  b1 = col1 + e1.dx,  b2 = col2 + e2.dx;
            int c0 = col0 + e0.dy,  c1 = col1 + e1.dy,  c2 = col2 + e2.dy;
            int d0 = b0   + e0.dy,  d1 = b1   + e1.dy,  d2 = b2   + e2.dy;

            int64_t iz_p[4];
            iz_p[0] = iz_col;
            iz_p[1] = iz_col + iz_dx_val;
            iz_p[2] = iz_col + iz_dy_val;
            iz_p[3] = iz_col + iz_dx_val + iz_dy_val;

            // Pixel 0 (qx, qy)
            if (qy <= maxy && qx <= maxx && inside(a0,tl0) && inside(a1,tl1) && inside(a2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[0] >> shift);
                if (iz16 >= zb_ref[qy][qx]) {
                    fb_ref[qy][qx] = color;
                    zb_ref[qy][qx] = iz16;
                }
            }
            // Pixel 1 (qx+1, qy)
            if (qy <= maxy && qx+1 <= maxx && inside(b0,tl0) && inside(b1,tl1) && inside(b2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[1] >> shift);
                if (iz16 >= zb_ref[qy][qx+1]) {
                    fb_ref[qy][qx+1] = color;
                    zb_ref[qy][qx+1] = iz16;
                }
            }
            // Pixel 2 (qx, qy+1)
            if (qy+1 <= maxy && qx <= maxx && inside(c0,tl0) && inside(c1,tl1) && inside(c2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[2] >> shift);
                if (iz16 >= zb_ref[qy+1][qx]) {
                    fb_ref[qy+1][qx] = color;
                    zb_ref[qy+1][qx] = iz16;
                }
            }
            // Pixel 3 (qx+1, qy+1)
            if (qy+1 <= maxy && qx+1 <= maxx && inside(d0,tl0) && inside(d1,tl1) && inside(d2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[3] >> shift);
                if (iz16 >= zb_ref[qy+1][qx+1]) {
                    fb_ref[qy+1][qx+1] = color;
                    zb_ref[qy+1][qx+1] = iz16;
                }
            }

            col0 += e0.dx * 2;
            col1 += e1.dx * 2;
            col2 += e2.dx * 2;
            iz_col += iz_dx_val * 2;
        }

        row0 += e0.dy * 2;
        row1 += e1.dy * 2;
        row2 += e2.dy * 2;
        iz_row += iz_dy_val * 2;
    }
}

void draw_triangle_hw(Vraster_top *dut, Vec2 v0, Vec2 v1, Vec2 v2,
                      uint16_t iz0, uint16_t iz1, uint16_t iz2,
                      uint32_t color) {
    dut->v0_x = v0.x; dut->v0_y = v0.y;
    dut->v1_x = v1.x; dut->v1_y = v1.y;
    dut->v2_x = v2.x; dut->v2_y = v2.y;
    dut->v0_iz = iz0; dut->v1_iz = iz1; dut->v2_iz = iz2;
    dut->color = color;
    dut->start = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->start = 0;

    int timeout = 200000;
    while (!dut->done && timeout-- > 0) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
    }

    if (timeout <= 0) {
        printf("ERROR: Hardware timeout\n");
    }
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vraster_top *dut = new Vraster_top;

    memset(fb_hw, 0, sizeof(fb_hw));
    memset(fb_ref, 0, sizeof(fb_ref));
    memset(zb_ref, 0, sizeof(zb_ref));

    // Reset
    dut->rst = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->rst = 0;

    // Draw overlapping triangles with varying depths to exercise z-buffer
    // Triangle 1: Large red background triangle, far (flat iz=100)
    draw_triangle_hw(dut, {20, 20}, {300, 40}, {160, 220}, 100, 100, 100, 0xFF0000);
    // Triangle 2: Green triangle, partially in front (iz varies 50-400, tilted)
    draw_triangle_hw(dut, {60, 60}, {250, 100}, {100, 200}, 50, 400, 200, 0x00FF00);
    // Triangle 3: Blue triangle, partially behind red, partially in front of green
    draw_triangle_hw(dut, {140, 30}, {310, 150}, {180, 210}, 150, 80, 300, 0x0000FF);
    // Triangle 4: Small white triangle, very close (high iz), fully overlapping others
    draw_triangle_hw(dut, {120, 100}, {200, 90}, {160, 160}, 500, 500, 500, 0xFFFFFF);
    // Triangle 5: Yellow triangle drawn last but far away — should be mostly occluded
    draw_triangle_hw(dut, {40, 80}, {220, 60}, {130, 190}, 30, 30, 30, 0xFFFF00);

    draw_triangle_ref({20, 20}, {300, 40}, {160, 220}, 100, 100, 100, 0xFF0000);
    draw_triangle_ref({60, 60}, {250, 100}, {100, 200}, 50, 400, 200, 0x00FF00);
    draw_triangle_ref({140, 30}, {310, 150}, {180, 210}, 150, 80, 300, 0x0000FF);
    draw_triangle_ref({120, 100}, {200, 90}, {160, 160}, 500, 500, 500, 0xFFFFFF);
    draw_triangle_ref({40, 80}, {220, 60}, {130, 190}, 30, 30, 30, 0xFFFF00);

    // Read back framebuffer
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int quad_x = x / 2;
            int quad_y = y / 2;
            int pixel_idx = (y % 2) * 2 + (x % 2);
            dut->fb_rd_pixel_addr = (quad_y * (W/2) + quad_x) * 4 + pixel_idx;
            dut->clk = 0; dut->eval();
            dut->clk = 1; dut->eval();
            uint16_t rgb565 = dut->fb_rd_data;
            uint8_t r = (rgb565 >> 11) << 3;
            uint8_t g = ((rgb565 >> 5) & 0x3F) << 2;
            uint8_t b = (rgb565 & 0x1F) << 3;
            fb_hw[y][x] = (r << 16) | (g << 8) | b;
        }
    }

    // Compare
    int errors = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
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
                    printf("MISMATCH at (%d,%d): hw=%06X ref=%06X\n",
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
