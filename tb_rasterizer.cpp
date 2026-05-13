#include <verilated.h>
#include "Vraster_top.h"
#include "Vraster_top_raster_top.h"
#include "Vraster_top_dpram__Ae_DB3e80.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>

static const int W = 320, H = 200;
static uint32_t fb_hw[H][W];
static uint32_t fb_ref[H][W];
static uint16_t zb_ref[H][W];

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
    // Convert to sub-pixel (4 frac bits, pixel center)
    const int SP = 16; // 1 << SUBPIXEL
    int sv0x = v0.x * SP + SP/2, sv0y = v0.y * SP + SP/2;
    int sv1x = v1.x * SP + SP/2, sv1y = v1.y * SP + SP/2;
    int sv2x = v2.x * SP + SP/2, sv2y = v2.y * SP + SP/2;

    int minx = std::max(0,   std::min({v0.x, v1.x, v2.x}));
    int miny = std::max(0,   std::min({v0.y, v1.y, v2.y}));
    int maxx = std::min(W-1, std::max({v0.x, v1.x, v2.x}));
    int maxy = std::min(H-1, std::max({v0.y, v1.y, v2.y}));

    minx &= ~1; miny &= ~1;

    // Edge function at pixel center of (minx, miny)
    // Using sub-pixel vertex coords and sub-pixel sample point
    int pcx = minx * SP + SP/2, pcy = miny * SP + SP/2;

    // Edge functions in sub-pixel space
    auto edge_sp = [](int v0x, int v0y, int v1x, int v1y, int px, int py) -> int64_t {
        return (int64_t)(v1x - v0x) * (py - v0y) - (int64_t)(v1y - v0y) * (px - v0x);
    };

    int64_t e0_init = edge_sp(sv0x, sv0y, sv1x, sv1y, pcx, pcy);
    int64_t e1_init = edge_sp(sv1x, sv1y, sv2x, sv2y, pcx, pcy);
    int64_t e2_init = edge_sp(sv2x, sv2y, sv0x, sv0y, pcx, pcy);

    // Per-pixel deltas (step by SP in sub-pixel space)
    int64_t e0_dx = -(int64_t)(sv1y - sv0y) * SP;
    int64_t e0_dy =  (int64_t)(sv1x - sv0x) * SP;
    int64_t e1_dx = -(int64_t)(sv2y - sv1y) * SP;
    int64_t e1_dy =  (int64_t)(sv2x - sv1x) * SP;
    int64_t e2_dx = -(int64_t)(sv0y - sv2y) * SP;
    int64_t e2_dy =  (int64_t)(sv0x - sv2x) * SP;

    // Top-left rule
    bool tl0 = (e0_dx > 0) || (e0_dx == 0 && e0_dy < 0);
    bool tl1 = (e1_dx > 0) || (e1_dx == 0 && e1_dy < 0);
    bool tl2 = (e2_dx > 0) || (e2_dx == 0 && e2_dy < 0);

    auto inside = [](int64_t e, bool tl) { return e > 0 || (e == 0 && tl); };

    // 1/z plane
    int64_t iz_dx_val = e1_dx * iz0 + e2_dx * iz1 + e0_dx * iz2;
    int64_t iz_dy_val = e1_dy * iz0 + e2_dy * iz1 + e0_dy * iz2;
    int64_t iz_init   = e1_init * iz0 + e2_init * iz1 + e0_init * iz2;

    // Determine shift for iz truncation (match hardware IZ_FRAC)
    int CW_val = 0, CH_val = 0;
    for (int v = W-1; v > 0; v >>= 1) CW_val++;
    for (int v = H-1; v > 0; v >>= 1) CH_val++;
    int EW_val = CW_val + CH_val + 4 + 4 + 1; // VW + VH + 1
    int IZ_FRAC = 16 + EW_val;
    int shift = IZ_FRAC - 16;

    int64_t row0 = e0_init, row1 = e1_init, row2 = e2_init;
    int64_t iz_row = iz_init;

    for (int qy = miny; qy <= maxy; qy += 2) {
        int64_t col0 = row0, col1 = row1, col2 = row2;
        int64_t iz_col = iz_row;

        for (int qx = minx; qx <= maxx; qx += 2) {
            int64_t a0 = col0,          a1 = col1,          a2 = col2;
            int64_t b0 = col0 + e0_dx,  b1 = col1 + e1_dx,  b2 = col2 + e2_dx;
            int64_t c0 = col0 + e0_dy,  c1 = col1 + e1_dy,  c2 = col2 + e2_dy;
            int64_t d0 = b0   + e0_dy,  d1 = b1   + e1_dy,  d2 = b2   + e2_dy;

            int64_t iz_p[4];
            iz_p[0] = iz_col;
            iz_p[1] = iz_col + iz_dx_val;
            iz_p[2] = iz_col + iz_dy_val;
            iz_p[3] = iz_col + iz_dx_val + iz_dy_val;

            if (qy <= maxy && qx <= maxx && inside(a0,tl0) && inside(a1,tl1) && inside(a2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[0] >> shift);
                if (iz16 >= zb_ref[qy][qx]) {
                    fb_ref[qy][qx] = color;
                    zb_ref[qy][qx] = iz16;
                }
            }
            if (qy <= maxy && qx+1 <= maxx && inside(b0,tl0) && inside(b1,tl1) && inside(b2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[1] >> shift);
                if (iz16 >= zb_ref[qy][qx+1]) {
                    fb_ref[qy][qx+1] = color;
                    zb_ref[qy][qx+1] = iz16;
                }
            }
            if (qy+1 <= maxy && qx <= maxx && inside(c0,tl0) && inside(c1,tl1) && inside(c2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[2] >> shift);
                if (iz16 >= zb_ref[qy+1][qx]) {
                    fb_ref[qy+1][qx] = color;
                    zb_ref[qy+1][qx] = iz16;
                }
            }
            if (qy+1 <= maxy && qx+1 <= maxx && inside(d0,tl0) && inside(d1,tl1) && inside(d2,tl2)) {
                uint16_t iz16 = (uint16_t)(iz_p[3] >> shift);
                if (iz16 >= zb_ref[qy+1][qx+1]) {
                    fb_ref[qy+1][qx+1] = color;
                    zb_ref[qy+1][qx+1] = iz16;
                }
            }

            col0 += e0_dx * 2;
            col1 += e1_dx * 2;
            col2 += e2_dx * 2;
            iz_col += iz_dx_val * 2;
        }

        row0 += e0_dy * 2;
        row1 += e1_dy * 2;
        row2 += e2_dy * 2;
        iz_row += iz_dy_val * 2;
    }
}

void draw_triangle_hw(Vraster_top *dut, Vec2 v0, Vec2 v1, Vec2 v2,
                      uint16_t iz0, uint16_t iz1, uint16_t iz2,
                      uint32_t color) {
    // Convert pixel coords to sub-pixel (4 fractional bits, sample at pixel center)
    dut->v0_x = v0.x * 16 + 8; dut->v0_y = v0.y * 16 + 8;
    dut->v1_x = v1.x * 16 + 8; dut->v1_y = v1.y * 16 + 8;
    dut->v2_x = v2.x * 16 + 8; dut->v2_y = v2.y * 16 + 8;
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

void clear_hw(Vraster_top *dut) {
    dut->rst = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->rst = 0;

    const int NUM_QUADS = (W/2) * (H/2);
    auto *top = dut->raster_top;
    for (int i = 0; i < NUM_QUADS; i++) {
        top->zb_banks__BRA__0__KET____DOT__zb->mem[i] = 0;
        top->zb_banks__BRA__1__KET____DOT__zb->mem[i] = 0;
        top->zb_banks__BRA__2__KET____DOT__zb->mem[i] = 0;
        top->zb_banks__BRA__3__KET____DOT__zb->mem[i] = 0;
        top->fb_banks__BRA__0__KET____DOT__fb->mem[i] = 0;
        top->fb_banks__BRA__1__KET____DOT__fb->mem[i] = 0;
        top->fb_banks__BRA__2__KET____DOT__fb->mem[i] = 0;
        top->fb_banks__BRA__3__KET____DOT__fb->mem[i] = 0;
    }
}

void read_fb(Vraster_top *dut) {
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
}

int compare_fb() {
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
            if (fb_hw[y][x] != ref_888) errors++;
        }
    }
    return errors;
}

void write_ppm(const char *filename) {
    FILE *f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint32_t c = fb_hw[y][x];
            uint8_t rgb[3] = {(uint8_t)(c>>16), (uint8_t)(c>>8), (uint8_t)c};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vraster_top *dut = new Vraster_top;

    const int NUM_FRAMES = 180;
    const int cx = W/2, cy = H/2;
    const int radius = 70;
    int total_errors = 0;

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        memset(fb_hw, 0, sizeof(fb_hw));
        memset(fb_ref, 0, sizeof(fb_ref));
        memset(zb_ref, 0, sizeof(zb_ref));

        // Clear HW framebuffer and z-buffer
        clear_hw(dut);

        // Rotating square as two adjacent triangles
        double angle = frame * 2.0 * M_PI / (NUM_FRAMES * 8);
        double cs = cos(angle), sn = sin(angle);

        int px[4], py[4];
        double offsets[4][2] = {{-1,-1},{1,-1},{1,1},{-1,1}};
        for (int i = 0; i < 4; i++) {
            double ox = offsets[i][0] * radius;
            double oy = offsets[i][1] * radius;
            px[i] = cx + (int)(ox * cs - oy * sn);
            py[i] = cy + (int)(ox * sn + oy * cs);
            if (px[i] < 0) px[i] = 0;
            if (px[i] >= W) px[i] = W-1;
            if (py[i] < 0) py[i] = 0;
            if (py[i] >= H) py[i] = H-1;
        }

        // Two adjacent triangles sharing diagonal p0-p2
        Vec2 a0={px[0],py[0]}, a1={px[1],py[1]}, a2={px[2],py[2]};
        Vec2 b0={px[0],py[0]}, b1={px[2],py[2]}, b2={px[3],py[3]};

        draw_triangle_hw(dut, a0, a1, a2, 200, 200, 200, 0x00FFFF);
        draw_triangle_hw(dut, b0, b1, b2, 150, 150, 150, 0xFF00FF);

        draw_triangle_ref(a0, a1, a2, 200, 200, 200, 0x00FFFF);
        draw_triangle_ref(b0, b1, b2, 150, 150, 150, 0xFF00FF);

        read_fb(dut);
        int errors = compare_fb();
        total_errors += errors;

        printf("Frame %02d: %s", frame, errors == 0 ? "PASS" : "FAIL");
        if (errors) printf(" (%d mismatches)", errors);
        printf("\n");

        char filename[64];
        snprintf(filename, sizeof(filename), "frame_%02d.ppm", frame);
        write_ppm(filename);
    }

    printf("\nTotal: %s (%d errors across %d frames)\n",
           total_errors == 0 ? "PASS" : "FAIL", total_errors, NUM_FRAMES);

    delete dut;
    return total_errors ? 1 : 0;
}
