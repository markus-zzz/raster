#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vsystem_top.h"
#include "Vsystem_top_system_top.h"
#include "Vsystem_top_gpu_top.h"
#ifdef SDRAM_INIT_MODE
// Non-default SDRAM_INIT_FILE parameter gives the parameterized modules an
// "__Iz1" name suffix.
#include "Vsystem_top_sdram_model__Iz1.h"
#include "Vsystem_top_dpram__DB20000_Iz1.h"
#else
#include "Vsystem_top_sdram_model.h"
#include "Vsystem_top_dpram__DB20000.h"
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <optional>
#include <vector>


static const int W = 320, H = 200;
static const int TW = 64, TH = 64;
static const int NTX = (W + TW - 1) / TW;
static const int NTY = (H + TH - 1) / TH;
static const int NUM_TILES = NTX * NTY;
static const int SP = 16;

// SDRAM memory map (halfword units). Total used: ~157 KB, fits in 256 KB.
//   TRI_BASE     0x00000  (1024 tris max * 16 hw)
//   BIN_BASE     0x04000  (per-tile counts)
//   BINLIST_BASE 0x05000  (NUM_TILES * MAX_FACES_PER_TILE = 0x5000 hw)
//   FB_BASE      0x0A000  (FRAME_W * FRAME_H = 0xFA00 hw)
static const int TRI_BASE     = 0x00000;
static const int BIN_BASE     = 0x04000;
static const int BINLIST_BASE = 0x05000;
static const int FB_BASE      = 0x0A000;

// Each tile gets a fixed-address bucket of MAX_FACES_PER_TILE entries in the
// binlist region. Tile T's bucket starts at BINLIST_BASE + T*MAX_FACES_PER_TILE
// (halfword units). A power of 2 makes the indexing trivial in HW.
static const int MAX_FACES_PER_TILE = 1024;

// Total SDRAM model storage in 16-bit words (must match sdram_model ADDR_BITS).
static const int SDRAM_WORDS = 1 << 17;  // 128 K words = 256 KB

static uint32_t framebuffer[H][W];

struct Face {
    int v[3];          // vertex indices
    glm::vec3 color;   // per-face hue (precomputed from object-space normal)
};

struct Tri2D {
    glm::ivec2 p[3];          // screen-space vertices (pixel units, pre-subpixel)
    int16_t iz_at_00;         // 1/z evaluated at screen origin, Q11.5
    int16_t diz_dx;           // 1/z gradient per pixel, X, Q11.5
    int16_t diz_dy;           // 1/z gradient per pixel, Y, Q11.5
    uint32_t color;           // packed 0xRRGGBB
    int bbminx, bbminy, bbmaxx, bbmaxy;
};

std::vector<glm::vec3> vertices;
std::vector<Face> faces;

void load_obj(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { printf("ERROR: cannot open %s\n", path); return; }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            glm::vec3 v;
            sscanf(line + 2, "%f %f %f", &v.x, &v.y, &v.z);
            vertices.push_back(v);
        } else if (line[0] == 'f' && line[1] == ' ') {
            int vi[4] = {0}, count = 0;
            char *p = line + 2;
            while (*p && *p != '\n' && count < 4) {
                if (*p >= '0' && *p <= '9') {
                    vi[count++] = atoi(p) - 1;
                    while (*p && *p != ' ' && *p != '\n') p++;
                } else {
                    p++;
                }
            }
            if (count >= 3) {
                faces.push_back({{vi[0], vi[1], vi[2]}, glm::vec3(0)});
                if (count == 4) faces.push_back({{vi[0], vi[2], vi[3]}, glm::vec3(0)});
            }
        }
    }
    fclose(f);

    // Precompute per-face hue from the object-space normal. This way the
    // runtime pipeline only needs the view-space normal (used for cull + light).
    for (auto &face : faces) {
        const glm::vec3 &v0 = vertices[face.v[0]];
        const glm::vec3 &v1 = vertices[face.v[1]];
        const glm::vec3 &v2 = vertices[face.v[2]];
        glm::vec3 n_obj = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        face.color = glm::vec3(0.5f) + 0.5f * n_obj;
    }

    printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

// Build the model-to-view matrix. Equivalent to R * diag(1,-1,1):
// y-flip first (so screen y increases downward), then rotation.
glm::mat3 build_view_rot(float angle_y, float angle_x) {
    glm::mat4 m(1.0f);
    m = glm::rotate(m, angle_x, glm::vec3(1, 0, 0));
    m = glm::rotate(m, angle_y, glm::vec3(0, 1, 0));
    m = glm::scale(m, glm::vec3(1, -1, 1));
    return glm::mat3(m);
}

// Light direction in view space
static const glm::vec3 LIGHT_DIR = glm::normalize(glm::vec3(0.2f, 0.3f, 1.0f));

// Process one object-space face all the way to a screen-space Tri2D, or return
// std::nullopt if the face is back-facing. This mirrors the per-triangle dataflow
// a hardware geometry pipeline would use.
std::optional<Tri2D> process_face(const Face &face, const glm::mat3 &R) {
    const float scale = 80.0f;

    // Step 1: object-space -> view-space (rotation + Y-flip baked into R)
    glm::vec3 vv[3];
    for (int i = 0; i < 3; i++) {
        vv[i] = R * vertices[face.v[i]];
    }

    // Step 2: view-space face normal
    glm::vec3 n = glm::normalize(glm::cross(vv[1] - vv[0], vv[2] - vv[0]));

    // Step 3: back-face cull (camera looks toward -Z; front faces have n.z < 0)
    if (n.z >= 0.0f) return std::nullopt;

    // Step 4: lighting from the view-space normal. Per-face hue is precomputed
    // at load time from the object-space normal.
    float diffuse = std::max(0.0f, glm::dot(-n, LIGHT_DIR));
    float intensity = 0.2f + 0.8f * diffuse;
    glm::vec3 col = glm::clamp(intensity * face.color, 0.0f, 1.0f);
    uint8_t r = (uint8_t)(col.r * 255);
    uint8_t g = (uint8_t)(col.g * 255);
    uint8_t b = (uint8_t)(col.b * 255);

    // Step 5: project view-space vertices to screen-space + 1/z
    glm::ivec2 sp[3];
    float iz[3];
    for (int i = 0; i < 3; i++) {
        sp[i] = glm::ivec2((int)(vv[i].x * scale + W / 2),
                           (int)(vv[i].y * scale + H / 2));
        iz[i] = std::max(1.0f, 275.0f + vv[i].z * 225.0f);
    }

    // Step 6: assemble Tri2D. Swap winding 1<->2 to undo the orientation
    // inversion caused by the Y-flip in build_view_rot (OBJ is CCW;
    // rasterizer expects CCW edge functions).
    Tri2D t;
    t.p[0] = sp[0]; t.p[1] = sp[2]; t.p[2] = sp[1];
    float izp[3] = { iz[0], iz[2], iz[1] };
    t.color = (r << 16) | (g << 8) | b;
    t.bbminx = std::min({t.p[0].x, t.p[1].x, t.p[2].x});
    t.bbminy = std::min({t.p[0].y, t.p[1].y, t.p[2].y});
    t.bbmaxx = std::max({t.p[0].x, t.p[1].x, t.p[2].x});
    t.bbmaxy = std::max({t.p[0].y, t.p[1].y, t.p[2].y});

    // Step 7: 1/z plane setup. Solve for diz_dx, diz_dy from the two edges
    // and extrapolate to screen origin. Result is fixed-point Q11.5.
    glm::vec2 e1 = glm::vec2(t.p[1] - t.p[0]);
    glm::vec2 e2 = glm::vec2(t.p[2] - t.p[0]);
    float area = e1.x * e2.y - e1.y * e2.x;
    float diz_dx = 0, diz_dy = 0;
    if (std::abs(area) > 0.001f) {
        float diz1 = izp[1] - izp[0];
        float diz2 = izp[2] - izp[0];
        diz_dx = (diz1 * e2.y - diz2 * e1.y) / area;
        diz_dy = (diz2 * e1.x - diz1 * e2.x) / area;
    }
    float iz_at_00 = izp[0] - diz_dx * t.p[0].x - diz_dy * t.p[0].y;

    t.iz_at_00 = (int16_t)roundf(iz_at_00 * 32.0f);
    t.diz_dx   = (int16_t)roundf(diz_dx   * 32.0f);
    t.diz_dy   = (int16_t)roundf(diz_dy   * 32.0f);

    return t;
}

// Serialize a fully-prepared Tri2D into the SDRAM tri-record region. All
// geometry (sub-pixel fixup, 1/z plane setup) is done in process_face — this
// is pure DMA-style packing.
void write_tri_record(auto &mem, int tri_id, const Tri2D &t) {
    int base = TRI_BASE + tri_id * 16;
    mem[base + 0] = (uint16_t)(t.p[0].x * SP + SP/2);
    mem[base + 1] = (uint16_t)(t.p[0].y * SP + SP/2);
    mem[base + 2] = (uint16_t)(t.p[1].x * SP + SP/2);
    mem[base + 3] = (uint16_t)(t.p[1].y * SP + SP/2);
    mem[base + 4] = (uint16_t)(t.p[2].x * SP + SP/2);
    mem[base + 5] = (uint16_t)(t.p[2].y * SP + SP/2);
    mem[base + 6]  = (uint16_t)t.iz_at_00;
    mem[base + 7]  = (uint16_t)t.diz_dx;
    mem[base + 8]  = (uint16_t)t.diz_dy;
    mem[base + 9]  = (uint16_t)(t.color & 0xFFFF);
    mem[base + 10] = (uint16_t)((t.color >> 16) & 0xFF);
}

// Per-tile bin counts. Reset at the start of each frame; incremented as
// triangles are streamed through bin_triangle(). Models a per-tile counter
// register in HW.
static int bin_count[NUM_TILES];

// Bin one triangle into all overlapping tile buckets. Each bucket lives at a
// fixed address (BINLIST_BASE + tile*MAX_FACES_PER_TILE), so the only state
// needed is the per-tile write pointer (bin_count[tile]).
void bin_triangle(auto &mem, int tri_id, const Tri2D &t) {
    int tx_min = std::max(0, t.bbminx / TW);
    int tx_max = std::min(NTX - 1, t.bbmaxx / TW);
    int ty_min = std::max(0, t.bbminy / TH);
    int ty_max = std::min(NTY - 1, t.bbmaxy / TH);
    for (int ty = ty_min; ty <= ty_max; ty++) {
        for (int tx = tx_min; tx <= tx_max; tx++) {
            int tile = ty * NTX + tx;
            int idx = BINLIST_BASE + tile * MAX_FACES_PER_TILE + bin_count[tile];
            mem[idx] = (uint16_t)tri_id;
            bin_count[tile]++;
        }
    }
}

// After all triangles for a frame are binned, write the per-tile count table
// the GPU consumes. Bucket offsets are implicit: tile T is at
// BINLIST_BASE + T * MAX_FACES_PER_TILE (computed in HW).
void finalize_bins(auto &mem) {
    for (int tile = 0; tile < NUM_TILES; tile++) {
        mem[BIN_BASE + tile] = (uint16_t)bin_count[tile];
    }
}

// Dump the entire SDRAM contents as a flat $readmemh hex file (one 16-bit word
// per line, in address order). Loadable directly into the SDRAM model's BRAM
// via $readmemh on real HW. Small enough (128 K words) to dump in full.
void dump_sdram_hex(const char *fn, const auto &mem) {
    FILE *f = fopen(fn, "w");
    if (!f) { printf("ERROR: cannot open %s for write\n", fn); return; }
    for (int i = 0; i < SDRAM_WORDS; i++)
        fprintf(f, "%04X\n", mem[i]);
    fclose(f);
}

void write_ppm(const char *fn) {
    FILE *f = fopen(fn, "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint32_t c = framebuffer[y][x];
            uint8_t rgb[3] = {(uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    Vsystem_top *dut = new Vsystem_top;

    bool do_trace = false;
    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--trace") == 0) do_trace = true;

    VerilatedFstC *tfp = nullptr;
    if (do_trace) {
        tfp = new VerilatedFstC;
        dut->trace(tfp, 99);
        tfp->open("gpu.fst");
    }
    int sim_time = 0;

    load_obj("suzanne.obj");
    if (vertices.empty()) return 1;

    printf("Screen: %dx%d, Tile: %dx%d, Grid: %dx%d\n", W, H, TW, TH, NTX, NTY);

    // Backdoor pointer to SDRAM model memory
    auto &sdram_mem = dut->system_top->sdram->mem_inst->mem;

    const int NUM_FRAMES = 90;

    // Reset
    dut->rst = 1;
    dut->display_enable = 0;
    dut->display_frame_start = 0;
    // Drive the display pixel clock-enable at full rate in this standalone
    // testbench (verified correct; the FIFO produces the same stream).
    dut->display_pix_ce = 1;
    dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    dut->rst = 0;

    // Wait for SDRAM init (100us = 10000 cycles)
    for (int i = 0; i < 11000; i++) {
        dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    }

    // Turn on the mocked display controller after SDRAM is initialized.
    // It will continuously read the FB and apply bus pressure.
    dut->display_enable = 1;

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        float angle_y = frame * 2.0f * (float)M_PI / NUM_FRAMES;
        float angle_x = -0.3f;
        glm::mat3 R = build_view_rot(angle_y, angle_x);

#ifndef SDRAM_INIT_MODE
        // Process each face end-to-end (transform -> normal -> cull -> light
        // -> project -> bbox -> bin -> tri record), one triangle at a time.
        // Models the streaming dataflow of a HW geometry pipeline.
        std::fill_n(bin_count, NUM_TILES, 0);
        int tri_id = 0;
        for (auto &face : faces) {
            auto t = process_face(face, R);
            if (!t) continue;
            write_tri_record(sdram_mem, tri_id, *t);
            bin_triangle(sdram_mem, tri_id, *t);
            tri_id++;
        }

        // Flush per-tile (offset, count) table the GPU reads.
        finalize_bins(sdram_mem);

        // Dump the prepared geometry for the first frame as a hex preload file
        // (for moving to real HW; loadable via $readmemh into the SDRAM BRAM).
        if (frame == 0)
            dump_sdram_hex("sdram_init.hex", sdram_mem);
#else
        // Geometry is preloaded once via $readmemh (SDRAM_INIT_FILE). We do not
        // repopulate it, so every frame renders the same preloaded image.
        (void)R;
        int tri_id = 0;
#endif

        // Clear FB region in SDRAM
        for (int i = 0; i < W * H; i++) sdram_mem[FB_BASE + i] = 0;

        // Start GPU
        dut->start = 1;
        dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->start = 0;

        int timeout = 50000000;
        while (!dut->done && timeout-- > 0) {
            dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
            dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        }
        if (timeout <= 0) { printf("Frame %03d: TIMEOUT\n", frame); break; }

        // Capture the framebuffer through the display controller.
        // 1. Disable the display so it stops issuing new reads.
        // 2. Wait until the SDRAM bus is idle (no in-flight requests). This
        //    guarantees all GPU writes have committed and all display reads
        //    have returned.
        // 3. Pulse frame_start so the display restarts at FB_BASE+0 with a
        //    clean FIFO state.
        // 4. Re-enable the display and collect 320*200 pixels from the
        //    pix_valid stream.
        dut->display_enable = 0;

        int idle_streak = 0;
        int idle_timeout = 10000;
        while (idle_streak < 16 && idle_timeout-- > 0) {
            dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
            dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
            idle_streak = dut->sdram_idle ? idle_streak + 1 : 0;
        }
        if (idle_timeout <= 0)
            printf("Frame %03d: SDRAM never went idle\n", frame);

        dut->display_frame_start = 1;
        dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->display_frame_start = 0;
        dut->display_enable = 1;

        int captured = 0;
        int cap_timeout = W * H * 16;
        while (captured < W * H && cap_timeout-- > 0) {
            dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
            dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
            if (dut->display_pix_valid) {
                uint16_t c = dut->display_pix_data;
                int y = captured / W;
                int x = captured % W;
                uint8_t r = (c >> 11) << 3;
                uint8_t g = ((c >> 5) & 0x3F) << 2;
                uint8_t b = (c & 0x1F) << 3;
                framebuffer[y][x] = (r << 16) | (g << 8) | b;
                captured++;
            }
        }
        if (cap_timeout <= 0)
            printf("Frame %03d: display capture timeout (got %d/%d)\n",
                   frame, captured, W * H);

        char fn[64];
        snprintf(fn, sizeof(fn), "frame_%03d.ppm", frame);
        write_ppm(fn);
        printf("Frame %03d: %d tris\n", frame, tri_id);
    }

    printf("\nDone\n");
    if (tfp) tfp->close();
    delete dut;
    return 0;
}
