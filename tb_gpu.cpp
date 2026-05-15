#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vgpu_top.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

static const int W = 320, H = 200;
static const int TW = 64, TH = 64;
static const int NTX = (W + TW - 1) / TW;
static const int NTY = (H + TH - 1) / TH;
static const int NUM_TILES = NTX * NTY;
static const int SP = 16; // 1 << SUBPIXEL

// Memory map (word addresses)
static const int TRI_BASE     = 0x000000;
static const int BIN_BASE     = 0x080000;
static const int BINLIST_BASE = 0x081000;
static const int FB_BASE      = 0x100000;

// External memory model
static const int MEM_SIZE = 0x200000; // 2M words
static uint16_t ext_mem[MEM_SIZE];

// Output framebuffer
static uint32_t framebuffer[H][W];

struct Vec3 { float x, y, z; };
struct Face { int v[3]; };
struct Vec2i { int x, y; };

struct Tri2D {
    Vec2i p[3];
    float iz[3];
    uint32_t color;
    int bbminx, bbminy, bbmaxx, bbmaxy;
};

std::vector<Vec3> vertices;
std::vector<Face> faces;

void load_obj(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { printf("ERROR: cannot open %s\n", path); return; }
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            Vec3 v; sscanf(line+2, "%f %f %f", &v.x, &v.y, &v.z);
            vertices.push_back(v);
        } else if (line[0] == 'f' && line[1] == ' ') {
            int vi[4]={0}, count=0; char *p=line+2;
            while (*p && count<4) { vi[count++]=atoi(p)-1; while(*p&&*p!=' '&&*p!='\n')p++; while(*p==' ')p++; }
            faces.push_back({{vi[0],vi[1],vi[2]}});
            if (count==4) faces.push_back({{vi[0],vi[2],vi[3]}});
        }
    }
    fclose(f);
    printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

void project(float angle_y, float angle_x,
             std::vector<Vec2i> &proj, std::vector<float> &proj_iz) {
    float cy=cosf(angle_y), sy=sinf(angle_y), cx=cosf(angle_x), sx=sinf(angle_x);
    float scale = 80.0f;
    int nv = vertices.size();
    proj.resize(nv); proj_iz.resize(nv);
    for (int i = 0; i < nv; i++) {
        float x=vertices[i].x, y=-vertices[i].y, z=vertices[i].z;
        float rx=x*cy+z*sy, rz=-x*sy+z*cy;
        float ry=y*cx-rz*sx, rz2=y*sx+rz*cx;
        proj[i] = {(int)(rx*scale+W/2), (int)(ry*scale+H/2)};
        proj_iz[i] = 275.0f + rz2*225.0f;
        if (proj_iz[i] < 1.0f) proj_iz[i] = 1.0f;
    }
}

uint32_t shade_face(Vec3 v0, Vec3 v1, Vec3 v2, float angle_y, float angle_x) {
    Vec3 e1={v1.x-v0.x,v1.y-v0.y,v1.z-v0.z}, e2={v2.x-v0.x,v2.y-v0.y,v2.z-v0.z};
    Vec3 n={e1.y*e2.z-e1.z*e2.y, e1.z*e2.x-e1.x*e2.z, e1.x*e2.y-e1.y*e2.x};
    float len=sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);
    if (len<1e-6f) return 0x404040;
    n.x/=len; n.y/=len; n.z/=len;
    float cy=cosf(angle_y),sy=sinf(angle_y),cx=cosf(angle_x),sx=sinf(angle_x);
    float nx2=n.x*cy+n.z*sy, nz2=-n.x*sy+n.z*cy;
    float ny2=n.y*cx-nz2*sx, nz3=n.y*sx+nz2*cx;
    float dot=nx2*0.186f+ny2*0.279f+nz3*0.932f;
    if (dot<0) dot=0;
    float i2=0.2f+0.8f*dot;
    uint8_t r=(uint8_t)(i2*(0.5f+0.5f*n.x)*255);
    uint8_t g=(uint8_t)(i2*(0.5f+0.5f*n.y)*255);
    uint8_t b=(uint8_t)(i2*(0.5f+0.5f*n.z)*255);
    return (r<<16)|(g<<8)|b;
}

// Write triangle record to ext_mem at TRI_BASE + tri_id*16
void write_tri_record(int tri_id, const Tri2D &t) {
    int base = TRI_BASE + tri_id * 16;
    ext_mem[base+0] = (uint16_t)(t.p[0].x * SP + SP/2);
    ext_mem[base+1] = (uint16_t)(t.p[0].y * SP + SP/2);
    ext_mem[base+2] = (uint16_t)(t.p[1].x * SP + SP/2);
    ext_mem[base+3] = (uint16_t)(t.p[1].y * SP + SP/2);
    ext_mem[base+4] = (uint16_t)(t.p[2].x * SP + SP/2);
    ext_mem[base+5] = (uint16_t)(t.p[2].y * SP + SP/2);

    // Compute iz plane at the triangle's screen-space bbox origin
    float area = (float)((t.p[1].x-t.p[0].x)*(t.p[2].y-t.p[0].y) -
                         (t.p[1].y-t.p[0].y)*(t.p[2].x-t.p[0].x));
    float diz_dx=0, diz_dy=0, iz_at_bb=t.iz[0];
    if (fabsf(area) > 0.001f) {
        diz_dx = ((t.iz[1]-t.iz[0])*(t.p[2].y-t.p[0].y) -
                  (t.iz[2]-t.iz[0])*(t.p[1].y-t.p[0].y)) / area;
        diz_dy = ((t.iz[2]-t.iz[0])*(t.p[1].x-t.p[0].x) -
                  (t.iz[1]-t.iz[0])*(t.p[2].x-t.p[0].x)) / area;
    }
    // iz_init: value at the HW's clamped bbox origin for this tile
    // Since the HW clamps per-tile, we store the plane coefficients and
    // let the HW compute iz_init at its own bbox origin.
    // Actually the HW loads iz_init directly — so we need to store it
    // at the correct point. But we don't know which tile will use this triangle.
    // Solution: store iz at vertex 0, plus gradients. The TB will compute
    // iz_init per-tile when writing bin lists... but that requires per-tile
    // per-triangle data which is expensive.
    //
    // Simpler: store diz_dx, diz_dy, and iz at (0,0) in screen space.
    // The HW can compute iz_at_bbox = iz_at_00 + diz_dx * minx + diz_dy * miny.
    // But that requires a multiply in HW which we removed...
    //
    // Simplest for now: store iz_init at the triangle's own bbox origin (unclamped).
    // The HW's bbox is clamped to the tile, so there's a mismatch.
    // We need the HW to step iz from the triangle's bbox to the tile's bbox.
    // Since iz steps linearly, the HW can do:
    //   iz_row = iz_init + iz_dx * (minx - tri_bbminx) + iz_dy * (miny - tri_bbminy)
    // But that's again a multiply...
    //
    // OK, let's just store iz_init as iz at pixel (0,0) and have the HW
    // do iz_row = iz_init + iz_dx * minx + iz_dy * miny using adds in a loop
    // during SETUP. minx is at most 319, so 319 adds of iz_dx. That's too slow.
    //
    // Best approach: store iz_init at (0,0). The HW accumulates iz_dx * minx
    // by shifting (minx is known). Actually just do the multiply — it's only
    // 16-bit × 9-bit, one DSP, one cycle. Let's add that back to the HW.
    //
    // For now: store iz at (0,0) = iz0 - diz_dx*p0.x - diz_dy*p0.y
    float iz_at_00 = t.iz[0] - diz_dx*t.p[0].x - diz_dy*t.p[0].y;

    ext_mem[base+6] = (uint16_t)(int16_t)roundf(iz_at_00 * 32.0f);
    ext_mem[base+7] = (uint16_t)(int16_t)roundf(diz_dx * 32.0f);
    ext_mem[base+8] = (uint16_t)(int16_t)roundf(diz_dy * 32.0f);
    ext_mem[base+9] = (uint16_t)(t.color & 0xFFFF);       // {G, R}
    ext_mem[base+10] = (uint16_t)((t.color >> 16) & 0xFF); // {0, B}
    // 11-15 reserved
}

// Prepare bin lists in ext_mem
void prepare_bins(const std::vector<Tri2D> &tris) {
    // Write bin pointer table and bin lists
    int list_offset = 0;
    for (int tile = 0; tile < NUM_TILES; tile++) {
        int tx = (tile % NTX) * TW;
        int ty = (tile / NTX) * TH;
        int tile_xmax = std::min(tx + TW - 1, W - 1);
        int tile_ymax = std::min(ty + TH - 1, H - 1);

        int start_offset = list_offset;
        for (int i = 0; i < (int)tris.size(); i++) {
            const auto &t = tris[i];
            if (t.bbmaxx < tx || t.bbminx > tile_xmax) continue;
            if (t.bbmaxy < ty || t.bbminy > tile_ymax) continue;
            ext_mem[BINLIST_BASE + list_offset] = (uint16_t)i;
            list_offset++;
        }
        int count = list_offset - start_offset;
        ext_mem[BIN_BASE + tile*2]     = (uint16_t)start_offset;
        ext_mem[BIN_BASE + tile*2 + 1] = (uint16_t)count;
    }
}

void write_ppm(const char *fn) {
    FILE *f = fopen(fn, "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint32_t c = framebuffer[y][x];
            uint8_t rgb[3] = {(uint8_t)(c>>16), (uint8_t)(c>>8), (uint8_t)c};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    Vgpu_top *dut = new Vgpu_top;

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

    const int NUM_FRAMES = 90;
    std::vector<Vec2i> proj;
    std::vector<float> proj_iz;
    std::vector<Tri2D> tris;

    // Reset
    dut->rst = 1;
    dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    dut->rst = 0;

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        float angle_y = frame * 2.0f * M_PI / NUM_FRAMES;
        float angle_x = -0.3f;

        project(angle_y, angle_x, proj, proj_iz);

        // Build visible triangles
        tris.clear();
        for (auto &face : faces) {
            Vec2i p0=proj[face.v[0]], p1=proj[face.v[1]], p2=proj[face.v[2]];
            int cross = (p1.x-p0.x)*(p2.y-p0.y) - (p1.y-p0.y)*(p2.x-p0.x);
            if (cross >= 0) continue;
            // Skip if any vertex off-screen (unsigned port limitation)
            if (p0.x<0||p0.x>=W||p0.y<0||p0.y>=H) continue;
            if (p1.x<0||p1.x>=W||p1.y<0||p1.y>=H) continue;
            if (p2.x<0||p2.x>=W||p2.y<0||p2.y>=H) continue;
            Tri2D t;
            t.p[0]=p0; t.p[1]=p2; t.p[2]=p1; // swap winding
            t.iz[0]=proj_iz[face.v[0]]; t.iz[1]=proj_iz[face.v[2]]; t.iz[2]=proj_iz[face.v[1]];
            t.color = shade_face(vertices[face.v[0]], vertices[face.v[1]],
                                 vertices[face.v[2]], angle_y, angle_x);
            t.bbminx = std::min({t.p[0].x, t.p[1].x, t.p[2].x});
            t.bbminy = std::min({t.p[0].y, t.p[1].y, t.p[2].y});
            t.bbmaxx = std::max({t.p[0].x, t.p[1].x, t.p[2].x});
            t.bbmaxy = std::max({t.p[0].y, t.p[1].y, t.p[2].y});
            tris.push_back(t);
        }

        // Write triangle records to ext_mem
        for (int i = 0; i < (int)tris.size(); i++)
            write_tri_record(i, tris[i]);

        // Prepare bin lists
        prepare_bins(tris);

        // Clear FB region
        memset(&ext_mem[FB_BASE], 0, W * H * sizeof(uint16_t));

        // Start GPU
        dut->start = 1;
        dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->start = 0;

        // Run until done
        int timeout = 100000000;
        uint16_t mem_rd_reg = 0; // 1-cycle read latency model
        while (!dut->done && timeout-- > 0) {
            if (dut->mem_we) {
                ext_mem[dut->mem_addr % MEM_SIZE] = dut->mem_wr_data;
            }
            dut->mem_rd_data = mem_rd_reg;
            dut->clk = 0; dut->eval();
            if (tfp) tfp->dump(sim_time); sim_time++;
            mem_rd_reg = ext_mem[dut->mem_addr % MEM_SIZE];
            dut->clk = 1; dut->eval();
            if (tfp) tfp->dump(sim_time); sim_time++;
        }
        if (timeout <= 0) { printf("Frame %03d: TIMEOUT\n", frame); break; }

        // Read back framebuffer from ext_mem
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                uint16_t c = ext_mem[FB_BASE + y*W + x];
                uint8_t r = (c >> 11) << 3;
                uint8_t g = ((c >> 5) & 0x3F) << 2;
                uint8_t b = (c & 0x1F) << 3;
                framebuffer[y][x] = (r<<16)|(g<<8)|b;
            }
        }

        char fn[64];
        snprintf(fn, sizeof(fn), "frame_%03d.ppm", frame);
        write_ppm(fn);
        printf("Frame %03d: %zu tris\n", frame, tris.size());
    }

    printf("\nDone\n");
    if (tfp) tfp->close();
    delete dut;
    return 0;
}
