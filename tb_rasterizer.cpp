#include <verilated.h>
#include "Vraster_top.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

// Screen
static const int W = 320, H = 200;
// Tile (must match HW parameters)
static const int TW = 64, TH = 64;
static const int NTX = (W + TW - 1) / TW;  // 5
static const int NTY = (H + TH - 1) / TH;  // 4 (last partial)

static uint32_t fb_hw[H][W];

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
            Vec3 v;
            sscanf(line + 2, "%f %f %f", &v.x, &v.y, &v.z);
            vertices.push_back(v);
        } else if (line[0] == 'f' && line[1] == ' ') {
            int vi[4] = {0}, count = 0;
            char *p = line + 2;
            while (*p && count < 4) {
                vi[count++] = atoi(p) - 1;
                while (*p && *p != ' ' && *p != '\n') p++;
                while (*p == ' ') p++;
            }
            faces.push_back({{vi[0], vi[1], vi[2]}});
            if (count == 4) faces.push_back({{vi[0], vi[2], vi[3]}});
        }
    }
    fclose(f);
    printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

void project(float angle_y, float angle_x,
             std::vector<Vec2i> &proj, std::vector<float> &proj_iz) {
    float cy = cosf(angle_y), sy = sinf(angle_y);
    float cx = cosf(angle_x), sx = sinf(angle_x);
    float scale = 80.0f;
    int nv = vertices.size();
    proj.resize(nv);
    proj_iz.resize(nv);
    for (int i = 0; i < nv; i++) {
        float x = vertices[i].x, y = -vertices[i].y, z = vertices[i].z;
        float rx = x*cy + z*sy;
        float rz = -x*sy + z*cy;
        float ry = y*cx - rz*sx;
        float rz2 = y*sx + rz*cx;
        proj[i].x = (int)(rx*scale + W/2);
        proj[i].y = (int)(ry*scale + H/2);
        proj_iz[i] = 275.0f + rz2*225.0f;
        if (proj_iz[i] < 1.0f) proj_iz[i] = 1.0f;
    }
}

uint32_t shade_face(Vec3 v0, Vec3 v1, Vec3 v2, float angle_y, float angle_x) {
    Vec3 e1 = {v1.x-v0.x, v1.y-v0.y, v1.z-v0.z};
    Vec3 e2 = {v2.x-v0.x, v2.y-v0.y, v2.z-v0.z};
    Vec3 n = {e1.y*e2.z - e1.z*e2.y, e1.z*e2.x - e1.x*e2.z, e1.x*e2.y - e1.y*e2.x};
    float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
    if (len < 1e-6f) return 0x404040;
    n.x /= len; n.y /= len; n.z /= len;
    float cy = cosf(angle_y), sy = sinf(angle_y);
    float cx = cosf(angle_x), sx = sinf(angle_x);
    float nx2 = n.x*cy + n.z*sy;
    float nz2 = -n.x*sy + n.z*cy;
    float ny2 = n.y*cx - nz2*sx;
    float nz3 = n.y*sx + nz2*cx;
    float lx = 0.186f, ly = 0.279f, lz = 0.932f;
    float dot = nx2*lx + ny2*ly + nz3*lz;
    if (dot < 0) dot = 0;
    float intensity = 0.2f + 0.8f*dot;
    float rb = 0.5f + 0.5f*n.x, gb = 0.5f + 0.5f*n.y, bb = 0.5f + 0.5f*n.z;
    uint8_t r = (uint8_t)(intensity*rb*255);
    uint8_t g = (uint8_t)(intensity*gb*255);
    uint8_t b = (uint8_t)(intensity*bb*255);
    return (r<<16)|(g<<8)|b;
}

void clear_tile(Vraster_top *dut) {
    dut->clear = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->clear = 0;

    int timeout = 10000;
    while (!dut->clear_done && timeout-- > 0) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
    }
}

void draw_triangle_hw(Vraster_top *dut, const Tri2D &t, int tile_ox, int tile_oy) {
    const int SP = 16;

    // Skip if entirely off-screen
    if ((t.p[0].x < 0 && t.p[1].x < 0 && t.p[2].x < 0) ||
        (t.p[0].x >= W && t.p[1].x >= W && t.p[2].x >= W) ||
        (t.p[0].y < 0 && t.p[1].y < 0 && t.p[2].y < 0) ||
        (t.p[0].y >= H && t.p[1].y >= H && t.p[2].y >= H)) return;

    // For now, conservative skip if any vertex is outside screen.
    // (Hardware uses unsigned vertex inputs; negative would wrap.)
    for (int i = 0; i < 3; i++) {
        if (t.p[i].x < 0 || t.p[i].x >= W || t.p[i].y < 0 || t.p[i].y >= H) return;
    }

    // 1/z plane gradients (in screen space)
    float area = (float)((t.p[1].x - t.p[0].x) * (t.p[2].y - t.p[0].y) -
                         (t.p[1].y - t.p[0].y) * (t.p[2].x - t.p[0].x));
    if (fabsf(area) < 0.001f) return;
    float diz_dx = ((t.iz[1]-t.iz[0])*(t.p[2].y-t.p[0].y) -
                    (t.iz[2]-t.iz[0])*(t.p[1].y-t.p[0].y)) / area;
    float diz_dy = ((t.iz[2]-t.iz[0])*(t.p[1].x-t.p[0].x) -
                    (t.iz[1]-t.iz[0])*(t.p[2].x-t.p[0].x)) / area;

    // iz at the clamped bbox origin (matching what HW will use)
    int tile_xmax = std::min(tile_ox + TW - 1, W - 1);
    int tile_ymax = std::min(tile_oy + TH - 1, H - 1);
    int bbminx = std::max(tile_ox, std::min({t.p[0].x, t.p[1].x, t.p[2].x}));
    int bbminy = std::max(tile_oy, std::min({t.p[0].y, t.p[1].y, t.p[2].y}));
    int bbmaxx = std::min(tile_xmax, std::max({t.p[0].x, t.p[1].x, t.p[2].x}));
    int bbmaxy = std::min(tile_ymax, std::max({t.p[0].y, t.p[1].y, t.p[2].y}));
    if (bbminx > bbmaxx || bbminy > bbmaxy) return;
    bbminx &= ~1;
    bbminy &= ~1;

    float iz_at_bb = t.iz[0] + diz_dx * (bbminx - t.p[0].x) + diz_dy * (bbminy - t.p[0].y);

    int16_t iz_init_fp = (int16_t)roundf(iz_at_bb * 32.0f);
    int16_t iz_dx_fp = (int16_t)roundf(diz_dx * 32.0f);
    int16_t iz_dy_fp = (int16_t)roundf(diz_dy * 32.0f);

    dut->tile_x = tile_ox;
    dut->tile_y = tile_oy;
    dut->v0_x = t.p[0].x * SP + SP/2;
    dut->v0_y = t.p[0].y * SP + SP/2;
    dut->v1_x = t.p[1].x * SP + SP/2;
    dut->v1_y = t.p[1].y * SP + SP/2;
    dut->v2_x = t.p[2].x * SP + SP/2;
    dut->v2_y = t.p[2].y * SP + SP/2;
    dut->iz_init = iz_init_fp;
    dut->iz_dx = iz_dx_fp;
    dut->iz_dy = iz_dy_fp;
    dut->color = t.color;
    dut->start = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->start = 0;

    int timeout = 100000;
    while (!dut->done && timeout-- > 0) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
    }
    if (timeout <= 0) printf("ERROR: timeout (tile %d,%d)\n", tile_ox, tile_oy);
}

void read_tile(Vraster_top *dut, int tile_ox, int tile_oy) {
    for (int y = 0; y < TH && (tile_oy + y) < H; y++) {
        for (int x = 0; x < TW && (tile_ox + x) < W; x++) {
            int qx = x / 2, qy = y / 2;
            int pidx = (y % 2) * 2 + (x % 2);
            dut->fb_rd_pixel_addr = (qy * (TW/2) + qx) * 4 + pidx;
            dut->clk = 0; dut->eval();
            dut->clk = 1; dut->eval();
            uint16_t c = dut->fb_rd_data;
            uint8_t r = (c >> 11) << 3;
            uint8_t g = ((c >> 5) & 0x3F) << 2;
            uint8_t b = (c & 0x1F) << 3;
            fb_hw[tile_oy + y][tile_ox + x] = (r << 16) | (g << 8) | b;
        }
    }
}

void write_ppm(const char *fn) {
    FILE *f = fopen(fn, "wb");
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

    load_obj("suzanne.obj");
    if (vertices.empty()) return 1;

    printf("Screen: %dx%d, Tile: %dx%d, Grid: %dx%d = %d tiles\n",
           W, H, TW, TH, NTX, NTY, NTX*NTY);

    const int NUM_FRAMES = 90;
    std::vector<Vec2i> proj;
    std::vector<float> proj_iz;
    std::vector<Tri2D> tris;

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        memset(fb_hw, 0, sizeof(fb_hw));

        float angle_y = frame * 2.0f * M_PI / NUM_FRAMES;
        float angle_x = -0.3f;

        project(angle_y, angle_x, proj, proj_iz);

        // Build visible triangle list
        tris.clear();
        for (auto &face : faces) {
            Vec2i p0 = proj[face.v[0]], p1 = proj[face.v[1]], p2 = proj[face.v[2]];
            int cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            if (cross >= 0) continue;
            Tri2D t;
            // Swap v1/v2 to fix winding
            t.p[0] = p0; t.p[1] = p2; t.p[2] = p1;
            t.iz[0] = proj_iz[face.v[0]];
            t.iz[1] = proj_iz[face.v[2]];
            t.iz[2] = proj_iz[face.v[1]];
            t.color = shade_face(vertices[face.v[0]], vertices[face.v[1]],
                                 vertices[face.v[2]], angle_y, angle_x);
            t.bbminx = std::min({p0.x, p1.x, p2.x});
            t.bbminy = std::min({p0.y, p1.y, p2.y});
            t.bbmaxx = std::max({p0.x, p1.x, p2.x});
            t.bbmaxy = std::max({p0.y, p1.y, p2.y});
            tris.push_back(t);
        }

        // SW binning + tile rendering
        int total_drawn = 0;
        for (int ty = 0; ty < NTY; ty++) {
            for (int tx = 0; tx < NTX; tx++) {
                int tile_ox = tx * TW;
                int tile_oy = ty * TH;
                int tile_xmax = std::min(tile_ox + TW - 1, W - 1);
                int tile_ymax = std::min(tile_oy + TH - 1, H - 1);

                // Build bin: triangles whose bbox overlaps this tile
                std::vector<int> bin;
                for (int i = 0; i < (int)tris.size(); i++) {
                    const auto &t = tris[i];
                    if (t.bbmaxx < tile_ox || t.bbminx > tile_xmax) continue;
                    if (t.bbmaxy < tile_oy || t.bbminy > tile_ymax) continue;
                    bin.push_back(i);
                }
                if (bin.empty()) continue;

                clear_tile(dut);
                for (int idx : bin) {
                    draw_triangle_hw(dut, tris[idx], tile_ox, tile_oy);
                    total_drawn++;
                }
                read_tile(dut, tile_ox, tile_oy);
            }
        }

        char fn[64];
        snprintf(fn, sizeof(fn), "frame_%03d.ppm", frame);
        write_ppm(fn);
        printf("Frame %03d: %zu tris, %d submissions\n", frame, tris.size(), total_drawn);
    }

    printf("\nDone\n");
    delete dut;
    return 0;
}
