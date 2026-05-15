#include <verilated.h>
#include "Vraster_top.h"
#include "Vraster_top_raster_top.h"
#include "Vraster_top_dpram__Ae_DB3e80.h"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>

static const int W = 320, H = 200;
static uint32_t fb_hw[H][W];

struct Vec3 { float x, y, z; };
struct Face { int v[3]; };

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
            // Parse face (may have v//vn or v/vt/vn format)
            Face face;
            int vi[4] = {0}, count = 0;
            char *p = line + 2;
            while (*p && count < 4) {
                vi[count] = atoi(p) - 1; // OBJ is 1-indexed
                count++;
                while (*p && *p != ' ' && *p != '\n') p++;
                while (*p == ' ') p++;
            }
            // Triangulate quads
            face.v[0] = vi[0]; face.v[1] = vi[1]; face.v[2] = vi[2];
            faces.push_back(face);
            if (count == 4) {
                face.v[0] = vi[0]; face.v[1] = vi[2]; face.v[2] = vi[3];
                faces.push_back(face);
            }
        }
    }
    fclose(f);
    printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

struct Vec2i { int x, y; };

void project(const Vec3 *verts, int nv, float angle_y, float angle_x,
             Vec2i *out, float *out_iz) {
    float cy = cosf(angle_y), sy = sinf(angle_y);
    float cx = cosf(angle_x), sx = sinf(angle_x);
    float scale = 80.0f;

    for (int i = 0; i < nv; i++) {
        // Flip Y to put model upright (screen Y increases downward)
        float x = verts[i].x, y = -verts[i].y, z = verts[i].z;
        float rx = x * cy + z * sy;
        float rz = -x * sy + z * cy;
        float ry = y * cx - rz * sx;
        float rz2 = y * sx + rz * cx;

        // Orthographic projection + scale to screen
        out[i].x = (int)((rx * scale) + W/2);
        out[i].y = (int)((ry * scale) + H/2);
        // 1/z: closer to camera (larger rz2) should have larger iz
        out_iz[i] = 275.0f + rz2 * 225.0f;
        if (out_iz[i] < 1.0f) out_iz[i] = 1.0f;
    }
}

void draw_triangle_hw(Vraster_top *dut, Vec2i v0, Vec2i v1, Vec2i v2,
                      float iz0, float iz1, float iz2,
                      uint32_t color) {
    const int SP = 16;
    // Skip if any vertex is off-screen (simple guard — no real clipping)
    auto offscreen = [](Vec2i v) { return v.x < 0 || v.x >= W || v.y < 0 || v.y >= H; };
    if (offscreen(v0) || offscreen(v1) || offscreen(v2)) return;

    // Compute 1/z plane equation: iz(x,y) = iz_init + iz_dx*x + iz_dy*y
    // Using barycentric interpolation divided by area
    float area = (float)((v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x));
    if (fabsf(area) < 0.001f) return;

    // Gradients: diz/dx and diz/dy
    float diz_dx = ((iz1 - iz0) * (v2.y - v0.y) - (iz2 - iz0) * (v1.y - v0.y)) / area;
    float diz_dy = ((iz2 - iz0) * (v1.x - v0.x) - (iz1 - iz0) * (v2.x - v0.x)) / area;

    // Compute iz at bounding box origin (what the HW will use as starting point)
    int bbminx = std::min({v0.x, v1.x, v2.x}) & ~1;
    int bbminy = std::min({v0.y, v1.y, v2.y}) & ~1;
    if (bbminx < 0) bbminx = 0;
    if (bbminy < 0) bbminy = 0;
    float iz_at_bb = iz0 + diz_dx * (bbminx - v0.x) + diz_dy * (bbminy - v0.y);

    // Scale by 32 for better z precision. Max iz value: 500*32=16000, fits int16.
    // Max gradient per pixel * 32 ~ small (Suzanne spans ~160px, iz range ~450, grad~3*32=96)
    // iz_at_bb max: 500*32=16000, well within int16 range
    int16_t iz_init_fp = (int16_t)roundf(iz_at_bb * 32.0f);
    int16_t iz_dx_fp = (int16_t)roundf(diz_dx * 32.0f);
    int16_t iz_dy_fp = (int16_t)roundf(diz_dy * 32.0f);

    dut->v0_x = v0.x * SP + SP/2;
    dut->v0_y = v0.y * SP + SP/2;
    dut->v1_x = v1.x * SP + SP/2;
    dut->v1_y = v1.y * SP + SP/2;
    dut->v2_x = v2.x * SP + SP/2;
    dut->v2_y = v2.y * SP + SP/2;
    dut->iz_init = iz_init_fp;
    dut->iz_dx = iz_dx_fp;
    dut->iz_dy = iz_dy_fp;
    dut->color = color;
    dut->start = 1;
    dut->clk = 0; dut->eval();
    dut->clk = 1; dut->eval();
    dut->start = 0;

    int timeout = 500000;
    while (!dut->done && timeout-- > 0) {
        dut->clk = 0; dut->eval();
        dut->clk = 1; dut->eval();
    }
    if (timeout <= 0) printf("ERROR: timeout\n");
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

// Simple face normal for flat shading — compute in view space
uint32_t shade_face(Vec3 v0, Vec3 v1, Vec3 v2, float angle_y, float angle_x) {
    // Face normal in object space
    Vec3 e1 = {v1.x-v0.x, v1.y-v0.y, v1.z-v0.z};
    Vec3 e2 = {v2.x-v0.x, v2.y-v0.y, v2.z-v0.z};
    Vec3 n = {e1.y*e2.z - e1.z*e2.y, e1.z*e2.x - e1.x*e2.z, e1.x*e2.y - e1.y*e2.x};
    float len = sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
    if (len < 1e-6f) return 0x404040;
    n.x /= len; n.y /= len; n.z /= len;

    // Rotate normal to view space
    float cy = cosf(angle_y), sy = sinf(angle_y);
    float cx = cosf(angle_x), sx = sinf(angle_x);
    float nx2 = n.x * cy + n.z * sy;
    float nz2 = -n.x * sy + n.z * cy;
    float ny2 = n.y * cx - nz2 * sx;
    float nz3 = n.y * sx + nz2 * cx;

    // Light direction
    float lx = 0.186f, ly = 0.279f, lz = 0.932f;
    float dot = nx2 * lx + ny2 * ly + nz3 * lz;
    if (dot < 0) dot = 0;
    float intensity = 0.2f + 0.8f * dot;

    // Color from object-space normal direction (gives each face a unique hue)
    float r_base = 0.5f + 0.5f * n.x;
    float g_base = 0.5f + 0.5f * n.y;
    float b_base = 0.5f + 0.5f * n.z;

    uint8_t r = (uint8_t)(intensity * r_base * 255);
    uint8_t g = (uint8_t)(intensity * g_base * 255);
    uint8_t b = (uint8_t)(intensity * b_base * 255);
    return (r << 16) | (g << 8) | b;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    Vraster_top *dut = new Vraster_top;

    load_obj("suzanne.obj");
    if (vertices.empty()) return 1;

    const int NUM_FRAMES = 90;
    int nv = vertices.size();
    std::vector<Vec2i> proj(nv);
    std::vector<float> proj_iz(nv);

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        clear_hw(dut);

        float angle_y = frame * 2.0f * M_PI / NUM_FRAMES;
        float angle_x = -0.3f; // slight tilt (negative to flip upright)

        project(vertices.data(), nv, angle_y, angle_x, proj.data(), proj_iz.data());

        int drawn = 0;
        for (auto &face : faces) {
            Vec2i p0 = proj[face.v[0]], p1 = proj[face.v[1]], p2 = proj[face.v[2]];
            float iz0 = proj_iz[face.v[0]];
            float iz1 = proj_iz[face.v[1]];
            float iz2 = proj_iz[face.v[2]];

            // Back-face culling (Y negated in projection flips winding)
            int cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
            if (cross >= 0) continue;

            uint32_t color = shade_face(vertices[face.v[0]], vertices[face.v[1]],
                                        vertices[face.v[2]], angle_y, angle_x);

            draw_triangle_hw(dut, p0, p2, p1, iz0, iz2, iz1, color);
            drawn++;
        }

        read_fb(dut);

        char filename[64];
        snprintf(filename, sizeof(filename), "frame_%03d.ppm", frame);
        write_ppm(filename);
        printf("Frame %03d: %d triangles drawn\n", frame, drawn);
    }

    printf("\nDone: %d frames rendered\n", NUM_FRAMES);
    delete dut;
    return 0;
}
