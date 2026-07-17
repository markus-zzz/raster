#include <verilated.h>
#include <verilated_fst_c.h>
#include "Vsystem_top.h"
#include "Vsystem_top_system_top.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <optional>
#include <vector>

//=========================================================================
// Q16.16 fixed-point support. The geometry pipeline (process_face) runs
// entirely in fixed point, matching the data formats in the README
// (vertices / matrix / light are Q16.16). Float is used only for input
// preparation: parsing the OBJ text and computing the rotation matrix's
// sin/cos (the matrix is a Q16.16 *input* to the pipeline), both converted
// to fixed point immediately.
//
// Format: Q12.20 in an int32 (32-bit total, matching the README's Q-format
// vertex words). 20 fractional bits give far better precision than Q16.16 for
// the direction / normalize / lighting / 1-z-gradient math (which was dropping
// near-silhouette triangles), while 12 integer bits (+/-2048) leave room for
// the larger magnitudes in the pipeline: screen coords (~320), 1/z (~840) and
// the x255 colour scaling. Multiply/divide use a 64-bit intermediate product
// (as a 32x32 hardware multiplier would) and truncate back to 32 bits.
//=========================================================================
typedef int32_t fx;                       // Q12.20 (32-bit)
static const int FXSH = 12;
static const fx  FX_ONE = 1 << FXSH;

static inline fx  fxf(double d) { return (fx)llround(d * (double)(1 << FXSH)); }
static inline fx  fxi(int i)    { return (fx)(i << FXSH); }
static inline int fxfloor(fx a) { return a >> FXSH; }             // arithmetic
static inline fx  fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> FXSH); }
static inline fx  fdiv(fx a, fx b) { return (fx)(((int64_t)a << FXSH) / b); }

// Round a Q12.20 value to Q11.5 (i.e. real * 32), half away from zero.
static inline int16_t to_q11_5(fx a) {
    int64_t v = (int64_t)a * 32;
    v += (v >= 0) ? (1 << (FXSH - 1)) : -(1 << (FXSH - 1));
    return (int16_t)(v >> FXSH);
}

struct v3 { fx x, y, z; };
struct v4 { fx x, y, z, w; };   // homogeneous: w=FX_ONE for a point, 0 for a direction
struct m4 { fx e[4][4]; };      // row-major 4x4 (Q16.16 in README)
static inline v3 xyz(v4 a) { return {a.x, a.y, a.z}; }

// Convert a glm float vector (host-side input prep) to fixed point.
static inline v3 to_fx3(glm::vec3 v)       { return {fxf(v.x), fxf(v.y), fxf(v.z)}; }
static inline v4 to_fx4(glm::vec3 v, fx w) { return {fxf(v.x), fxf(v.y), fxf(v.z), w}; }

// Dot product in fixed point (used by the per-frame lighting).
static inline fx v3dot(v3 a, v3 b)  { return fmul(a.x,b.x) + fmul(a.y,b.y) + fmul(a.z,b.z); }
// Full homogeneous 4x4 * 4-vector. A point uses w=FX_ONE (so the 4th matrix
// column, the translation, is applied); a direction/normal uses w=0 (so
// translation drops out). With an affine matrix (bottom row [0 0 0 1]) a point
// keeps w=FX_ONE, so no perspective divide is needed.
static inline v4 m4mul(const m4 &M, v4 v) {
    return {
        fmul(M.e[0][0],v.x) + fmul(M.e[0][1],v.y) + fmul(M.e[0][2],v.z) + fmul(M.e[0][3],v.w),
        fmul(M.e[1][0],v.x) + fmul(M.e[1][1],v.y) + fmul(M.e[1][2],v.z) + fmul(M.e[1][3],v.w),
        fmul(M.e[2][0],v.x) + fmul(M.e[2][1],v.y) + fmul(M.e[2][2],v.z) + fmul(M.e[2][3],v.w),
        fmul(M.e[3][0],v.x) + fmul(M.e[3][1],v.y) + fmul(M.e[3][2],v.z) + fmul(M.e[3][3],v.w)
    };
}
static inline fx fxclamp01(fx a) { return a < 0 ? 0 : (a > FX_ONE ? FX_ONE : a); }


static const int W = 320, H = 480;
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
static const int FB_BASE      = 0x30000;

// Geometry-input regions for the HW geometry pass. Placed above the original
// 128K-halfword area (the fake SDRAM is now 256K halfwords), so the inputs are
// persistent and never overlap the framebuffer or the geometry outputs. Bases
// must match the geom_front parameters in system_top and be 8-aligned.
static const int MATRIX_BASE  = 0x20000;   // NUM_FRAMES matrices, 24 hw each
static const int LIGHT_BASE   = 0x20880;   // after the matrix array
static const int VTX_BASE     = 0x21000;   // vertex i at +i*8
static const int FACE_BASE    = 0x22000;   // face f at +f*16

// Each tile gets a fixed-address bucket of MAX_FACES_PER_TILE entries in the
// binlist region. Tile T's bucket starts at BINLIST_BASE + T*MAX_FACES_PER_TILE
// (halfword units). A power of 2 makes the indexing trivial in HW.
static const int MAX_FACES_PER_TILE = 1024;

// Total SDRAM model storage in 16-bit words (must match sdram_model ADDR_BITS).
static const int SDRAM_WORDS = 192 * 1024;  // 384 KB = 192 K halfwords

static uint32_t framebuffer[H][W];

struct Face {
    int v[3];     // vertex indices
    v3  color;    // per-face hue (precomputed from object-space normal), Q12.20 in [0,1]
    v4  normal;   // pre-negated normalized object-space face normal (w=0), Q12.20
};

struct Tri2D {
    int p[3][2];              // screen-space vertices (pixel units, pre-subpixel)
    int16_t iz_at_00;         // 1/z evaluated at screen origin, Q11.5
    int16_t diz_dx;           // 1/z gradient per pixel, X, Q11.5
    int16_t diz_dy;           // 1/z gradient per pixel, Y, Q11.5
    uint32_t color;           // packed 0xRRGGBB
    int bbminx, bbminy, bbmaxx, bbmaxy;
};

std::vector<v4> vertices;
std::vector<Face> faces;

void load_obj(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { printf("ERROR: cannot open %s\n", path); return; }
    std::vector<glm::vec3> gv;   // float vertices from the OBJ (host input prep)
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            glm::vec3 v;
            sscanf(line + 2, "%f %f %f", &v.x, &v.y, &v.z);
            gv.push_back(v);
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
                faces.push_back({{vi[0], vi[1], vi[2]}, {0,0,0}, {0,0,0,0}});
                if (count == 4) faces.push_back({{vi[0], vi[2], vi[3]}, {0,0,0}, {0,0,0,0}});
            }
        }
    }
    fclose(f);

    // Convert the parsed vertices to the pipeline's fixed-point format (points,
    // w=1). Per the README, vertices are a Q-format input.
    for (auto &v : gv)
        vertices.push_back(to_fx4(v, FX_ONE));

    // Precompute the normalized object-space face normal and per-face hue with
    // glm (host-side float math). Storing the normal means the per-frame
    // pipeline only rotates it (rotations preserve length) instead of
    // recomputing cross + normalize, so no fixed-point sqrt is needed at
    // runtime. The normal is *negated* (w=0): the runtime view normal is
    // -R*n_obj (R has det -1 from the Y-flip), so pre-negating lets the pipeline
    // use the transform result directly. Colour uses the unnegated normal.
    for (auto &face : faces) {
        glm::vec3 n_obj = glm::normalize(glm::cross(gv[face.v[1]] - gv[face.v[0]],
                                                    gv[face.v[2]] - gv[face.v[0]]));
        face.normal = to_fx4(-n_obj, 0);
        face.color  = to_fx3(glm::vec3(0.5f) + 0.5f * n_obj);   // n in [-1,1] -> [0,1]
    }

    printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

// Build the model-to-view matrix as a 4x4 affine: upper-left 3x3 is the
// rotation R = Rx * Ry * S (S = diag(1,-1,1), y-flip so screen y increases
// downward), the 4th column is the translation t, and the bottom row is
// [0 0 0 1]. The rotation is prepared with float sin/cos on the host and handed
// to the pipeline as a Q16.16 matrix (per the README, a Q16.16 4x4 input).
// Build the model-to-view matrix as a 4x4 affine: upper-left 3x3 is the
// rotation R = Rx * Ry * S (S = diag(1,-1,1), y-flip so screen y increases
// downward), the 4th column is the translation t, and the bottom row is
// [0 0 0 1]. The rotation is prepared with glm (host float math) and handed to
// the pipeline as a Q-format 4x4 (per the README, a Q16.16 4x4 input). glm is
// column-major, so R[col][row]; we store row-major.
m4 build_view_matrix(float angle_y, float angle_x, v3 t) {
    glm::mat4 g(1.0f);
    g = glm::rotate(g, angle_x, glm::vec3(1, 0, 0));
    g = glm::rotate(g, angle_y, glm::vec3(0, 1, 0));
    g = glm::scale(g, glm::vec3(1, -1, 1));
    glm::mat3 R(g);
    m4 M;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M.e[i][j] = fxf(R[j][i]);          // upper-left 3x3 rotation
    M.e[0][3] = t.x; M.e[1][3] = t.y; M.e[2][3] = t.z;   // translation column
    M.e[3][0] = 0; M.e[3][1] = 0; M.e[3][2] = 0; M.e[3][3] = FX_ONE;  // affine
    return M;
}

// Negated light direction in view space: -normalize(0.2, 0.3, 1.0), computed
// once at startup with glm (host float math). Pre-negating folds the diffuse
// sign in so the pipeline needs no runtime negate: diffuse = dot(-n, L) =
// dot(n, -L).
static const v3 NEG_LIGHT_DIR = to_fx3(-glm::normalize(glm::vec3(0.2f, 0.3f, 1.0f)));

// Process one object-space face all the way to a screen-space Tri2D, or return
// std::nullopt if the face is back-facing. This mirrors the per-triangle dataflow
// a hardware geometry pipeline would use.
std::optional<Tri2D> process_face(const Face &face, const m4 &M) {
    const fx scale = fxi(80);

    // Step 1: object-space -> view-space (rotation + Y-flip + translation).
    // Vertices carry w=1, so m4mul applies the translation column.
    v4 vv[3];
    for (int i = 0; i < 3; i++)
        vv[i] = m4mul(M, vertices[face.v[i]]);

    // Step 2: view-space face normal. Rotations preserve length, so rotating
    // the precomputed unit normal (w=0, so translation drops out) yields a unit
    // view-space normal directly -- no cross product, normalize (sqrt) or negate
    // at runtime. The face normal was pre-negated at load to fold in the det=-1
    // of R's Y-flip, so this matches recomputing it from the transformed edges.
    v3 n = xyz(m4mul(M, face.normal));

    // Step 3: back-face cull (camera looks toward -Z; front faces have n.z < 0)
    if (n.z >= 0) return std::nullopt;

    // Step 4: lighting from the view-space normal. Per-face hue is precomputed
    // at load time from the object-space normal.
    fx diffuse   = v3dot(n, NEG_LIGHT_DIR);
    if (diffuse < 0) diffuse = 0;
    fx intensity = fxf(0.2) + fmul(fxf(0.8), diffuse);
    fx cr = fxclamp01(fmul(intensity, face.color.x));
    fx cg = fxclamp01(fmul(intensity, face.color.y));
    fx cb = fxclamp01(fmul(intensity, face.color.z));
    uint8_t r = (uint8_t)fxfloor(fmul(cr, fxi(255)));
    uint8_t g = (uint8_t)fxfloor(fmul(cg, fxi(255)));
    uint8_t b = (uint8_t)fxfloor(fmul(cb, fxi(255)));

    // Step 5: project view-space vertices to screen-space + 1/z
    int sp[3][2];
    fx  iz[3];
    for (int i = 0; i < 3; i++) {
        sp[i][0] = fxfloor(fmul(vv[i].x, scale) + fxi(W / 2));
        sp[i][1] = fxfloor(fmul(vv[i].y, scale) + fxi(H / 2));
        fx z = fxf(275.0) + fmul(vv[i].z, fxi(225));   // 275 + z*225
        iz[i] = (z > FX_ONE) ? z : FX_ONE;             // max(1, ...)
    }

    // Step 6: assemble Tri2D. Swap winding 1<->2 to undo the orientation
    // inversion caused by the Y-flip in build_view_matrix (OBJ is CCW;
    // rasterizer expects CCW edge functions).
    Tri2D t;
    t.p[0][0] = sp[0][0]; t.p[0][1] = sp[0][1];
    t.p[1][0] = sp[2][0]; t.p[1][1] = sp[2][1];
    t.p[2][0] = sp[1][0]; t.p[2][1] = sp[1][1];
    fx izp[3] = { iz[0], iz[2], iz[1] };
    t.color = (r << 16) | (g << 8) | b;
    t.bbminx = std::min({t.p[0][0], t.p[1][0], t.p[2][0]});
    t.bbminy = std::min({t.p[0][1], t.p[1][1], t.p[2][1]});
    t.bbmaxx = std::max({t.p[0][0], t.p[1][0], t.p[2][0]});
    t.bbmaxy = std::max({t.p[0][1], t.p[1][1], t.p[2][1]});

    // Step 7: 1/z plane setup. Solve for diz_dx, diz_dy from the two edges
    // and extrapolate to screen origin. Result is fixed-point Q11.5.
    int e1x = t.p[1][0] - t.p[0][0], e1y = t.p[1][1] - t.p[0][1];
    int e2x = t.p[2][0] - t.p[0][0], e2y = t.p[2][1] - t.p[0][1];
    int64_t area = (int64_t)e1x * e2y - (int64_t)e1y * e2x;
    fx diz_dx = 0, diz_dy = 0;
    if (area != 0) {
        fx diz1 = izp[1] - izp[0];   // Q16.16
        fx diz2 = izp[2] - izp[0];
        int64_t nx = (int64_t)diz1 * e2y - (int64_t)diz2 * e1y;
        int64_t ny = (int64_t)diz2 * e1x - (int64_t)diz1 * e2x;
        diz_dx = (fx)(nx / area);
        diz_dy = (fx)(ny / area);
    }
    fx iz_at_00 = izp[0]
                - (fx)((int64_t)diz_dx * t.p[0][0])
                - (fx)((int64_t)diz_dy * t.p[0][1]);

    t.iz_at_00 = to_q11_5(iz_at_00);
    t.diz_dx   = to_q11_5(diz_dx);
    t.diz_dy   = to_q11_5(diz_dy);

    return t;
}

// Serialize a fully-prepared Tri2D into the SDRAM tri-record region. All
// geometry (sub-pixel fixup, 1/z plane setup) is done in process_face — this
// is pure DMA-style packing.
void write_tri_record(auto &mem, int tri_id, const Tri2D &t) {
    int base = TRI_BASE + tri_id * 16;
    mem[base + 0] = (uint16_t)(t.p[0][0] * SP + SP/2);
    mem[base + 1] = (uint16_t)(t.p[0][1] * SP + SP/2);
    mem[base + 2] = (uint16_t)(t.p[1][0] * SP + SP/2);
    mem[base + 3] = (uint16_t)(t.p[1][1] * SP + SP/2);
    mem[base + 4] = (uint16_t)(t.p[2][0] * SP + SP/2);
    mem[base + 5] = (uint16_t)(t.p[2][1] * SP + SP/2);
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

// Dump the on-chip input ROM image as a flat $readmemh hex file (one 16-bit
// word per line, address order). This is copied into SDRAM at startup by the
// hardware loader (sdram_loader), in both simulation and on the FPGA.
static const int INPUT_BASE  = 0x20000;   // = MATRIX_BASE (halfwords)
static const int INPUT_WORDS = 23680;     // 0x20000..0x25C80, multiple of 8
void dump_inputs_hex(const char *fn, const std::vector<uint16_t> &rom) {
    FILE *f = fopen(fn, "w");
    if (!f) { printf("ERROR: cannot open %s for write\n", fn); return; }
    fprintf(f, "#include <stdint.h>\nconst uint16_t sdram_inputs[] = {\n");
    for (int i = 0; i < INPUT_WORDS; i++)
        fprintf(f, "  0x%04X,\n", rom[i]);
    fprintf(f, "};");
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
    bool do_trace = false;
    bool emit_inputs_only = false;   // just (re)generate sdram_inputs.hex and exit
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--trace") == 0) do_trace = true;
        if (strcmp(argv[i], "--emit-inputs") == 0) emit_inputs_only = true;
    }

    load_obj("suzanne.obj");
    if (vertices.empty()) return 1;

    printf("Screen: %dx%d, Tile: %dx%d, Grid: %dx%d\n", W, H, TW, TH, NTX, NTY);

    // Number of frames the standalone sim renders (the FPGA runs forever). Kept
    // small: each frame is a full geometry+raster pass, so this is just enough
    // to check the CPU-computed matrix animates correctly.
    const int NUM_FRAMES = 3;

    // Build the on-chip input image (mesh + light) and write it as a C header
    // (sdram_inputs.hex). The CPU firmware #includes it and DMAs it into SDRAM
    // at boot. The per-frame matrices are NO LONGER baked here -- the CPU
    // computes them at runtime -- so the matrix region is left zeroed.
    {
        std::vector<uint16_t> rom(INPUT_WORDS, 0);
        auto w16 = [&](int a, uint16_t v){ rom[a - INPUT_BASE] = v; };
        auto w32 = [&](int a, uint32_t v){ w16(a, v & 0xFFFF); w16(a+1, (v>>16) & 0xFFFF); };
        w32(LIGHT_BASE + 0, (uint32_t)NEG_LIGHT_DIR.x);
        w32(LIGHT_BASE + 2, (uint32_t)NEG_LIGHT_DIR.y);
        w32(LIGHT_BASE + 4, (uint32_t)NEG_LIGHT_DIR.z);
        for (size_t i = 0; i < vertices.size(); i++) {
            w32(VTX_BASE + i*8 + 0, (uint32_t)vertices[i].x);
            w32(VTX_BASE + i*8 + 2, (uint32_t)vertices[i].y);
            w32(VTX_BASE + i*8 + 4, (uint32_t)vertices[i].z);
        }
        for (size_t f = 0; f < faces.size(); f++) {
            int b = FACE_BASE + f*16;
            w16(b+0, faces[f].v[0]);
            w16(b+1, faces[f].v[1]);
            w16(b+2, faces[f].v[2]);
            w32(b+3,  (uint32_t)faces[f].normal.x);
            w32(b+5,  (uint32_t)faces[f].normal.y);
            w32(b+7,  (uint32_t)faces[f].normal.z);
            w32(b+9,  (uint32_t)faces[f].color.x);
            w32(b+11, (uint32_t)faces[f].color.y);
            w32(b+13, (uint32_t)faces[f].color.z);
        }
        dump_inputs_hex("sdram_inputs.hex", rom);
    }

    // In --emit-inputs mode we only (re)generate the header the firmware needs;
    // the CPU ROM (bios.vh) is then built from it before the real sim run.
    if (emit_inputs_only) { printf("Wrote sdram_inputs.hex\n"); return 0; }

    // Construct the DUT (its CPU ROM reads bios.vh via $readmemh at construction).
    Vsystem_top *dut = new Vsystem_top;
    VerilatedFstC *tfp = nullptr;
    if (do_trace) {
        tfp = new VerilatedFstC;
        dut->trace(tfp, 99);
        tfp->open("gpu.fst");
    }
    int sim_time = 0;

    // One clock cycle. Drives display_pix_ce at 1-in-8 (12.5 MHz relative to
    // the 100 MHz sim clock) so the display's SDRAM demand matches the FPGA
    // and doesn't monopolise the bus now that it's the highest-priority master.
    auto tick = [&]() {
        dut->display_pix_ce = ((sim_time >> 1) & 7) == 0;
        dut->clk = 0; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
        dut->clk = 1; dut->eval(); if (tfp) tfp->dump(sim_time); sim_time++;
    };

    // Reset
    dut->rst = 1;
    dut->display_enable = 0;
    dut->display_frame_start = 0;
    tick();
    dut->rst = 0;

    // Wait for SDRAM init (100us = 10000 cycles)
    for (int i = 0; i < 11000; i++) {
        tick();
    }

    // Turn on the mocked display controller after SDRAM is initialized.
    // It will continuously read the FB and apply bus pressure.
    dut->display_enable = 1;

    dut->nfaces = (uint16_t)faces.size();

    for (int frame = 0; frame < NUM_FRAMES; frame++) {
        // Assert start and hold it: the sequencer renders as soon as the CPU
        // has published this frame's matrix (matrix_ready), and bumps
        // frame_count when done. The CPU paces the animation via that handshake.
        dut->start = 1;
        int timeout = 50000000;
        while (!dut->done && timeout-- > 0) {
            tick();
        }
        dut->start = 0;   // drop start so no new render begins during capture
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
            tick();
            idle_streak = dut->sdram_idle ? idle_streak + 1 : 0;
        }
        if (idle_timeout <= 0)
            printf("Frame %03d: SDRAM never went idle\n", frame);

        dut->display_frame_start = 1;
        tick();
        dut->display_frame_start = 0;
        dut->display_enable = 1;

        int captured = 0;
        int cap_timeout = W * H * 16;
        while (captured < W * H && cap_timeout-- > 0) {
            tick();
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
        printf("Frame %03d: captured (%zu faces)\n", frame, faces.size());
    }

    printf("\nDone\n");
    if (tfp) tfp->close();
    delete dut;
    return 0;
}
