#include <stdalign.h>
#include <stdint.h>

#include "sdram_inputs.hex"   // uint16_t sdram_inputs[] : mesh + light (matrix region zeroed)
#include "sintab.h"           // quarter-wave sin LUT (Q12.20); full sine via fx_sin

//=========================================================================
// Per-frame view matrix, computed on the CPU in Q12.20 fixed point and written
// into SDRAM at MATRIX_BASE (0x20000). Replaces the 90 pre-baked matrices: the
// geometry engine always reads the single matrix at MATRIX_BASE, and the CPU
// refreshes it every frame via a handshake with the render sequencer.
//
//   matrix = Rx(ax) * Ry(ay) * diag(1,-1,1)   (matches host build_view_matrix)
//
// Stored as the top 3 rows of a 4x4 affine, row-major, 12 x Q12.20 (24 hw),
// translation column = 0 -- the exact layout the loader used for a baked matrix.
//=========================================================================

#define FXSH 12
typedef int32_t fx;   // Q20.12

static inline fx fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> FXSH); }

// Binary angle: full turn = 2^32, so wrap is free. sin/cos via the 1024-entry
// full-period LUT with linear interpolation on the low bits.
static inline fx fx_sin(uint32_t a) {
    uint32_t quad = a >> 30;              // quadrant 0..3
    uint32_t x    = a & 0x3FFFFFFF;       // angle within the quadrant
    if (quad & 1) x = 0x40000000u - x;    // quadrants 1,3: reflect about pi/2
    uint32_t idx = x >> 22;               // quarter-table index 0..256
    uint32_t f   = x & 0x3FFFFF;          // Q22 fraction between samples
    fx s0 = sinq[idx];
    fx s1 = sinq[idx + 1];                // guard entries keep this in bounds
    fx s  = s0 + (fx)(((int64_t)(s1 - s0) * (int32_t)f) >> 22);
    return (quad & 2) ? -s : s;           // quadrants 2,3: negate
}
static inline fx fx_cos(uint32_t a) { return fx_sin(a + 0x40000000u); }

// SDRAM MMIO / DMA
#define MATRIX_BASE 0x20000
volatile uint32_t *const FRAME_COUNT = (volatile uint32_t *)0x20000000;

// SDRAM layout (halfword addresses). The descriptor list lives in the gap after
// the 24-hw matrix; geom_front's desc_head must match DESC_HEAD.
#define DESC_HEAD     0x20040
#define VTX_BASE      0x21000
#define FACE_BASE     0x22000
// raster output bases (must match gpu_top's fixed TRI/BIN/BINLIST read bases)
#define TRI_BASE      0x00000
#define BIN_BASE      0x04000
#define BINLIST_BASE  0x05000
// negated light dir within the sdram_inputs blob (blob base 0x20000, light 0x20880)
#define LIGHT_OFF_HW  0x880
#define GEOM_CMD_SET  0
#define GEOM_CMD_OBJ  1

// Second object: a small cube that orbits the Suzanne head. Its matrix is
// rebuilt per frame; its geometry (below) is written to SDRAM once. Placed
// after the mesh blob (which ends ~0x25C80) and the framebuffer is at 0x30000.
#define CUBE_MAT_BASE  0x20080   // cube matrix (24 hw), in the gap before light
#define CUBE_VTX_BASE  0x26000
#define CUBE_FACE_BASE 0x26200
#define CUBE_NFACES    12

// Custom PCPI op: DMA 8 x u16 (= 4 words at src_ram_addr) into SDRAM at
// dst_sdram_addr (halfword address). src must be 16-byte aligned.
static inline void sdram_write_8_x_u16(uint32_t src_ram_addr, uint32_t dst_sdram_addr) {
    __asm__ __volatile__ (
        // .insn r opcode, funct3, funct7, rd, rs1, rs2
        ".insn r 0x0b, 1, 0x22, x0, %0, %1"
        : : "r" (src_ram_addr), "r" (dst_sdram_addr) : "memory"
    );
}

void *memcpy(void *dst, const void *src, unsigned n) {
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    for (unsigned i = 0; i < n; i++) d[i] = s[i];
    return dst;
}

void *memset(void *dst, int c, unsigned n) {
    uint8_t *d = (uint8_t *)dst;
    for (unsigned i = 0; i < n; i++) d[i] = (uint8_t)c;
    return dst;
}

alignas(64) uint16_t wrbuf[8];
alignas(16) uint16_t mbuf[24];   // 12 x Q12.20 matrix elements, lo/hi per element

static inline void mat_put(int idx, fx v) {
    mbuf[2 * idx]     = (uint16_t)(v & 0xFFFF);
    mbuf[2 * idx + 1] = (uint16_t)((v >> 16) & 0xFFFF);
}

// Build matrix for (ax, ay) with vertical translation ty (view units) into
// mbuf, then DMA it to MATRIX_BASE (3 bursts).
static void write_matrix(uint32_t ax, uint32_t ay, fx ty) {
    fx cx = fx_cos(ax), sx = fx_sin(ax);
    fx cy = fx_cos(ay), sy = fx_sin(ay);

    //  [ cy       0     sy    | 0  ]
    //  [ sx*sy   -cx  -sx*cy  | ty ]   (col 3 = translation; ty bobs view-Y)
    //  [-cx*sy   -sx   cx*cy  | 0  ]
    mat_put(0,  cy);            mat_put(1,  0);   mat_put(2,  sy);            mat_put(3,  0);
    mat_put(4,  fmul(sx, sy));  mat_put(5, -cx);  mat_put(6, -fmul(sx, cy));  mat_put(7,  ty);
    mat_put(8, -fmul(cx, sy));  mat_put(9, -sx);  mat_put(10, fmul(cx, cy));  mat_put(11, 0);

    sdram_write_8_x_u16((uint32_t)&mbuf[0],  MATRIX_BASE + 0);
    sdram_write_8_x_u16((uint32_t)&mbuf[8],  MATRIX_BASE + 8);
    sdram_write_8_x_u16((uint32_t)&mbuf[16], MATRIX_BASE + 16);
}

alignas(16) uint16_t dbuf[16];   // one descriptor = 16 hw (2 bursts)

static inline void d_put32(int hw, uint32_t v) {
    dbuf[hw]     = (uint16_t)(v & 0xFFFF);
    dbuf[hw + 1] = (uint16_t)((v >> 16) & 0xFFFF);
}
static inline void desc_flush(uint32_t dst_hw) {   // DMA 16 hw = 2 bursts
    sdram_write_8_x_u16((uint32_t)&dbuf[0], dst_hw + 0);
    sdram_write_8_x_u16((uint32_t)&dbuf[8], dst_hw + 8);
}

// ---- second object geometry: a unit cube (model coords +/-1 in Q20.12) ----
#define CH   (1 << FXSH)          // cube half extent = 1.0
#define ONE  (1 << FXSH)          // 1.0 in Q20.12 (unit normal / full colour)
static const fx cube_vtx[8][3] = {
    {-CH,-CH,-CH},{ CH,-CH,-CH},{ CH, CH,-CH},{-CH, CH,-CH},
    {-CH,-CH, CH},{ CH,-CH, CH},{ CH, CH, CH},{-CH, CH, CH},
};
// 12 triangles (2 per face). Winding is CCW-from-outside (same convention as
// the loaded mesh), and the stored normal follows the pipeline convention
// normal = -(v1-v0)x(v2-v0), i.e. the *inward* normal (this folds in the det=-1
// of the matrix's Y-flip so back-face culling keeps the correct faces).
static const struct { uint16_t i0, i1, i2; fx nx, ny, nz, r, g, b; } cube_face[12] = {
    {0,3,2,  0,0, ONE,  ONE,0,0},   {0,2,1,  0,0, ONE,  ONE,0,0},   // -Z red
    {4,5,6,  0,0,-ONE,  0,ONE,0},   {4,6,7,  0,0,-ONE,  0,ONE,0},   // +Z green
    {0,7,3,  ONE,0,0,   0,0,ONE},   {0,4,7,  ONE,0,0,   0,0,ONE},   // -X blue
    {1,2,6, -ONE,0,0,   ONE,ONE,0}, {1,6,5, -ONE,0,0,   ONE,ONE,0}, // +X yellow
    {0,1,5,  0, ONE,0,  0,ONE,ONE}, {0,5,4,  0, ONE,0,  0,ONE,ONE}, // -Y cyan
    {3,6,2,  0,-ONE,0,  ONE,0,ONE}, {3,7,6,  0,-ONE,0,  ONE,0,ONE}, // +Y magenta
};

// Write the cube's (static) vertices and faces into SDRAM once.
static void write_cube_geometry(void) {
    for (int i = 0; i < 8; i++) {
        d_put32(0, (uint32_t)cube_vtx[i][0]);
        d_put32(2, (uint32_t)cube_vtx[i][1]);
        d_put32(4, (uint32_t)cube_vtx[i][2]);
        dbuf[6] = 0; dbuf[7] = 0;
        sdram_write_8_x_u16((uint32_t)&dbuf[0], CUBE_VTX_BASE + i * 8);
    }
    for (int f = 0; f < 12; f++) {
        dbuf[0] = cube_face[f].i0; dbuf[1] = cube_face[f].i1; dbuf[2] = cube_face[f].i2;
        d_put32(3, (uint32_t)cube_face[f].nx); d_put32(5, (uint32_t)cube_face[f].ny);
        d_put32(7, (uint32_t)cube_face[f].nz);
        d_put32(9, (uint32_t)cube_face[f].r);  d_put32(11, (uint32_t)cube_face[f].g);
        d_put32(13, (uint32_t)cube_face[f].b); dbuf[15] = 0;
        desc_flush(CUBE_FACE_BASE + f * 16);   // 16 hw = 2 bursts
    }
}

// Orbit + spin parameters (Q20.12 view units; angles are binary 2^32/turn).
#define CUBE_SCALE (1434)           // 0.35
#define ORBIT_R    (4915)           // 1.2 view units (screen radius ~96 px)
#define ORBIT_RZ   (1638)           // 0.4 view units of depth sway

// Build the cube's matrix: small scaled tumble about X and Y, translated onto a
// circle around the head at height ty (so it orbits the head as the head bobs).
static void write_cube_matrix(uint32_t rx, uint32_t ry, uint32_t phi, fx ty) {
    fx cx = fx_cos(rx), sx = fx_sin(rx);
    fx cy = fx_cos(ry), sy = fx_sin(ry), S = CUBE_SCALE;
    fx ox = fmul(ORBIT_R, fx_cos(phi));
    fx oy = ty + fmul(ORBIT_R, fx_sin(phi));
    fx oz =       fmul(ORBIT_RZ, fx_cos(phi));
    // 3x3 = S * Rx(rx) * Ry(ry) * diag(1,-1,1)
    mat_put(0,  fmul(S, cy));               mat_put(1,  0);           mat_put(2,  fmul(S, sy));               mat_put(3,  ox);
    mat_put(4,  fmul(S, fmul(sx, sy)));     mat_put(5, -fmul(S, cx)); mat_put(6, -fmul(S, fmul(sx, cy)));     mat_put(7,  oy);
    mat_put(8, -fmul(S, fmul(cx, sy)));     mat_put(9, -fmul(S, sx)); mat_put(10, fmul(S, fmul(cx, cy)));     mat_put(11, oz);
    sdram_write_8_x_u16((uint32_t)&mbuf[0],  CUBE_MAT_BASE + 0);
    sdram_write_8_x_u16((uint32_t)&mbuf[8],  CUBE_MAT_BASE + 8);
    sdram_write_8_x_u16((uint32_t)&mbuf[16], CUBE_MAT_BASE + 16);
}

// Build the static descriptor list: SET_OUTPUT (bases + light) -> OBJECT (head)
// -> OBJECT (cube) -> end. Matrix contents are refreshed in place each frame.
static void build_descriptors(void) {
    // SET_OUTPUT @ DESC_HEAD
    d_put32(0, GEOM_CMD_SET);
    d_put32(2, DESC_HEAD + 16);                 // next -> OBJECT (head)
    d_put32(4, TRI_BASE);
    d_put32(6, BINLIST_BASE);
    d_put32(8, BIN_BASE);
    for (int k = 0; k < 6; k++)                 // inline negated light dir (3 x u32)
        dbuf[10 + k] = sdram_inputs[LIGHT_OFF_HW + k];
    desc_flush(DESC_HEAD);
    // OBJECT (head) @ DESC_HEAD+16
    d_put32(0, GEOM_CMD_OBJ);
    d_put32(2, DESC_HEAD + 32);                 // next -> OBJECT (cube)
    d_put32(4, VTX_BASE);
    d_put32(6, FACE_BASE);
    d_put32(8, MATRIX_BASE);
    dbuf[10] = (uint16_t)SDRAM_NUM_FACES;
    for (int k = 11; k < 16; k++) dbuf[k] = 0;
    desc_flush(DESC_HEAD + 16);
    // OBJECT (cube) @ DESC_HEAD+32
    d_put32(0, GEOM_CMD_OBJ);
    d_put32(2, 0);                              // next = null (end of list)
    d_put32(4, CUBE_VTX_BASE);
    d_put32(6, CUBE_FACE_BASE);
    d_put32(8, CUBE_MAT_BASE);
    dbuf[10] = (uint16_t)CUBE_NFACES;
    for (int k = 11; k < 16; k++) dbuf[k] = 0;
    desc_flush(DESC_HEAD + 32);
}

int main(void) {
    // 1) One-shot load of the static inputs (mesh + light) into SDRAM.
    for (unsigned i = 0; i < sizeof(sdram_inputs) / sizeof(sdram_inputs[0]); i += 8) {
        for (unsigned j = 0; j < 8; j++) wrbuf[j] = sdram_inputs[i + j];
        sdram_write_8_x_u16((uint32_t)&wrbuf[0], MATRIX_BASE + i);
    }

    // 2) Build the geometry descriptor list once (after the blob load so it is
    //    not clobbered): SET_OUTPUT -> OBJECT(head) -> OBJECT(cube) -> end, and
    //    write the cube's static geometry into SDRAM.
    write_cube_geometry();
    build_descriptors();

    // 3) Per-frame matrix, handshaked with the render sequencer:
    //    write matrix -> signal ready -> wait until frame_count advances.
    const uint32_t AY_INC = 47721859u >> 1;   // 2^32 / 90  (Y: 1 revolution / 90 frames)
    const uint32_t AX_INC = 95443718u >> 1;   // 2^32 / 45  (X: 2 revolutions / 90 frames)
    const uint32_t BOB_INC = 17895697u;       // 2^32 / 240 (up-down cycle every 240 frames)
    const fx       BOB_AMP = (5 * (1 << FXSH)) / 4;  // 1.25 view units (~100 px at scale 80)
    const uint32_t PHI_INC  = 35791394u / 2;      // 2^32 / 120 (cube orbits once / 120 frames)
    const uint32_t RX_INC   = 71582788u / 2;      // 2^32 / 60  (tumble about X, once / 60 frames)
    const uint32_t RY_INC   = 50529027u / 2;      // 2^32 / 85  (tumble about Y, once / 85 frames)
    uint32_t ax = 0, ay = 0, bob = 0, phi = 0, rx = 0, ry = 0;
    uint32_t last = *FRAME_COUNT;

    for (;;) {
        fx ty = fmul(BOB_AMP, fx_sin(bob));  // slow vertical bob
        write_matrix(ax, ay, ty);            // Suzanne head at MATRIX_BASE
        write_cube_matrix(rx, ry, phi, ty);  // orbiting cube tumbling about X and Y
        *FRAME_COUNT = 1;                    // any write -> matrix_ready
        while (*FRAME_COUNT == last) { }     // wait until this frame is rendered
        last = *FRAME_COUNT;
        ay  += AY_INC;
        ax  += AX_INC;
        bob += BOB_INC;
        phi += PHI_INC;
        rx  += RX_INC;
        ry  += RY_INC;
    }
    return 0;
}

uint32_t *irq(uint32_t *regs, uint32_t irqs) {
    return regs;
}
