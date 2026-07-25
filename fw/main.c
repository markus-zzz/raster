#include <stdalign.h>
#include <stdint.h>

#include "fixmath.h"
#include "gputypes.h"
#include "sdram.h"

#include "suzanne.h"

volatile uint32_t *const FRAME_COUNT = (volatile uint32_t *)0x20000000;

// SDRAM layout (halfword addresses). The descriptor list lives in the gap after
// the 24-hw matrix; geom_front's desc_head must match DESC_HEAD.
#define DESC_HEAD 0x20040
// raster output bases (must match gpu_top's fixed TRI/BIN/BINLIST read bases)
#define TRI_BASE 0x00000
#define BIN_BASE 0x04000
#define BINLIST_BASE 0x05000

uint32_t suz_mat_base = 0;
uint32_t suz_vtx_base = 0;
uint32_t suz_face_base = 0;

uint32_t cube_mat_base = 0;
uint32_t cube_vtx_base = 0;
uint32_t cube_face_base = 0;

static void write_matrix(uint32_t ax, uint32_t ay, fx ty) {
  fx cx = fx_cos(ax), sx = fx_sin(ax);
  fx cy = fx_cos(ay), sy = fx_sin(ay);

  GpuMatrix_t gm;
  //  [ cy       0     sy    | 0  ]
  //  [ sx*sy   -cx  -sx*cy  | ty ]   (col 3 = translation; ty bobs view-Y)
  //  [-cx*sy   -sx   cx*cy  | 0  ]
  // clang-format off
  gm.m[0][0] =  cy;            gm.m[0][1] =  0;   gm.m[0][2] =  sy;            gm.m[0][3] = 0;
  gm.m[1][0] =  fmul(sx, sy);  gm.m[1][1] = -cx;  gm.m[1][2] = -fmul(sx, cy);  gm.m[1][3] = ty;
  gm.m[2][0] = -fmul(cx, sy);  gm.m[2][1] = -sx;  gm.m[2][2] =  fmul(cx, cy);  gm.m[2][3] = 0;
  // clang-format on

  sdram_write(suz_mat_base, &gm, sizeof(gm));
}

// ---- second object geometry: a unit cube (model coords +/-1 in Q20.12) ----
static const GpuVertex_t cube_vtx[] = {
  // clang-format off
  {-ONE,-ONE,-ONE},{ ONE,-ONE,-ONE},{ ONE, ONE,-ONE},{-ONE, ONE,-ONE},
  {-ONE,-ONE, ONE},{ ONE,-ONE, ONE},{ ONE, ONE, ONE},{-ONE, ONE, ONE},
  // clang-format on
};
// 12 triangles (2 per face). Winding is CCW-from-outside (same convention as
// the loaded mesh), and the stored normal follows the pipeline convention
// normal = -(v1-v0)x(v2-v0), i.e. the *inward* normal (this folds in the det=-1
// of the matrix's Y-flip so back-face culling keeps the correct faces).
static const GpuFace_t cube_face[] = {
  // clang-format off
  {0,3,2,  0,0, ONE,  ONE,0,0},   {0,2,1,  0,0, ONE,  ONE,0,0},   // -Z red
  {4,5,6,  0,0,-ONE,  0,ONE,0},   {4,6,7,  0,0,-ONE,  0,ONE,0},   // +Z green
  {0,7,3,  ONE,0,0,   0,0,ONE},   {0,4,7,  ONE,0,0,   0,0,ONE},   // -X blue
  {1,2,6, -ONE,0,0,   ONE,ONE,0}, {1,6,5, -ONE,0,0,   ONE,ONE,0}, // +X yellow
  {0,1,5,  0, ONE,0,  0,ONE,ONE}, {0,5,4,  0, ONE,0,  0,ONE,ONE}, // -Y cyan
  {3,6,2,  0,-ONE,0,  ONE,0,ONE}, {3,7,6,  0,-ONE,0,  ONE,0,ONE}, // +Y magenta
  // clang-format on
};

// Write the cube's (static) vertices and faces into SDRAM once.
static void write_cube_geometry(void) {
  sdram_write(cube_vtx_base, cube_vtx, sizeof(cube_vtx));
  sdram_write(cube_face_base, cube_face, sizeof(cube_face));
}

// Orbit + spin parameters (Q20.12 view units; angles are binary 2^32/turn).
#define CUBE_SCALE (1434) // 0.35
#define ORBIT_R (4915)    // 1.2 view units (screen radius ~96 px)
#define ORBIT_RZ (1638)   // 0.4 view units of depth sway

// Build the cube's matrix: small scaled tumble about X and Y, translated onto a
// circle around the head at height ty (so it orbits the head as the head bobs).
static void write_cube_matrix(uint32_t rx, uint32_t ry, uint32_t phi, fx ty) {
  fx cx = fx_cos(rx), sx = fx_sin(rx);
  fx cy = fx_cos(ry), sy = fx_sin(ry), S = CUBE_SCALE;
  fx ox = fmul(ORBIT_R, fx_cos(phi));
  fx oy = ty + fmul(ORBIT_R, fx_sin(phi));
  fx oz = fmul(ORBIT_RZ, fx_cos(phi));
  // 3x3 = S * Rx(rx) * Ry(ry) * diag(1,-1,1)
  GpuMatrix_t gm;
  // clang-format off
  gm.m[0][0] =  fmul(S, cy);           gm.m[0][1] =  0;           gm.m[0][2] =  fmul(S, sy);           gm.m[0][3] = ox;
  gm.m[1][0] =  fmul(S, fmul(sx, sy)); gm.m[1][1] = -fmul(S, cx); gm.m[1][2] = -fmul(S, fmul(sx, cy)); gm.m[1][3] = oy;
  gm.m[2][0] = -fmul(S, fmul(cx, sy)); gm.m[2][1] = -fmul(S, sx); gm.m[2][2] =  fmul(S, fmul(cx, cy)); gm.m[2][3] = oz;
  // clang-format on
  sdram_write(cube_mat_base, &gm, sizeof(gm));
}

// Build the static descriptor list: SET_OUTPUT (bases + light) -> OBJECT (head)
// -> OBJECT (cube) -> end. Matrix contents are refreshed in place each frame.
static void build_descriptors(void) {
  uint32_t gd_set_sdram_addr = DESC_HEAD; // sdram_alloc(sizeof(GeomDesc_t));
  uint32_t gd_obj0_sdram_addr = sdram_alloc(sizeof(GeomDesc_t));
  uint32_t gd_obj1_sdram_addr = sdram_alloc(sizeof(GeomDesc_t));

  GeomDesc_t gd_set = {0};
  gd_set.cmd = GEOM_DESC_CMD_SET;
  gd_set.next = gd_obj0_sdram_addr;
  gd_set.set.tri_base = TRI_BASE;
  gd_set.set.binlist_base = BINLIST_BASE;
  gd_set.set.bin_base = BIN_BASE;
  gd_set.set.light_dir[0] = -771;
  gd_set.set.light_dir[1] = -1156;
  gd_set.set.light_dir[2] = -3853;

  sdram_write(gd_set_sdram_addr, &gd_set, sizeof(gd_set));

  // Suzanne
  GeomDesc_t gd_obj0 = {0};
  gd_obj0.cmd = GEOM_DESC_CMD_OBJ;
  gd_obj0.next = gd_obj1_sdram_addr;
  gd_obj0.obj.vertex_base = suz_vtx_base;
  gd_obj0.obj.faces_base = suz_face_base;
  gd_obj0.obj.matrix_base = suz_mat_base;
  gd_obj0.obj.num_faces = sizeof(suz_face) / sizeof(suz_face[0]);

  sdram_write(gd_obj0_sdram_addr, &gd_obj0, sizeof(gd_obj0));

  // Cube
  GeomDesc_t gd_obj1 = {0};
  gd_obj1.cmd = GEOM_DESC_CMD_OBJ;
  gd_obj1.next = 0;
  gd_obj1.obj.vertex_base = cube_vtx_base;
  gd_obj1.obj.faces_base = cube_face_base;
  gd_obj1.obj.matrix_base = cube_mat_base;
  gd_obj1.obj.num_faces = sizeof(cube_face) / sizeof(cube_face[0]);

  sdram_write(gd_obj1_sdram_addr, &gd_obj1, sizeof(gd_obj1));
}

int main(void) {
  suz_mat_base = sdram_alloc(sizeof(GpuMatrix_t));
  suz_vtx_base = sdram_alloc(sizeof(suz_vtx));
  suz_face_base = sdram_alloc(sizeof(suz_face));
  cube_mat_base = sdram_alloc(sizeof(GpuMatrix_t));
  cube_vtx_base = sdram_alloc(sizeof(cube_vtx));
  cube_face_base = sdram_alloc(sizeof(cube_face));

  sdram_write(suz_vtx_base, suz_vtx, sizeof(suz_vtx));
  sdram_write(suz_face_base, suz_face, sizeof(suz_face));

  // 2) Build the geometry descriptor list once (after the blob load so it is
  //    not clobbered): SET_OUTPUT -> OBJECT(head) -> OBJECT(cube) -> end, and
  //    write the cube's static geometry into SDRAM.
  write_cube_geometry();
  build_descriptors();

  // 3) Per-frame matrix, handshaked with the render sequencer:
  //    write matrix -> signal ready -> wait until frame_count advances.
  const uint32_t AY_INC =
      47721859u >> 1; // 2^32 / 90  (Y: 1 revolution / 90 frames)
  const uint32_t AX_INC =
      95443718u >> 1; // 2^32 / 45  (X: 2 revolutions / 90 frames)
  const uint32_t BOB_INC =
      17895697u; // 2^32 / 240 (up-down cycle every 240 frames)
  const fx BOB_AMP =
      (5 * (1 << FXSH)) / 4; // 1.25 view units (~100 px at scale 80)
  const uint32_t PHI_INC =
      35791394u / 2; // 2^32 / 120 (cube orbits once / 120 frames)
  const uint32_t RX_INC =
      71582788u / 2; // 2^32 / 60  (tumble about X, once / 60 frames)
  const uint32_t RY_INC =
      50529027u / 2; // 2^32 / 85  (tumble about Y, once / 85 frames)
  uint32_t ax = 0, ay = 0, bob = 0, phi = 0, rx = 0, ry = 0;
  uint32_t last = *FRAME_COUNT;

  for (;;) {
    fx ty = fmul(BOB_AMP, fx_sin(bob)); // slow vertical bob
    write_matrix(ax, ay, ty);           // Suzanne head at suz_mat_base
    write_cube_matrix(rx, ry, phi, ty); // orbiting cube tumbling about X and Y
    *FRAME_COUNT = 1;                   // any write -> matrix_ready
    while (*FRAME_COUNT == last) {
    } // wait until this frame is rendered
    last = *FRAME_COUNT;
    ay += AY_INC;
    ax += AX_INC;
    bob += BOB_INC;
    phi += PHI_INC;
    rx += RX_INC;
    ry += RY_INC;
  }
  return 0;
}

uint32_t *irq(uint32_t *regs, uint32_t irqs) { return regs; }
