#pragma once

#include "fixmath.h"

typedef struct __attribute__((packed)) {
  fx m[4][4];
} GpuMatrix_t;

typedef struct __attribute__((packed)) {
  fx x, y, z;
  uint32_t _pad;
} GpuVertex_t;

typedef struct __attribute__((packed)) {
  uint16_t i0, i1, i2;
  fx nx, ny, nz, r, g, b;
  uint16_t _pad;
} GpuFace_t;

typedef struct __attribute__((packed)) {
  enum { GEOM_DESC_CMD_SET = 0, GEOM_DESC_CMD_OBJ = 1 } cmd;
  uint32_t next;
  //
  union {
    struct {
      uint32_t tri_base;
      uint32_t binlist_base;
      uint32_t bin_base;
      uint32_t light_dir[3];
    } set;
    struct {
      uint32_t vertex_base;
      uint32_t faces_base;
      uint32_t matrix_base;
      uint16_t num_faces;
    } obj;
  };
} GeomDesc_t;
