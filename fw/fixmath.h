#pragma once

#include <stdint.h>

#include "sintab.h" // quarter-wave sin LUT (Q12.20); full sine via fx_sin

#define FXSH 12
#define ONE (1 << FXSH) // 1.0 in Q20.12

typedef int32_t fx; // Q20.12

static inline fx fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> FXSH); }

// Binary angle: full turn = 2^32, so wrap is free. sin/cos via the 1024-entry
// full-period LUT with linear interpolation on the low bits.
static inline fx fx_sin(uint32_t a) {
  uint32_t quad = a >> 30;     // quadrant 0..3
  uint32_t x = a & 0x3FFFFFFF; // angle within the quadrant
  if (quad & 1)
    x = 0x40000000u - x;     // quadrants 1,3: reflect about pi/2
  uint32_t idx = x >> 22;    // quarter-table index 0..256
  uint32_t f = x & 0x3FFFFF; // Q22 fraction between samples
  fx s0 = sinq[idx];
  fx s1 = sinq[idx + 1]; // guard entries keep this in bounds
  fx s = s0 + (fx)(((int64_t)(s1 - s0) * (int32_t)f) >> 22);
  return (quad & 2) ? -s : s; // quadrants 2,3: negate
}
static inline fx fx_cos(uint32_t a) { return fx_sin(a + 0x40000000u); }
