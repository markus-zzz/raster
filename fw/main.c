#include <stdalign.h>
#include <stdint.h>

#include "sdram_inputs.hex"   // uint16_t sdram_inputs[] : mesh + light (matrix region zeroed)
#include "sintab.h"           // static const int sintab[1024] : sin(2*pi*k/1024) in Q12.20

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

#define FXSH 20
typedef int32_t fx;   // Q12.20

static inline fx fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> FXSH); }

// Binary angle: full turn = 2^32, so wrap is free. sin/cos via the 1024-entry
// full-period LUT with linear interpolation on the low bits.
static inline fx fx_sin(uint32_t a) {
    uint32_t k = a >> 22;             // table index 0..1023
    uint32_t f = a & 0x3FFFFF;        // Q22 fraction between samples
    fx s0 = sintab[k];
    fx s1 = sintab[(k + 1) & 1023];
    return s0 + (fx)(((int64_t)(s1 - s0) * (int32_t)f) >> 22);
}
static inline fx fx_cos(uint32_t a) { return fx_sin(a + 0x40000000u); }

// SDRAM MMIO / DMA
#define MATRIX_BASE 0x20000
volatile uint32_t *const FRAME_COUNT = (volatile uint32_t *)0x20000000;

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

alignas(64) uint16_t wrbuf[8];
alignas(16) uint16_t mbuf[24];   // 12 x Q12.20 matrix elements, lo/hi per element

static inline void mat_put(int idx, fx v) {
    mbuf[2 * idx]     = (uint16_t)(v & 0xFFFF);
    mbuf[2 * idx + 1] = (uint16_t)((v >> 16) & 0xFFFF);
}

// Build matrix for (ax, ay) into mbuf, then DMA it to MATRIX_BASE (3 bursts).
static void write_matrix(uint32_t ax, uint32_t ay) {
    fx cx = fx_cos(ax), sx = fx_sin(ax);
    fx cy = fx_cos(ay), sy = fx_sin(ay);

    //  [ cy       0     sy    ]
    //  [ sx*sy   -cx  -sx*cy  ]   (col 3 = translation = 0)
    //  [-cx*sy   -sx   cx*cy  ]
    mat_put(0,  cy);            mat_put(1,  0);   mat_put(2,  sy);            mat_put(3,  0);
    mat_put(4,  fmul(sx, sy));  mat_put(5, -cx);  mat_put(6, -fmul(sx, cy));  mat_put(7,  0);
    mat_put(8, -fmul(cx, sy));  mat_put(9, -sx);  mat_put(10, fmul(cx, cy));  mat_put(11, 0);

    sdram_write_8_x_u16((uint32_t)&mbuf[0],  MATRIX_BASE + 0);
    sdram_write_8_x_u16((uint32_t)&mbuf[8],  MATRIX_BASE + 8);
    sdram_write_8_x_u16((uint32_t)&mbuf[16], MATRIX_BASE + 16);
}

int main(void) {
    // 1) One-shot load of the static inputs (mesh + light) into SDRAM.
    for (unsigned i = 0; i < sizeof(sdram_inputs) / sizeof(sdram_inputs[0]); i += 8) {
        for (unsigned j = 0; j < 8; j++) wrbuf[j] = sdram_inputs[i + j];
        sdram_write_8_x_u16((uint32_t)&wrbuf[0], MATRIX_BASE + i);
    }

    // 2) Per-frame matrix, handshaked with the render sequencer:
    //    write matrix -> signal ready -> wait until frame_count advances.
    const uint32_t AY_INC = 47721859u >> 1;   // 2^32 / 90  (Y: 1 revolution / 90 frames)
    const uint32_t AX_INC = 95443718u >> 1;   // 2^32 / 45  (X: 2 revolutions / 90 frames)
    uint32_t ax = 0, ay = 0;
    uint32_t last = *FRAME_COUNT;

    for (;;) {
        write_matrix(ax, ay);
        *FRAME_COUNT = 1;                    // any write -> matrix_ready
        while (*FRAME_COUNT == last) { }     // wait until this frame is rendered
        last = *FRAME_COUNT;
        ay += AY_INC;
        ax += AX_INC;
    }
    return 0;
}

uint32_t *irq(uint32_t *regs, uint32_t irqs) {
    return regs;
}
