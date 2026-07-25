#pragma once

#include <stdint.h>

void *memcpy(void *dst, const void *src, unsigned n) {
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;
  for (unsigned i = 0; i < n; i++)
    d[i] = s[i];
  return dst;
}

void *memset(void *dst, int c, unsigned n) {
  uint8_t *d = (uint8_t *)dst;
  for (unsigned i = 0; i < n; i++)
    d[i] = (uint8_t)c;
  return dst;
}

// Custom PCPI op: DMA 8 x u16 (= 4 words at src_ram_addr) into SDRAM at
// dst_sdram_addr (halfword address). src must be 16-byte aligned.
static inline void sdram_write_8_x_u16(uint32_t src_ram_addr,
                                       uint32_t dst_sdram_addr) {
  __asm__ __volatile__(
      // .insn r opcode, funct3, funct7, rd, rs1, rs2
      ".insn r 0x0b, 1, 0x22, x0, %0, %1"
      :
      : "r"(src_ram_addr), "r"(dst_sdram_addr)
      : "memory");
}

static void sdram_write(uint32_t sdram_dst, const void *ram_src,
                        uint32_t size) {
  uint16_t __attribute__((aligned(16))) ram_buf[8];
  for (uint32_t off = 0; off < size; off += 16) {
    memcpy(ram_buf, ram_src + off, 16);
    sdram_write_8_x_u16((uint32_t)ram_buf, sdram_dst + (off >> 1));
  }
}

// Keep in mind that SDRAM is word addressed
uint32_t sdram_alloc(uint32_t size) {
  static uint32_t sdram_alloc_next = 0x20100;
  uint32_t tmp = sdram_alloc_next;
  sdram_alloc_next += ((size + 7) & ~7); // Everything is multiple of 8 words
  return tmp;
}
