#!/usr/bin/env python3
# Generate the reciprocal seed LUT for recipdiv (Newton-Raphson divider).
# 256 entries indexed by the top 8 fraction bits of the normalized mantissa
# d in [1,2); seed[i] = round(2^23 / d_i) with d_i = 1 + (i+0.5)/256, i.e. 1/d
# in Q.23. Values are <= 2^23 so 24 bits (6 hex digits) each.
FR = 23
N = 256
with open("recip_seed.hex", "w") as f:
    for i in range(N):
        d = 1.0 + (i + 0.5) / N
        v = round((1 << FR) / d) & ((1 << (FR + 1)) - 1)
        f.write("%06x\n" % v)
