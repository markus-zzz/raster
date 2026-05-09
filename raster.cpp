#include <cstdint>
#include <cstdio>
#include <algorithm>

static const int W = 320, H = 240;
static uint32_t fb[H][W];

struct Vec2 { int x, y; };

// Edge function: sign tells which side of the edge (v0->v1) point p is on.
// E(p) = (v1-v0) x (p-v0)  (2D cross product / signed area)
static int edge_func(Vec2 v0, Vec2 v1, Vec2 p) {
    return (v1.x - v0.x) * (p.y - v0.y) - (v1.y - v0.y) * (p.x - v0.x);
}

// Incremental deltas for stepping one pixel in x or y
struct EdgeStep {
    int val;   // current edge function value
    int dx;    // delta when stepping +1 in x
    int dy;    // delta when stepping +1 in y
};

static EdgeStep make_edge(Vec2 v0, Vec2 v1, Vec2 origin) {
    return {
        edge_func(v0, v1, origin),
        -(v1.y - v0.y),   // d/dx of edge func
         (v1.x - v0.x),   // d/dy of edge func
    };
}

void draw_triangle(Vec2 v0, Vec2 v1, Vec2 v2, uint32_t color) {
    // Bounding box clamped to screen
    int minx = std::max(0,   std::min({v0.x, v1.x, v2.x}));
    int miny = std::max(0,   std::min({v0.y, v1.y, v2.y}));
    int maxx = std::min(W-1, std::max({v0.x, v1.x, v2.x}));
    int maxy = std::min(H-1, std::max({v0.y, v1.y, v2.y}));

    // Align bounding box to 2x2 quad grid
    minx &= ~1; miny &= ~1;

    Vec2 origin = {minx, miny};
    EdgeStep e0 = make_edge(v0, v1, origin);
    EdgeStep e1 = make_edge(v1, v2, origin);
    EdgeStep e2 = make_edge(v2, v0, origin);

    // Row-start values (top-left of each quad row)
    int row0 = e0.val, row1 = e1.val, row2 = e2.val;

    for (int qy = miny; qy <= maxy; qy += 2) {
        // Column-start values for this quad row
        int col0 = row0, col1 = row1, col2 = row2;

        for (int qx = minx; qx <= maxx; qx += 2) {
            // Evaluate all 4 pixels of the 2x2 quad incrementally
            // p00=top-left, p10=top-right, p01=bot-left, p11=bot-right
            int a0 = col0,          a1 = col1,          a2 = col2;
            int b0 = col0 + e0.dx,  b1 = col1 + e1.dx,  b2 = col2 + e2.dx;
            int c0 = col0 + e0.dy,  c1 = col1 + e1.dy,  c2 = col2 + e2.dy;
            int d0 = b0   + e0.dy,  d1 = b1   + e1.dy,  d2 = b2   + e2.dy;

            // Inside test: all three edge functions >= 0
            if (qy   <= maxy && qx   <= maxx && a0>=0 && a1>=0 && a2>=0) fb[qy  ][qx  ] = color;
            if (qy   <= maxy && qx+1 <= maxx && b0>=0 && b1>=0 && b2>=0) fb[qy  ][qx+1] = color;
            if (qy+1 <= maxy && qx   <= maxx && c0>=0 && c1>=0 && c2>=0) fb[qy+1][qx  ] = color;
            if (qy+1 <= maxy && qx+1 <= maxx && d0>=0 && d1>=0 && d2>=0) fb[qy+1][qx+1] = color;

            // Step one quad to the right
            col0 += e0.dx * 2;
            col1 += e1.dx * 2;
            col2 += e2.dx * 2;
        }

        // Step one quad down
        row0 += e0.dy * 2;
        row1 += e1.dy * 2;
        row2 += e2.dy * 2;
    }
}

int main() {
    // Clear to black
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            fb[y][x] = 0;

    // Draw a couple of flat-shaded triangles
    draw_triangle({10, 10}, {200, 30}, {100, 180}, 0xFF0000); // red
    draw_triangle({150, 50}, {310, 80}, {230, 190}, 0x00FF00); // green
    draw_triangle({50, 100}, {160, 80}, {80, 190}, 0x0000FF); // blue

    // Write PPM
    FILE *f = fopen("out.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            uint32_t c = fb[y][x];
            uint8_t rgb[3] = {(uint8_t)(c>>16), (uint8_t)(c>>8), (uint8_t)c};
            fwrite(rgb, 1, 3, f);
        }
    fclose(f);
    printf("Wrote out.ppm\n");
}
