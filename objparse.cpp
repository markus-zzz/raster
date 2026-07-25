#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <vector>

#include "fw/gputypes.h"

typedef int32_t fx; // Q20.12 (32-bit)
static const int FXSH = 12;
static const fx FX_ONE = 1 << FXSH;

static inline fx fxf(double d) { return (fx)llround(d * (double)(1 << FXSH)); }
static inline fx fxi(int i) { return (fx)(i << FXSH); }
static inline int fxfloor(fx a) { return a >> FXSH; } // arithmetic
static inline fx fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> FXSH); }
static inline fx fdiv(fx a, fx b) { return (fx)(((int64_t)a << FXSH) / b); }

// Round a Q12.20 value to Q11.5 (i.e. real * 32), half away from zero.
static inline int16_t to_q11_5(fx a) {
  int64_t v = (int64_t)a * 32;
  v += (v >= 0) ? (1 << (FXSH - 1)) : -(1 << (FXSH - 1));
  return (int16_t)(v >> FXSH);
}

struct v3 {
  fx x, y, z;
};
struct v4 {
  fx x, y, z, w;
}; // homogeneous: w=FX_ONE for a point, 0 for a direction
struct m4 {
  fx e[4][4];
}; // row-major 4x4 (Q16.16 in README)
static inline v3 xyz(v4 a) { return {a.x, a.y, a.z}; }

// Convert a glm float vector (host-side input prep) to fixed point.
static inline v3 to_fx3(glm::vec3 v) { return {fxf(v.x), fxf(v.y), fxf(v.z)}; }
static inline v4 to_fx4(glm::vec3 v, fx w) {
  return {fxf(v.x), fxf(v.y), fxf(v.z), w};
}

std::vector<v4> vertices;
std::vector<GpuFace_t> faces;

void load_obj(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) {
    printf("ERROR: cannot open %s\n", path);
    return;
  }
  std::vector<glm::vec3> gv; // float vertices from the OBJ (host input prep)
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
          while (*p && *p != ' ' && *p != '\n')
            p++;
        } else {
          p++;
        }
      }
      if (count >= 3) {
        faces.push_back({{vi[0], vi[1], vi[2]}, {0, 0, 0}, {0, 0, 0, 0}});
        if (count == 4)
          faces.push_back({{vi[0], vi[2], vi[3]}, {0, 0, 0}, {0, 0, 0, 0}});
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
    face.color = to_fx3(glm::vec3(0.5f) + 0.5f * n_obj); // n in [-1,1] -> [0,1]
  }

  printf("Loaded %zu vertices, %zu triangles\n", vertices.size(), faces.size());
}

int main(int argc, char **argv) {

  load_obj("suzanne.obj");
  if (vertices.empty())
    return 1;

  FILE *fp = fopen("suzanne.h", "w");
  fprintf(fp, "#include \"gputypes.h\"\n");
  fprintf(fp, "const GpuVertex_t suz_vtx[] = {\n");

  for (auto &v : vertices) {
    fprintf(fp, " {0x%x,0x%x,0x%x},\n", v.x, v.y, v.z);
  }
  fprintf(fp, "};\n\n");

  fprintf(fp, "const GpuFace_t suz_face[] = {\n");
  for (auto &f : faces) {
    fprintf(fp, " {%d,%d,%d,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x},\n", f.v[0],
            f.v[1], f.v[2], f.normal.x, f.normal.y, f.normal.z, f.color.x,
            f.color.y, f.color.z);
  }
  fprintf(fp, "};\n");
  fclose(fp);
  return 0;
}
