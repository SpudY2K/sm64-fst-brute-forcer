// Entry points.
// =============

#include "globals.h.cl"
#include "solvers.h.cl"

// CONFIG-RELATED
// ==============

kernel void set_platform_pos(float x, float y, float z) {
  platform_pos[0] = x;
  platform_pos[1] = y;
  platform_pos[2] = z;
}

kernel void set_squish_ceilings(float n0, float n1, float n2, float n3) {
  squishCeilings[0] = n0 > -0.5;
  squishCeilings[1] = n1 > -0.5;
  squishCeilings[2] = n2 > -0.5;
  squishCeilings[3] = n3 > -0.5;
}

kernel void set_start_triangle(
  const global short* tris, const global float* norms) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 3; y++) {
      startTriangles[x][y][0] = tris[9 * x + 3 * y];
      startTriangles[x][y][1] = tris[9 * x + 3 * y + 1];
      startTriangles[x][y][2] = tris[9 * x + 3 * y + 2];
      startNormals[x][y]      = norms[3 * x + y];
    }
  }
}

// ACTUAL GOOD STUFF
// =================

kernel void test_pu_solution() {
  int idx = get_group_id(0) * get_num_groups(0) + get_local_id(0);

  if (idx < 8 * nPUSolutions) {
    int solIdx = idx % nPUSolutions;
    idx        = idx / nPUSolutions;

    int f = idx % 2;
    idx   = idx / 2;

    int d = idx % 2;
    idx   = idx / 2;

    int h = idx;
    find_10k_route(solIdx, f, d, h);
  }
}

kernel void test_plat_solution() {
  int idx = get_group_id(0) * get_num_groups(0) + get_local_id(0);

  if (idx < nPlatSolutions) {
    find_pu_slide_setup(&(platSolutions[idx]), idx);
  }
}

kernel void search_positions(
  float minX, float deltaX, float minZ, float deltaZ, int width, int height,
  float normalX, float normalY, float normalZ, int maxFrames) {
  int idx = get_group_id(0) * get_num_groups(0) + get_local_id(0);

  if (idx < width * height) {
    float marioPos[3] = {
      minX - fmod(minX, deltaX) + deltaX * (idx % width), -2500.0f,
      minZ - fmod(minZ, deltaZ) + deltaZ * (idx / width)};
    float normal[3] = {normalX, normalY, normalZ};

    try_position(marioPos, normal, maxFrames);
  }
}