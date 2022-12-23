// Entry points for configuring global variables.
// =============================================

#include "globals.h.cl"

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

kernel void set_start_triangle(const global short* tris, const global float* norms) {
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 3; y++) {
            startTriangles[x][y][0] = tris[9 * x + 3 * y];
            startTriangles[x][y][1] = tris[9 * x + 3 * y + 1];
            startTriangles[x][y][2] = tris[9 * x + 3 * y + 2];
            startNormals[x][y]      = norms[3 * x + y];
        }
    }
}