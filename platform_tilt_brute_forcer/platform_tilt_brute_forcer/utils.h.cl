#ifndef UTILS_H_CL
#define UTILS_H_CL

#include "structs.h.cl"

bool check_inbounds(const float* mario_pos);

short atan2_lookupG(float z, float x);

short atan2sG(float z, float x);

int find_floor(
    float* pos, short (&triangles)[2][3][3], float (&normals)[2][3],
    float* pheight);
int find_floor(
    float* position, SurfaceG** floor, float& floor_y, SurfaceG* floor_set,
    int n_floor_set);
float find_closest_mag(float target);

int atan2b(double z, double x);

#endif