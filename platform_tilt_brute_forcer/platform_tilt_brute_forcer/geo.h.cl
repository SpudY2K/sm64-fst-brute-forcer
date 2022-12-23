#ifndef GEO_H_CL
#define GEO_H_CL

#include "structs.h.cl"

constexpr int total_floorsG = 350;
extern global const SurfaceG floorsG[total_floorsG];

extern global float platform_pos[3];
extern global short startTriangles[2][3][3];
extern global float startNormals[2][3];
extern global bool squishCeilings[4];

constexpr float oneUpPlatformNormalYLeft = 0.351623833;
constexpr float oneUpPlatformNormalXLeft = -0.9361413717;
constexpr float oneUpPlatformYMinLeft    = -3071.0f;
constexpr float oneUpPlatformYMaxLeft    = -2661.0f;
constexpr float oneUpPlatformZMinLeft    = -613.0f;
constexpr float oneUpPlatformZMaxLeft    = -306.0f;
constexpr float oneUpPlatformXMinLeft    = -4607.0f;
constexpr float oneUpPlatformXMaxLeft    = -4453.0f;

constexpr float oneUpPlatformNormalYRight = 0.349620432;
constexpr float oneUpPlatformNormalXRight = 0.936891377;
constexpr float oneUpPlatformYMinRight    = -2661.0f;
constexpr float oneUpPlatformYMaxRight    = -3071.0f;
constexpr float oneUpPlatformZMinRight    = -613.0f;
constexpr float oneUpPlatformZMaxRight    = -306.0f;
constexpr float oneUpPlatformXMinRight    = -4146.0f;
constexpr float oneUpPlatformXMaxRight    = -3993.0f;

inline bool check_inbounds(const float* mario_pos) {
    short x_mod = (short) (int) mario_pos[0];
    short y_mod = (short) (int) mario_pos[1];
    short z_mod = (short) (int) mario_pos[2];

    return (abs(x_mod) < 8192 & abs(y_mod) < 8192 & abs(z_mod) < 8192);
}

inline void set_squish_ceilings(float n0, float n1, float n2, float n3) {
    squishCeilings[0] = n0 > -0.5;
    squishCeilings[1] = n1 > -0.5;
    squishCeilings[2] = n2 > -0.5;
    squishCeilings[3] = n3 > -0.5;
}

inline void set_platform_pos(float x, float y, float z) {
    platform_pos[0] = x;
    platform_pos[1] = y;
    platform_pos[2] = z;
}

inline int find_floor(
    float* position, SurfaceG** floor, float& floor_y, SurfaceG floor_set[],
    int n_floor_set) {
    short x = (short) (int) position[0];
    short y = (short) (int) position[1];
    short z = (short) (int) position[2];

    int floor_idx = -1;

    for (int i = 0; i < n_floor_set; ++i) {
        if (x < floor_set[i].min_x || x > floor_set[i].max_x ||
            z < floor_set[i].min_z || z > floor_set[i].max_z) {
            continue;
        }

        if ((floor_set[i].vertices[0][2] -
             z) * (floor_set[i].vertices[1][0] - floor_set[i].vertices[0][0]) -
                (floor_set[i].vertices[0][0] - x) *
                    (floor_set[i].vertices[1][2] -
                     floor_set[i].vertices[0][2]) <
            0) {
            continue;
        }
        if ((floor_set[i].vertices[1][2] -
             z) * (floor_set[i].vertices[2][0] - floor_set[i].vertices[1][0]) -
                (floor_set[i].vertices[1][0] - x) *
                    (floor_set[i].vertices[2][2] -
                     floor_set[i].vertices[1][2]) <
            0) {
            continue;
        }
        if ((floor_set[i].vertices[2][2] -
             z) * (floor_set[i].vertices[0][0] - floor_set[i].vertices[2][0]) -
                (floor_set[i].vertices[2][0] - x) *
                    (floor_set[i].vertices[0][2] -
                     floor_set[i].vertices[2][2]) <
            0) {
            continue;
        }

        float height =
            -(x * floor_set[i].normal[0] + floor_set[i].normal[2] * z +
              floor_set[i].origin_offset) /
            floor_set[i].normal[1];

        if (y - (height + -78.0f) < 0.0f) {
            continue;
        }

        floor_y   = height;
        *floor    = &floor_set[i];
        floor_idx = i;
        break;
    }

    return floor_idx;
}

#endif