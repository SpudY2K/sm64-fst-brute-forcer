#ifndef CONSTANTS_H_CL
#define CONSTANTS_H_CL

#include "structs.h.cl"

extern const global short default_triangles[2][3][3];
extern const global float normal_offsets[4][3];

constexpr int n_y_ranges       = 1;
extern const global double lower_y[n_y_ranges];
extern const global double upper_y[n_y_ranges];

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

constexpr int total_floorsG = 350;
extern const global SurfaceG floorsG[total_floorsG];

// Precomputed via Python script.
constexpr int magCount = 1168;
extern const global float magSet[magCount];

// The OG SM64 trig tables. Arctan is extended to enable faster searching.
extern const global float gSineTableG[4096];
extern const global float gCosineTableG[4096];
extern const global int gArctanTableG[8192];

// Precomputed via Python script.
extern const global int gReverseArctanTable[65536];

#endif