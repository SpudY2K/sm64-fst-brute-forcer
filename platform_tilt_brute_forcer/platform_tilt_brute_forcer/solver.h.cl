#ifndef SOLVER_H_CL
#define SOLVER_H_CL

#include "geo.h.cl"
#include "structs.h.cl"

extern constant ushort gReverseArctanTable[65536];
extern constant int

bool test_stick_position(
    int solIdx, int x, int y, float endSpeed, float vel1, float xVel1,
    float zVel1, int angle, int cameraYaw, float* startPosition,
    float* oneUpPlatformPosition, float oneUpPlatformXMin,
    float oneUpPlatformXMax, float oneUpPlatformYMin, float oneUpPlatformYMax,
    float oneUpPlatformZMin, float oneUpPlatformZMax,
    float oneUpPlatformNormalX, float oneUpPlatformNormalY, int f,
    float* frame1Position, float* returnPosition, short (&triangles)[2][3][3],
    float (&normals)[2][3], int d, int q);

int calculate_camera_yaw(
    float* currentPosition, float* lakituPosition);

bool test_one_up_position(
    int solIdx, float* startPosition, float* oneUpPlatformPosition,
    float* returnPosition, float endSpeed, float oneUpPlatformXMin,
    float oneUpPlatformXMax, float oneUpPlatformYMin, float oneUpPlatformYMax,
    float oneUpPlatformZMin, float oneUpPlatformZMax,
    float oneUpPlatformNormalX, float oneUpPlatformNormalY, int f, int d,
    short (&triangles)[2][3][3], float (&normals)[2][3]);

bool find_10k_route(int solIdx, int f, int d, int h);

void find_pu_slide_setup(struct PlatformSolution* sol, int solIdx);

void platform_logic(
    float* platform_normal, float* mario_pos, short (&triangles)[2][3][3],
    float (&normals)[2][3], float (&mat)[4][4]);

#endif