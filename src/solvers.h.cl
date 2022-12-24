#ifndef SOLVERS_H_CL
#define SOLVERS_H_CL

#include "structs.h.cl"

// All of Spud's massive functions.

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

void find_pu_slide_setup(PlatformSolution* sol, int solIdx);

bool try_pu_xz(
    float* normal, float* position, short (&current_triangles)[2][3][3],
    float (&triangle_normals)[2][3], double x, double z, double nx, double ny,
    double nz, int tilt_idx, int q_steps, double max_speed,
    struct PlatformSolution partialSolution);

bool try_pu_x(
    float* normal, float* position, short (&current_triangles)[2][3][3],
    float (&triangle_normals)[2][3], float (&T_start)[4][4],
    float (&T_tilt)[4][4], double x, double x1_min, double x1_max,
    double x2_min, double x2_max, double platform_min_x, double platform_max_x,
    double platform_min_z, double platform_max_z, double m, double c_min,
    double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps,
    double max_speed, struct PlatformSolution& partialSolution);
    
bool try_pu_z(
    float* normal, float* position, short (&current_triangles)[2][3][3],
    float (&triangle_normals)[2][3], float (&T_start)[4][4],
    float (&T_tilt)[4][4], double z, double z1_min, double z1_max,
    double z2_min, double z2_max, double platform_min_x, double platform_max_x,
    double platform_min_z, double platform_max_z, double m, double c_min,
    double c_max, double nx, double ny, double nz, int tilt_idx, int q_steps,
    double max_speed, struct PlatformSolution& partialSolution);

void try_normal(
    float* normal, float* position, struct PlatformSolution& partialSolution);

void try_position(float* marioPos, float* normal, int maxFrames);


#endif