#ifndef GLOBALS_H_CL
#define GLOBALS_H_CL

#include "structs.h.cl"

constexpr int MAX_PLAT_SOLUTIONS = 50000;
constexpr int MAX_PU_SOLUTIONS = 50000000;
constexpr int MAX_10K_SOLUTIONS = 200000;

extern global float platform_pos[3];
extern global short startTriangles[2][3][3];
extern global float startNormals[2][3];
extern global bool squishCeilings[4];

extern global PlatformSolution platSolutions[MAX_PLAT_SOLUTIONS];
extern global int nPlatSolutions;

extern global PUSolution puSolutions[MAX_PU_SOLUTIONS];
extern global int nPUSolutions;

extern global TenKSolution tenKSolutions[MAX_10K_SOLUTIONS];
extern global int n10KSolutions;

#endif