#ifndef GLOBALS_H_CL
#define GLOBALS_H_CL

#include "structs.cl.h"

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