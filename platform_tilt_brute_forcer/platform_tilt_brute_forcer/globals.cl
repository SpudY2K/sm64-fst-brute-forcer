#include "globals.h.cl"

global float platform_pos[3] = {};
global short startTriangles[2][3][3] = {};
global float startNormals[2][3] = {};
global bool squishCeilings[4] = {};

global PlatformSolution platSolutions[MAX_PLAT_SOLUTIONS] = {};
global int nPlatSolutions = 0;

global PUSolution puSolutions[MAX_PU_SOLUTIONS] = {};
global int nPUSolutions = 0;

global TenKSolution tenKSolutions[MAX_10K_SOLUTIONS] = {};
global int n10KSolutions = 0;