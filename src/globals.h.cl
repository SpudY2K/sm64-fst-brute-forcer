#ifndef GLOBALS_H_CL
#define GLOBALS_H_CL

#include "structs.h.cl"



inline global float platform_pos[3] = {};
inline global short startTriangles[2][3][3] = {};
inline global float startNormals[2][3] = {};
inline global bool squishCeilings[4] = {};

inline global PlatformSolution* platSolutions;
inline global int* nPlatSolutions;

inline global PUSolution* puSolutions;
inline global int* nPUSolutions;

inline global TenKSolution* tenKSolutions;
inline global int* n10KSolutions;

#endif