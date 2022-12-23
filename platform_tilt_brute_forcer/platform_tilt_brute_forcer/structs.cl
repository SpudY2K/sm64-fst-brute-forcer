#include "structs.h.cl"

constant float normal_offsets[4][3] = {
    {0.01f, -0.01f, 0.01f},
    {-0.01f, -0.01f, 0.01f},
    {-0.01f, -0.01f, -0.01f},
    {0.01f, -0.01f, -0.01f},
};

constant short default_triangles[2][3][3] = {
    {{307, 307, -306}, {-306, 307, -306}, {-306, 307, 307}},
    {{307, 307, -306}, {-306, 307, 307}, {307, 307, 307}},
};

global struct TenKSolution tenKSolutions[MAX_10K_SOLUTIONS] = {};
global int nPlatSolutions = 0;
global struct PUSolution puSolutions[MAX_PU_SOLUTIONS] = {};
global int nPUSolutions = 0;
global struct TenKSolution tenKSolutions[MAX_10K_SOLUTIONS] = {};
global int n10KSolutions = 0;
