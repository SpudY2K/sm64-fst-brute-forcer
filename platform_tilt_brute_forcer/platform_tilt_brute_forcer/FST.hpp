#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include "vmath.hpp"

struct GPULimits {
    int MAX_UPWARP_SOLUTIONS = 10000000;
    int MAX_PLAT_SOLUTIONS = 200000;

    int MAX_SK_PHASE_ONE = 50000;
    int MAX_SK_PHASE_TWO_A = 50000;
    int MAX_SK_PHASE_TWO_B = 50000;
    int MAX_SK_PHASE_TWO_C = 5000000;
    int MAX_SK_PHASE_TWO_D = 5000000;
    int MAX_SK_PHASE_THREE = 4000000;
    int MAX_SK_PHASE_FOUR = 5000000;
    int MAX_SK_PHASE_FIVE = 5000000;
    int MAX_SK_PHASE_SIX = 200000;

    int MAX_SK_UPWARP_SOLUTIONS = 100000;
    int MAX_SPEED_SOLUTIONS = 100000000;
    int MAX_10K_SOLUTIONS = 500000;
    int MAX_SLIDE_SOLUTIONS = 2000000;
    int MAX_BD_SOLUTIONS = 50000;

    int MAX_DOUBLE_10K_SOLUTIONS = 300000;
    int MAX_BULLY_PUSH_SOLUTIONS = 900000;

    int MAX_SQUISH_SPOTS = 5000;
    int MAX_STRAIN_SETUPS = 500000;
};

struct FSTOptions {
    int nThreads = 256;
    int cudaDevice = 0;

    int nPUFrames = 3;
    int maxFrames = 100;

    float deltaX = 0.5f;
    float deltaZ = 0.5f;

    //Vec3f platformPos = { -1945.0f, -3225.0f, -715.0f };
    Vec3f platformPos = { -2866.0f, -3225.0f, -715.0f };

    double maxSpeed = 6553600.0;

    float maxSlidingSpeed = 20.0f;
    float maxSlidingSpeedToPlatform = 5.0f;

    int outputLevel = 1;
    bool silent = false;

    struct GPULimits limits;
};

struct SolStruct {
    struct SKPhase1* sk1Solutions;
    struct SKPhase2* sk2ASolutions;
    struct SKPhase2* sk2BSolutions;
    struct SKPhase2* sk2CSolutions;
    struct SKPhase2* sk2DSolutions;
    struct SKPhase3* sk3Solutions;
    struct SKPhase4* sk4Solutions;
    struct SKPhase5* sk5Solutions;
    struct SKPhase6* sk6Solutions;
    struct PlatformSolution* platSolutions;
    struct UpwarpSolution* upwarpSolutions;
    struct SKUpwarpSolution* skuwSolutions;
    struct SpeedSolution* speedSolutions;
    struct TenKSolution* tenKSolutions;
    struct DoubleTenKSolution* doubleTenKSolutions;
    struct BullyPushSolution* bullyPushSolutions;
    struct SlideSolution* slideSolutions;
    struct BDSolution* bdSolutions;
};

struct SolCounts {
    int nSK1Solutions;
    int nSK2ASolutions;
    int nSK2BSolutions;
    int nSK2CSolutions;
    int nSK2DSolutions;
    int nSK3Solutions;
    int nSK4Solutions;
    int nSK5Solutions;
    int nSK6Solutions;
    int nPlatSolutions;
    int nUpwarpSolutions;
    int nSKUWSolutions;
    int nSpeedSolutions;
    int n10KSolutions;
    int nDouble10KSolutions;
    int nBullyPushSolutions;
    int nSlideSolutions;
    int nBDSolutions;
    int nFullSolutions;
};

struct FSTData {
    struct StrainSetup* devStrainSetups;
    float* devSquishSpots;
    int* devNSquishSpots;
    int* hostNSquishSpots;
    short* dev_tris;
    float* dev_norms;
    short* host_tris;
    float* host_norms;
    short* dev_ceiling_tris;
    float* dev_ceiling_norms;
    short* host_ceiling_tris;
    float* host_ceiling_norms;
    short* floorPoints;
    short* devFloorPoints;
    int* squishEdges;
    int* devSquishEdges;
    struct SolStruct s;
};

struct StrainSetup {
    float sidewardStrain;
    float forwardStrain;
};

struct SKPhase1 {
    int x1;
    int z1;
    int q2;
    double minSpeed;
    double maxSpeed;
    double minF1Dist;
    double maxF1Dist;
    int minF1AngleIdx;
    int maxF1AngleIdx;
};

struct SKPhase2 {
    int p1Idx;
    int f2Angle;
    int tenKFloorIdx;
    float lower;
    float upper;
    double sinAngle;
    double cosAngle;
};

struct SKPhase3 {
    int p2Idx;
    int p2Type;
    int x2;
    int z2;
};

struct SKPhase4 {
    int p3Idx;
    int cameraYaw;
    double minM1;
    double maxM1;
    double minN1;
    double maxN1;
    float minPre10KSpeed;
    float maxPre10KSpeed;
    float minPost10KSpeed;
    float maxPost10KSpeed;
    double minAngleDiff;
    double maxAngleDiff;
};

struct SKPhase5 {
    int p4Idx;
    int stickX;
    int stickY;
    int f1Angle;
};

struct SKPhase6 {
    int p5Idx;
    float minPre10KSpeed;
    float maxPre10KSpeed;
    float minPost10KSpeed;
    float maxPost10KSpeed;
    int f3Angle;
};

struct UpwarpSolution {
    int platformSolutionIdx;
    int pux;
    int puz;
};

struct PlatformSolution {
    float returnPosition[3];
    float endPosition[3];
    float endNormal[3];
    short endTriangles[2][3][3];
    float endTriangleNormals[2][3];
    int endFloorIdx;
    float landingFloorNormalsY[3];
    float landingPositions[3][3];
    float penultimateFloorNormalY;
    float penultimatePosition[3];
    int nFrames;
};

struct SKUpwarpSolution {
    int skIdx;
    int uwIdx;
    float minSpeed;
    float maxSpeed;
    float speedRange;
    float xVelRange;
    float zVelRange;
};

struct SpeedSolution {
    int skuwSolutionIdx;
    float returnSpeed;
    float forwardStrain;
    float xStrain;
    float zStrain;
};

struct TenKSolution {
    int speedSolutionIdx;
    float departureSpeed;
    float pre10KVel[2];
    float returnVel[2];
    float startPosition[2][3];
    float frame1Position[3];
    float frame2Position[3];
    int minStartAngle;
    int maxStartAngle;
    double minEndAngle;
    double maxEndAngle;
    double minM1;
    double maxM1;
    int squishCeiling;
    int bdSetups;
    int bpSetups;
};

struct DoubleTenKSolution {
    int tenKSolutionIdx;
    float post10KXVel;
    float post10KZVel;
    float minStartX;
    float maxStartX;
    float minStartZ;
    float maxStartZ;
};

struct BullyPushSolution {
    int doubleTenKSolutionIdx;
    float bullyMinX;
    float bullyMaxX;
    float bullyMinZ;
    float bullyMaxZ;
    int pushAngle;
    int squishPushQF;
    float squishPushMinX;
    float squishPushMaxX;
    float squishPushMinZ;
    float squishPushMaxZ;
    float maxSpeed;
    float minSlidingSpeedX;
    float minSlidingSpeedZ;
};

struct SlideSolution {
    int tenKSolutionIdx;
    float preUpwarpPosition[3];
    float upwarpPosition[3];
    int angle;
    float stickMag;
    int intendedDYaw;
    int postSlideAngle;
    float postSlideSpeed;
};

struct BDSolution {
    int slideSolutionIdx;
    float landingPosition[3];
    int cameraYaw;
    int stickX;
    int stickY;
    float postSlideSpeed;
};

enum SolutionStage {
    STAGE_NOTHING = 0,
    STAGE_PLATFORM = 1,
    STAGE_UPWARP = 2,
    STAGE_SLIDE_KICK = 3,
    STAGE_SKUW = 4,
    STAGE_SPEED = 5,
    STAGE_TEN_K = 6,
    STAGE_SLIDE = 7,
    STAGE_BREAKDANCE = 8,
    STAGE_DOUBLE_TEN_K = 9,
    STAGE_BULLY_PUSH = 10,
    STAGE_COMPLETE = 11
};

enum LogType {
    LOG_INFO = 'I',
    LOG_WARNING = 'W',
    LOG_ERROR = 'E'
};

void initialise_solution_file_stream(std::ofstream& wf, std::string outPath, struct FSTOptions* o);
int initialise_fst_vars(struct FSTData* p, struct FSTOptions* o, std::ofstream& logf);
void copy_solution_counts_to_cpu(struct SolCounts* countsCPU);
void copy_solutions_to_cpu(struct FSTData* p, struct SolStruct* solutionsCPU, struct SolCounts* countsCPU);
void write_solution_file_header(int outputLevel, std::ofstream& wf); void write_solutions_to_file(float* startNormal, struct FSTOptions* o, struct FSTData* p, struct SolStruct* solutionsCPU, struct SolCounts* countsCPU, int floorIdx, std::ofstream& wf);
void write_line_to_log_file(LogType type, std::string content, std::ofstream& logf);
SolutionStage check_normal(float* startNormal, struct FSTOptions* o, struct FSTData* p, std::ofstream& wf);
SolutionStage check_normal(float* startNormal, struct FSTOptions* o, struct FSTData* p, std::ofstream& wf, std::ofstream& logf);
void free_fst_vars(struct FSTData* p);
void free_solution_pointers_cpu(SolStruct* s);
void free_solution_pointers_gpu(SolStruct* s);
void print_success();
bool test_device();