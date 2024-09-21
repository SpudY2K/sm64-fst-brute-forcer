#include <cstring>
#include <string>
#include <math.h>
#include "FST.hpp"
#include "utils.hpp"

enum CheckpointLoad {
    LOAD_SUCCESSFUL = 0,
    LOAD_FAILED = 1,
    OPEN_FAILED = 2,
    CREATED_NEW = 3,
    CREATED_OVERWRITE = 4,
    WRITE_FAILED = 5,
};

struct CheckpointData {
    int version = 3356;
    int startX = 0;
    int startY = 0;
    int startXZ = 0;
    int startQuad = 0;
    int fullSolutionCount = 0;
    int partialSolutionCount = 0;
    int warningNormalCount = 0;
};

struct SearchOptions {
    std::string outFile = "outData.csv";
    std::string logFile = "";
    std::string checkpointFile = "";

    float minNX = 0.1808f;
    float maxNX = 0.1808f;
    float minNY = 0.87752f;
    float maxNY = 0.87752f;
    float minNXZ = -0.57664f;
    float maxNXZ = -0.57664f;

    int nSamplesNX = 21;
    int nSamplesNXZ = 21;
    int nSamplesNY = 21;

    bool zMode = false;
    bool quadMode = false;

    bool verbose = false;
};

bool try_read_checkpoint(struct CheckpointData* c, struct SearchOptions* s, struct FSTOptions* o, std::ifstream& cf) {
    if (!cf.good()) return false;

    int saveVersion;
    cf.read((char*)(&saveVersion), sizeof saveVersion);
    if (!cf.good()) return false;

    if (c->version == saveVersion) {
        struct CheckpointData newC = (*c);
        struct SearchOptions newS = (*s);
        struct FSTOptions newO = (*o);

        cf.read((char*)(&newC.startX), sizeof newC.startX);
        if (!cf.good()) return false;
        cf.read((char*)(&newC.startY), sizeof newC.startY);
        if (!cf.good()) return false;
        cf.read((char*)(&newC.startXZ), sizeof newC.startXZ);
        if (!cf.good()) return false;
        cf.read((char*)(&newC.startQuad), sizeof newC.startQuad);
        if (!cf.good()) return false;
        if (newC.startX == 0 && newC.startY == 0 && newC.startXZ == 0 && newC.startQuad == 0) return false;
        cf.read((char*)(&newC.fullSolutionCount), sizeof newC.fullSolutionCount);
        if (!cf.good()) return false;
        cf.read((char*)(&newC.partialSolutionCount), sizeof newC.partialSolutionCount);
        if (!cf.good()) return false;
        cf.read((char*)(&newC.warningNormalCount), sizeof newC.warningNormalCount);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.minNX), sizeof newS.minNX);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.maxNX), sizeof newS.maxNX);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.minNY), sizeof newS.minNY);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.maxNY), sizeof newS.maxNY);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.minNXZ), sizeof newS.minNXZ);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.maxNXZ), sizeof newS.maxNXZ);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.nSamplesNX), sizeof newS.nSamplesNX);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.nSamplesNY), sizeof newS.nSamplesNY);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.nSamplesNXZ), sizeof newS.nSamplesNXZ);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.zMode), sizeof newS.zMode);
        if (!cf.good()) return false;
        cf.read((char*)(&newS.quadMode), sizeof newS.quadMode);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.platformPos[0]), sizeof newO.platformPos[0]);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.platformPos[1]), sizeof newO.platformPos[1]);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.platformPos[2]), sizeof newO.platformPos[2]);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.deltaX), sizeof newO.deltaX);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.deltaZ), sizeof newO.deltaZ);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.nPUFrames), sizeof newO.nPUFrames);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.maxFrames), sizeof newO.maxFrames);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.maxSpeed), sizeof newO.maxSpeed);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.maxSlidingSpeed), sizeof newO.maxSlidingSpeed);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.maxSlidingSpeedToPlatform), sizeof newO.maxSlidingSpeedToPlatform);
        if (!cf.good()) return false;
        cf.read((char*)(&newO.outputLevel), sizeof newO.outputLevel);
        if (!cf.good()) return false;
        int stringSize;
        cf.read((char*)(&stringSize), sizeof stringSize);
        if (!cf.good()) return false;
        newS.outFile = "";
        newS.outFile.resize(stringSize);
        cf.read((char*)(newS.outFile.c_str()), stringSize);
        if (!cf.good()) return false;
        cf.read((char*)(&stringSize), sizeof stringSize);
        if (!cf.good()) return false;
        newS.logFile = "";
        newS.logFile.resize(stringSize);
        cf.read((char*)(newS.logFile.c_str()), stringSize);
        if (!cf.good()) return false;

        (*c) = newC;
        (*s) = newS;
        (*o) = newO;

        return true;
    }
    else {
        return false;
    }
}

bool try_write_new_checkpoint(struct CheckpointData* c, struct SearchOptions* s, struct FSTOptions* o, std::ofstream& cf) {
    cf.write((char*)(&c->version), sizeof c->version);
    if (!cf.good()) return false;
    cf.write((char*)(&c->startX), sizeof c->startX);
    if (!cf.good()) return false;
    cf.write((char*)(&c->startY), sizeof c->startY);
    if (!cf.good()) return false;
    cf.write((char*)(&c->startXZ), sizeof c->startXZ);
    if (!cf.good()) return false;
    cf.write((char*)(&c->startQuad), sizeof c->startQuad);
    if (!cf.good()) return false;
    cf.write((char*)(&c->fullSolutionCount), sizeof c->fullSolutionCount);
    if (!cf.good()) return false;
    cf.write((char*)(&c->partialSolutionCount), sizeof c->partialSolutionCount);
    if (!cf.good()) return false;
    cf.write((char*)(&c->warningNormalCount), sizeof c->warningNormalCount);
    if (!cf.good()) return false;
    cf.write((char*)(&s->minNX), sizeof s->minNX);
    if (!cf.good()) return false;
    cf.write((char*)(&s->maxNX), sizeof s->maxNX);
    if (!cf.good()) return false;
    cf.write((char*)(&s->minNY), sizeof s->minNY);
    if (!cf.good()) return false;
    cf.write((char*)(&s->maxNY), sizeof s->maxNY);
    if (!cf.good()) return false;
    cf.write((char*)(&s->minNXZ), sizeof s->minNXZ);
    if (!cf.good()) return false;
    cf.write((char*)(&s->maxNXZ), sizeof s->maxNXZ);
    if (!cf.good()) return false;
    cf.write((char*)(&s->nSamplesNX), sizeof s->nSamplesNX);
    if (!cf.good()) return false;
    cf.write((char*)(&s->nSamplesNY), sizeof s->nSamplesNY);
    if (!cf.good()) return false;
    cf.write((char*)(&s->nSamplesNXZ), sizeof s->nSamplesNXZ);
    if (!cf.good()) return false;
    cf.write((char*)(&s->zMode), sizeof s->zMode);
    if (!cf.good()) return false;
    cf.write((char*)(&s->quadMode), sizeof s->quadMode);
    if (!cf.good()) return false;
    cf.write((char*)(&o->platformPos[0]), sizeof o->platformPos[0]);
    if (!cf.good()) return false;
    cf.write((char*)(&o->platformPos[1]), sizeof o->platformPos[1]);
    if (!cf.good()) return false;
    cf.write((char*)(&o->platformPos[2]), sizeof o->platformPos[2]);
    if (!cf.good()) return false;
    cf.write((char*)(&o->deltaX), sizeof o->deltaX);
    if (!cf.good()) return false;
    cf.write((char*)(&o->deltaZ), sizeof o->deltaZ);
    if (!cf.good()) return false;
    cf.write((char*)(&o->nPUFrames), sizeof o->nPUFrames);
    if (!cf.good()) return false;
    cf.write((char*)(&o->maxFrames), sizeof o->maxFrames);
    if (!cf.good()) return false;
    cf.write((char*)(&o->maxSpeed), sizeof o->maxSpeed);
    if (!cf.good()) return false;
    cf.write((char*)(&o->maxSlidingSpeed), sizeof o->maxSlidingSpeed);
    if (!cf.good()) return false;
    cf.write((char*)(&o->maxSlidingSpeedToPlatform), sizeof o->maxSlidingSpeedToPlatform);
    if (!cf.good()) return false;
    cf.write((char*)(&o->outputLevel), sizeof o->outputLevel);
    if (!cf.good()) return false;
    int stringSize = s->outFile.size();
    cf.write((char*)(&stringSize), sizeof stringSize);
    if (!cf.good()) return false;
    cf.write((char*)(s->outFile.c_str()), stringSize);
    if (!cf.good()) return false;
    stringSize = s->logFile.size();
    cf.write((char*)(&stringSize), sizeof stringSize);
    if (!cf.good()) return false;
    cf.write((char*)(s->logFile.c_str()), stringSize);
    if (!cf.good()) return false;

    return true;
}

CheckpointLoad write_new_checkpoint(struct CheckpointData* c, struct SearchOptions* s, struct FSTOptions* o, CheckpointLoad loadStatus) {
    std::ofstream cfOut(s->checkpointFile, std::ios::out | std::ios::binary | std::ios::trunc);

    if (cfOut.is_open()) {
        if (try_write_new_checkpoint(c, s, o, cfOut)) {
            cfOut.close();

            if (loadStatus == LOAD_FAILED) {
                return CREATED_OVERWRITE;
            }
            else {
                return CREATED_NEW;
            }
        }
        else {
            cfOut.close();
            return WRITE_FAILED;
        }
    }
    else {
        return OPEN_FAILED;
    }
}

CheckpointLoad load_from_checkpoint(struct CheckpointData* c, struct SearchOptions* s, struct FSTOptions* o) {
    std::ifstream cfIn(s->checkpointFile, std::ios::in | std::ios::binary);
    bool fileExists = cfIn.is_open();

    if (fileExists) {
        if (try_read_checkpoint(c, s, o, cfIn)) {
            cfIn.close();
            return LOAD_SUCCESSFUL;
        }
        else {
            cfIn.close();
            return LOAD_FAILED;
        }
    }

    return OPEN_FAILED;
}

bool update_checkpoint(struct CheckpointData* c, std::ofstream& cf) {
    if (cf.is_open()) {
        cf.seekp(sizeof c->version);
        if (!cf.good()) return false;
        cf.write((char*)(&c->startX), sizeof c->startX);
        if (!cf.good()) return false;
        cf.write((char*)(&c->startY), sizeof c->startY);
        if (!cf.good()) return false;
        cf.write((char*)(&c->startXZ), sizeof c->startXZ);
        if (!cf.good()) return false;
        cf.write((char*)(&c->startQuad), sizeof c->startQuad);
        if (!cf.good()) return false;
        cf.write((char*)(&c->fullSolutionCount), sizeof c->fullSolutionCount);
        if (!cf.good()) return false;
        cf.write((char*)(&c->partialSolutionCount), sizeof c->partialSolutionCount);
        if (!cf.good()) return false;
        cf.write((char*)(&c->warningNormalCount), sizeof c->warningNormalCount);
        if (!cf.good()) return false;

        return true;
    }
    else {
        return false;
    }
}

void print_options(struct SearchOptions* s, struct FSTOptions* o) {
    printf("Max Tilt Frames: %d\n", o->maxFrames);
    printf("Off Platform Frames: %d\n", o->nPUFrames);
    printf("X Normal Range: (%g, %g)\n", s->minNX, s->maxNX);
    if (s->zMode) {
        printf("Z Normal Range: (%g, %g)\n", s->minNXZ, s->maxNXZ);
    }
    else {
        printf("XZ Sum Range: (%g, %g)\n", s->minNXZ, s->maxNXZ);
    }
    printf("Y Normal Range: (%g, %g)\n", s->minNY, s->maxNY);
    printf("X Normal Samples: %d\n", s->nSamplesNX);
    if (s->zMode) {
        printf("Z Normal Samples: %d\n", s->nSamplesNXZ);
    }
    else {
        printf("XZ Sum Samples: %d\n", s->nSamplesNXZ);
    }
    printf("Y Normal Samples: %d\n", s->nSamplesNY);
    printf("X Spacing: %g\n", o->deltaX);
    printf("Z Spacing: %g\n", o->deltaZ);
    if (s->quadMode) {
        printf("Quadrant Search: on\n");
    }
    else {
        printf("Platform Position: (%g, %g, %g)\n", o->platformPos[0], o->platformPos[1], o->platformPos[2]);
        printf("Quadrant Search: off\n");
    }
    printf("\n");
}

void write_options_to_log_file(struct SearchOptions* s, struct FSTOptions* o, std::ofstream& logf) {
    char logContent[200];
    sprintf(logContent, "Option - Max_Tilt_Frames = %d", o->maxFrames);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    sprintf(logContent, "Option - Off_Platform_Frames = %d", o->nPUFrames);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    sprintf(logContent, "Option - X_Normal_Range = [%s, %s]", float2string_max(s->minNX, 1).c_str(), float2string_max(s->maxNX, 1).c_str());
    write_line_to_log_file(LOG_INFO, logContent, logf);
    if (s->zMode) {
        sprintf(logContent, "Option - Z_Normal_Range = [%.10g, %.10g]", s->minNXZ, s->maxNXZ);
        write_line_to_log_file(LOG_INFO, logContent, logf);
    }
    else {
        sprintf(logContent, "Option - XZ_Sum_Range = [%s, %s]", float2string_max(s->minNXZ, 1).c_str(), float2string_max(s->maxNXZ, 1).c_str());
        write_line_to_log_file(LOG_INFO, logContent, logf);
    }
    sprintf(logContent, "Option - Y_Normal_Range = [%s, %s]", float2string_max(s->minNY, 1).c_str(), float2string_max(s->maxNY, 1).c_str());
    write_line_to_log_file(LOG_INFO, logContent, logf);
    sprintf(logContent, "Option - X_Normal_Samples = %d", s->nSamplesNX);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    if (s->zMode) {
        sprintf(logContent, "Option - Z_Normal_Samples = %d", s->nSamplesNXZ);
        write_line_to_log_file(LOG_INFO, logContent, logf);
    }
    else {
        sprintf(logContent, "Option - XZ_Sum_Samples = %d", s->nSamplesNXZ);
        write_line_to_log_file(LOG_INFO, logContent, logf);
    }
    sprintf(logContent, "Option - Y_Normal_Samples = %d", s->nSamplesNY);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    sprintf(logContent, "Option - X_Spacing = %.10g", o->deltaX);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    sprintf(logContent, "Option - Z_Spacing = %.10g", o->deltaZ);
    write_line_to_log_file(LOG_INFO, logContent, logf);
    if (s->quadMode) {
        write_line_to_log_file(LOG_INFO, "Option - Quadrant_Search = on", logf);
    }
    else {
        sprintf(logContent, "Option - Platform_Position = (%.10g, %.10g, %.10g)", o->platformPos[0], o->platformPos[1], o->platformPos[2]);
        write_line_to_log_file(LOG_INFO, logContent, logf);
        write_line_to_log_file(LOG_INFO, "Option - Quadrant_Search = off", logf);
    }
}

void print_help_text(struct SearchOptions* s, struct FSTOptions* o) {
    printf("BitFS Final Speed Transfer Brute Forcer.\n");
    printf("A GPU-based brute forcer to search for working setups for the FST step of the BitFS 0xA TAS.\n\n");
    printf("This program accepts the following options:\n\n");
    printf("  Platform normal search settings:\n");
    printf("    -nx <min_nx> <max_nx> <n_samples>\n");
    printf("         Inclusive range of x normals to be considered, and the number of normals to sample.\n");
    printf("         If min_nx==max_nx then n_samples will be set to 1.\n");
    printf("             Default: %g %g %d\n\n", s->minNX, s->maxNX, s->nSamplesNX);
    printf("    -nxz <min_nxz> <max_nxz> <n_samples>\n");
    printf("         Inclusive range of xz sums to be considered, and the number of z normals to sample.\n");
    printf("         If min_nxz==max_nxz then n_samples will be set to 1.\n");
    printf("         Set these values negative to search negative z normals.\n");
    printf("             Default: %g %g %d\n\n", s->minNXZ, s->maxNXZ, s->nSamplesNXZ);
    printf("    -ny <min_ny> <max_ny> <n_samples>\n");
    printf("         Inclusive range of y normals to be considered, and the number of normals to sample.\n");
    printf("         If min_ny==max_ny then n_samples will be set to 1.\n");
    printf("             Default: %g %g %d\n\n", s->minNY, s->maxNY, s->nSamplesNY);
    printf("    -nz\n");
    printf("         Search by z normal instead of xz sum.\n");
    printf("         Ranges supplied with -nxz will be converted to z normal ranges.\n");
    printf("             Default: off\n\n");
    printf("    -q\n");
    printf("         Search all 8 \"quadrants\" simultaneously. Overrides platform position set by -p.\n");
    printf("             Default: off\n\n");
    printf("  Brute forcer settings:\n");
    printf("    -f <frames>\n");
    printf("         Maximum frames of platform tilt considered.\n");
    printf("             Default: %d\n\n", o->maxFrames);
    printf("    -pu <frames>\n");
    printf("         Number of frames of PU movement for 10k PU route.\n");
    printf("         Currently, only 3 frame routes are supported.\n");
    printf("             Default: %d\n\n", o->nPUFrames);
    printf("    -dx <delta_x>\n");
    printf("         x coordinate spacing of positions on the platform.\n");
    printf("             Default: %g\n\n", o->deltaX);
    printf("    -dz <delta_z>\n");
    printf("         z coordinate spacing of positions on the platform.\n");
    printf("             Default: %g\n\n", o->deltaZ);
    printf("    -p <platform_x> <platform_y> <platform_z>\n");
    printf("         Position of the pyramid platform.\n");
    printf("             Default: %g %g %g\n\n", o->platformPos[0], o->platformPos[1], o->platformPos[2]);
    printf("  Output settings:\n");
    printf("    -c <path>\n");
    printf("         Path to the checkpoint file.\n");
    printf("             Default: %s\n\n", s->outFile.c_str());
    printf("    -l <path>\n");
    printf("         Path to the log file.\n");
    printf("             Default: %s\n\n", s->logFile.c_str());
    printf("    -o <path>\n");
    printf("         Path to the output file.\n");
    printf("             Default: %s\n\n", s->outFile.c_str());
    printf("    -m\n");
    printf("         Output mode. The amount of detail provided in the output file.\n");
    printf("           0: Minimal output. Prints all normals with full solutions, along with number of full solutions found.\n");
    printf("           1: Minimal output with partial solutions. Prints all normals with 10k partial solutions or better, along with the latest stage with solutions.\n");
    printf("           2: Full output. Prints all normals with full solutions, along with full details of the setup.\n");
    printf("             Default: %d\n\n", o->outputLevel);
    printf("  GPU settings:\n");
    printf("    -d <device_id>\n");
    printf("         The SYCL device used to run the program.\n");
    printf("             Default: %d\n\n", o->cudaDevice);
    printf("    -t <threads>\n");
    printf("         Size of SYCL work group to assign to the program.\n");
    printf("             Default: %d\n\n", o->nThreads);
    printf("    -lsk1 <n_solutions>\n");
    printf("         Maximum number of phase 1 solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_ONE);
    printf("    -lsk2a <n_solutions>\n");
    printf("         Maximum number of phase 2a solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_TWO_A);
    printf("    -lsk2b <n_solutions>\n");
    printf("         Maximum number of phase 2b solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_TWO_B);
    printf("    -lsk2c <n_solutions>\n");
    printf("         Maximum number of phase 2c solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_TWO_C);
    printf("    -lsk2d <n_solutions>\n");
    printf("         Maximum number of phase 2d solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_TWO_D);
    printf("    -lsk3 <n_solutions>\n");
    printf("         Maximum number of phase 3 solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_THREE);
    printf("    -lsk4 <n_solutions>\n");
    printf("         Maximum number of phase 4 solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_FOUR);
    printf("    -lsk5 <n_solutions>\n");
    printf("         Maximum number of phase 5 solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_FIVE);
    printf("    -lsk6 <n_solutions>\n");
    printf("         Maximum number of phase 6 solutions for 10k setup search.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_PHASE_SIX);
    printf("    -lp <n_solutions>\n");
    printf("         Maximum number of platform tilt solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_PLAT_SOLUTIONS);
    printf("    -lu <n_solutions>\n");
    printf("         Maximum number of upwarp solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_UPWARP_SOLUTIONS);
    printf("    -lsku <n_solutions>\n");
    printf("         Maximum number of slide kick upwarp solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SK_UPWARP_SOLUTIONS);
    printf("    -ls <n_solutions>\n");
    printf("         Maximum number of speed solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SPEED_SOLUTIONS);
    printf("    -l10k <n_solutions>\n");
    printf("         Maximum number of 10k solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_10K_SOLUTIONS);
    printf("    -lsl <n_solutions>\n");
    printf("         Maximum number of slide solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SLIDE_SOLUTIONS);
    printf("    -lbd <n_solutions>\n");
    printf("         Maximum number of breakdance solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_BD_SOLUTIONS);
    printf("    -ld10k <n_solutions>\n");
    printf("         Maximum number of double 10k solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_DOUBLE_10K_SOLUTIONS);
    printf("    -lbp <n_solutions>\n");
    printf("         Maximum number of bully push solutions.\n");
    printf("             Default: %d\n\n", o->limits.MAX_BULLY_PUSH_SOLUTIONS);
    printf("    -lsq <n_squish_spots>\n");
    printf("         Maximum number of squish spots.\n");
    printf("             Default: %d\n\n", o->limits.MAX_SQUISH_SPOTS);
    printf("    -lst <n_strain_setups>\n");
    printf("         Maximum number of strain setups.\n");
    printf("             Default: %d\n\n", o->limits.MAX_STRAIN_SETUPS);
    printf("  Misc settings:\n");
    printf("    -b\n");
    printf("         Disable buffering on stdout and stderr.\n");
    printf("             Default: off\n\n");
    printf("    -v\n");
    printf("         Verbose mode. Prints all parameters used in the brute forcer.\n");
    printf("             Default: off\n\n");
    printf("    -s\n");
    printf("         Silent mode. Suppresses all print statements output by the brute forcer.\n");
    printf("             Default: off\n\n");
    printf("    -h --help\n");
    printf("         Prints this text.\n");
}

bool parse_inputs(int argc, char* argv[], struct SearchOptions* s, struct FSTOptions* o) {
    bool success = true;

    int i;

    try {
        for (i = 1; i < argc; i++) {
            if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
                print_help_text(s, o);
                exit(0);
            }
            else if (!strcmp(argv[i], "-f")) {
                o->maxFrames = std::stoi(argv[i + 1]);

                i += 1;
            }
            else if (!strcmp(argv[i], "-pu")) {
                o->nPUFrames = std::stoi(argv[i + 1]);

                i += 1;
            }
            else if (!strcmp(argv[i], "-d")) {
                o->cudaDevice = std::stoi(argv[i + 1]);

                i += 1;
            }
            else if (!strcmp(argv[i], "-t")) {
                o->nThreads = std::stoi(argv[i + 1]);

                i += 1;
            }
            else if (!strcmp(argv[i], "-nx")) {
                s->minNX = std::stof(argv[i + 1]);
                s->maxNX = std::stof(argv[i + 2]);

                if (s->minNX == s->maxNX) {
                    s->nSamplesNX = 1;
                }
                else {
                    s->nSamplesNX = std::stoi(argv[i + 3]);
                }

                i += 3;
            }
            else if (!strcmp(argv[i], "-nxz")) {
                s->minNXZ = std::stof(argv[i + 1]);
                s->maxNXZ = std::stof(argv[i + 2]);

                if (s->minNXZ == s->maxNXZ) {
                    s->nSamplesNXZ = 1;
                }
                else {
                    s->nSamplesNXZ = std::stoi(argv[i + 3]);
                }

                i += 3;
            }
            else if (!strcmp(argv[i], "-ny")) {
                s->minNY = std::stof(argv[i + 1]);
                s->maxNY = std::stof(argv[i + 2]);

                if (s->minNY == s->maxNY) {
                    s->nSamplesNY = 1;
                }
                else {
                    s->nSamplesNY = std::stoi(argv[i + 3]);
                }

                i += 3;
            }
            else if (!strcmp(argv[i], "-nz")) {
                s->zMode = true;
            }
            else if (!strcmp(argv[i], "-dx")) {
                o->deltaX = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-dz")) {
                o->deltaZ = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-p")) {
                o->platformPos[0] = std::stof(argv[i + 1]);
                o->platformPos[1] = std::stof(argv[i + 2]);
                o->platformPos[2] = std::stof(argv[i + 3]);
                i += 3;
            }
            else if (!strcmp(argv[i], "-q")) {
                s->quadMode = true;
            }
            else if (!strcmp(argv[i], "-o")) {
                s->outFile = argv[i + 1];
                i += 1;
            }
            else if (!strcmp(argv[i], "-l")) {
                s->logFile = argv[i + 1];
                i += 1;
            }
            else if (!strcmp(argv[i], "-c")) {
                s->checkpointFile = argv[i + 1];
                i += 1;
            }
            else if (!strcmp(argv[i], "-m")) {
                o->outputLevel = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk1")) {
                o->limits.MAX_SK_PHASE_ONE = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk2a")) {
                o->limits.MAX_SK_PHASE_TWO_A = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk2b")) {
                o->limits.MAX_SK_PHASE_TWO_B = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk2c")) {
                o->limits.MAX_SK_PHASE_TWO_C = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk2d")) {
                o->limits.MAX_SK_PHASE_TWO_D = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk3")) {
                o->limits.MAX_SK_PHASE_THREE = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk4")) {
                o->limits.MAX_SK_PHASE_FOUR = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk5")) {
                o->limits.MAX_SK_PHASE_FIVE = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsk6")) {
                o->limits.MAX_SK_PHASE_SIX = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lp")) {
                o->limits.MAX_PLAT_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lu")) {
                o->limits.MAX_UPWARP_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsku")) {
                o->limits.MAX_SK_UPWARP_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-ls")) {
                o->limits.MAX_SPEED_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-l10k")) {
                o->limits.MAX_10K_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsl")) {
                o->limits.MAX_SLIDE_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lbd")) {
                o->limits.MAX_BD_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-ld10k")) {
                o->limits.MAX_DOUBLE_10K_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lbp")) {
                o->limits.MAX_BULLY_PUSH_SOLUTIONS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lsq")) {
                o->limits.MAX_SQUISH_SPOTS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-lst")) {
                o->limits.MAX_STRAIN_SETUPS = std::stof(argv[i + 1]);
                i += 1;
            }
            else if (!strcmp(argv[i], "-s")) {
                o->silent = true;
            }
            else if (!strcmp(argv[i], "-v")) {
                s->verbose = true;
            }
            else if (!strcmp(argv[i], "-b")) {
                setbuf(stdout, NULL);
                setbuf(stderr, NULL);
            }
        }
    }
    catch (std::invalid_argument const& ex) {
        fprintf(stderr, "Error: Arguments to option %s are invalid.\n", argv[i]);
        success = false;
    }

    return success;
}

int main(int argc, char* argv[]) {
    struct SearchOptions s;
    struct FSTOptions o;
    struct FSTData p;
    struct CheckpointData c;
    std::ofstream wf;
    std::ofstream logf;
    std::ofstream cf;

    if (!parse_inputs(argc, argv, &s, &o)) {
        return -1;
    }

    CheckpointLoad loadStatus = load_from_checkpoint(&c, &s, &o);
    bool resume = (loadStatus == LOAD_SUCCESSFUL);

    if (resume) {
        logf.open(s.logFile, std::ofstream::app);
    }
    else {
        logf.open(s.logFile, std::ofstream::trunc);
    }

    write_line_to_log_file(LOG_INFO, "FST Brute Forcer Started", logf);

    if (resume) {
        if (!o.silent) printf("Progress restored from checkpoint file.\n");
        write_line_to_log_file(LOG_INFO, "Checkpoint Loaded", logf);
    }

    if (!s.logFile.empty() && !logf.is_open()) {
        if (!o.silent) fprintf(stderr, "Warning: ofstream is not open. No logging data will be written to the log file.\n");
        if (!o.silent) fprintf(stderr, "         This may be due to an invalid log file path.\n");
    }

    s.nSamplesNX = s.minNX == s.maxNX ? 1 : s.nSamplesNX;
    s.nSamplesNY = s.minNY == s.maxNY ? 1 : s.nSamplesNY;
    s.nSamplesNXZ = s.minNXZ == s.maxNXZ ? 1 : s.nSamplesNXZ;
    s.verbose = s.verbose && !o.silent;

    if (o.nPUFrames != 3) {
        if (!o.silent) fprintf(stderr, "Error: This brute forcer currently only supports 3 frame 10k routes. Value selected: %d.", o.nPUFrames);
        write_line_to_log_file(LOG_ERROR, "Invalid Number of Frames Specified for 10K PU Route", logf);
        return 1;
    }

    write_line_to_log_file(LOG_INFO, "Allocating Device Memory", logf);

    int err = initialise_fst_vars(&p, &o, logf);

    if (err != 0) {
        if (!o.silent && (err & 0x2)) fprintf(stderr, "       Run this program with --help for details on how to change internal memory limits.\n");

        return err;
    }

    write_line_to_log_file(LOG_INFO, "Device Memory Allocation Successful", logf);

    if (s.verbose) {
        print_options(&s, &o);
    }

    if (!resume) {
        write_options_to_log_file(&s, &o, logf);
    }

    initialise_solution_file_stream(wf, s.outFile, &o, resume);

    if (!resume) {
        loadStatus = write_new_checkpoint(&c, &s, &o, loadStatus);

        switch (loadStatus) {
        case CREATED_NEW:
            write_line_to_log_file(LOG_INFO, "New Checkpoint File Created", logf);
            break;
        case CREATED_OVERWRITE:
            if (!o.silent) fprintf(stderr, "Warning: Could not read checkpoint file. Search will start from the beginning.\n");;
            write_line_to_log_file(LOG_WARNING, "Could Not Read Checkpoint File - Search Will Start From Beginning", logf);
            break;
        case WRITE_FAILED:
            if (!o.silent) fprintf(stderr, "Warning: Could not write to checkpoint file. Progress from this run cannot be resumed.\n");
            if (!o.silent) fprintf(stderr, "         This may be due to an invalid checkpoint file path.\n");
            write_line_to_log_file(LOG_WARNING, "Could Not Write New Checkpoint File", logf);
            break;
        case OPEN_FAILED:
            if (!s.checkpointFile.empty()) {
                if (!o.silent) fprintf(stderr, "Warning: Could not open checkpoint file. Progress from this run cannot be resumed.\n");
                if (!o.silent) fprintf(stderr, "         This may be due to an invalid checkpoint file path.\n");
                write_line_to_log_file(LOG_WARNING, "Could Not Open Checkpoint File", logf);
            }
            break;
        }
    }

    if (loadStatus == LOAD_SUCCESSFUL || loadStatus == CREATED_NEW || loadStatus == CREATED_OVERWRITE) {
        cf.open(s.checkpointFile, std::ios::out | std::ios::binary | std::ios::in);
    }

    if (resume) {
        write_line_to_log_file(LOG_INFO, "Resuming Search", logf);
    } else {
        write_line_to_log_file(LOG_INFO, "Starting Search", logf);
    }

    const float deltaNX = (s.nSamplesNX > 1) ? (s.maxNX - s.minNX) / (s.nSamplesNX - 1) : 0;
    const float deltaNY = (s.nSamplesNY > 1) ? (s.maxNY - s.minNY) / (s.nSamplesNY - 1) : 0;
    const float deltaNXZ = (s.nSamplesNXZ > 1) ? (s.maxNXZ - s.minNXZ) / (s.nSamplesNXZ - 1) : 0;

    char logContent[200];

    for (int j = c.startXZ; j < s.nSamplesNXZ; j++) {
        c.startXZ = j;
        sprintf(logContent, "Searching - %s = %s (%d/%d)", s.zMode ? "Z" : "XZ", float2string_max(s.minNXZ + j * deltaNXZ, 1).c_str(), j + 1, s.nSamplesNXZ);
        write_line_to_log_file(LOG_INFO, logContent, logf);

        for (int h = c.startY; h < s.nSamplesNY; h++) {
            c.startY = h;
            if (!o.silent) printf("Searching: %s=%.10g (%d/%d), Y=%.10g (%d/%d)\n", s.zMode ? "Z" : "XZ", s.minNXZ + j * deltaNXZ, j + 1, s.nSamplesNXZ, s.minNY + h * deltaNY, h + 1, s.nSamplesNY);
            for (int i = c.startX; i < s.nSamplesNX; i++) {
                c.startX = i;
                for (int quad = c.startQuad; quad < (s.quadMode ? 8 : 1); quad++) {
                    c.startQuad = quad;
                    update_checkpoint(&c, cf);

                    float normX;
                    float normY;
                    float normZ;

                    if (s.quadMode) {
                        float signX = 2.0 * (quad % 2) - 1.0;
                        float signZ = 2.0 * ((quad / 2) % 2) - 1.0;
                        o.platformPos[0] = (quad / 4) == 0 ? -1945.0f : -2866.0f;
                        o.platformPos[1] = -3225.0f;
                        o.platformPos[2] = -715.0f;

                        normX = signX * fabs(s.minNX + i * deltaNX);
                        normY = s.minNY + h * deltaNY;

                        if (s.zMode) {
                            normZ = signZ * (fabs(s.minNXZ + j * deltaNXZ));
                        }
                        else {
                            normZ = signZ * (fabs(s.minNXZ + j * deltaNXZ) - fabs(normX));
                        }
                    }
                    else {
                        normX = s.minNX + i * deltaNX;
                        normY = s.minNY + h * deltaNY;

                        if (s.zMode) {
                            normZ = s.minNXZ + j * deltaNXZ;
                        }
                        else {
                            float normNXZ = s.minNXZ + j * deltaNXZ;
                            float signZ = (normNXZ > 0) - (normNXZ < 0);

                            normZ = signZ * (fabs(normNXZ) - fabs(normX));
                        }
                    }

                    float testNormal[3] = { normX, normY, normZ };

                    FSTOutput output = check_normal(testNormal, &o, &p, wf, logf);

                    if (output.cudaError == 0x0) {
                        if (output.flags & SW_FLAG_ALL) {
                            c.warningNormalCount++;
                        }

                        if (output.bestStage == STAGE_COMPLETE) {
                            c.fullSolutionCount++;
                        }
                        else if (output.bestStage >= STAGE_TEN_K) {
                            c.partialSolutionCount++;
                        }
                    }
                    else {
                        if (output.cudaError == 701) {
                            if (!o.silent) fprintf(stderr, "Error: SYCL work group too large for the number of registers on your device.\n");
                            if (!o.silent) fprintf(stderr, "       Run this program with --help for details on how to change number of threads.\n");
                            sprintf(logContent, "SYCL work group too large");
                            write_line_to_log_file(LOG_ERROR, logContent, logf);
                        }
                        else {
                            if (!o.silent) fprintf(stderr, "Error: Normal (%.10g, %.10g, %.10g) failed with the following error code: %d.\n", testNormal[0], testNormal[1], testNormal[2], output.cudaError);
                            sprintf(logContent, "Normal (%s, %s, %s) failed with error code %d", float2string_max(testNormal[0], 1).c_str(), float2string_max(testNormal[1], 1).c_str(), float2string_max(testNormal[2], 1).c_str(), output.cudaError);
                            write_line_to_log_file(LOG_ERROR, logContent, logf);
                        }

                        return output.cudaError;
                    }
                }

                c.startQuad = 0;
            }

            c.startX = 0;
        }

        c.startY = 0;
    }

    write_line_to_log_file(LOG_INFO, "Search Completed", logf);

    if (!o.silent) printf("\n");
    if (!o.silent) printf("Search found %d normal%s with full solutions and %d normal%s with partial solutions.\n", c.fullSolutionCount, c.fullSolutionCount == 1 ? "" : "s", c.partialSolutionCount, c.partialSolutionCount == 1 ? "" : "s");

    sprintf(logContent, "Search Found %d Normal%s with Full Solutions and %d Normal%s with Partial Solutions", c.fullSolutionCount, c.fullSolutionCount == 1 ? "" : "s", c.partialSolutionCount, c.partialSolutionCount == 1 ? "" : "s");
    write_line_to_log_file(LOG_INFO, logContent, logf);

    if (c.warningNormalCount > 0) {
        if (!o.silent) printf("%d normal%s produced warnings.\n", c.warningNormalCount, c.warningNormalCount == 1 ? "" : "s");

        sprintf(logContent, "%d Normal%s Produced Warnings", c.warningNormalCount, c.warningNormalCount == 1 ? "" : "s");
        write_line_to_log_file(LOG_WARNING, logContent, logf);
    }

    if (!o.silent) printf("\n");

    if (test_device()) {
        if (!o.silent) print_success();
        write_line_to_log_file(LOG_INFO, "SYCL Device Test Successful", logf);
    }
    else {
        if (!o.silent) printf("Error: SYCL device test failed.\nThis run may have encountered an error on a previous normal check.");

        write_line_to_log_file(LOG_ERROR, "SYCL Device Test Failed", logf);
    }

    write_line_to_log_file(LOG_INFO, "Deallocating Device Memory", logf);

    free_fst_vars(&p);

    if (cf.is_open()) {
        cf.close();
        std::remove(s.checkpointFile.c_str());
    }

    write_line_to_log_file(LOG_INFO, "FST Brute Forcer Finished", logf);

    wf.close();
    logf.close();
}