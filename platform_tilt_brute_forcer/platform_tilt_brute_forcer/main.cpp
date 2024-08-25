#include <cstring>
#include <string>
#include "FST.hpp"

struct SearchOptions {
    std::string outFile = "outData.csv";
    std::string logFile = "";

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
    printf("Z Normal Samples: %d\n", s->nSamplesNXZ);
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
    std::ofstream wf;
    std::ofstream logf;

    if (!parse_inputs(argc, argv, &s, &o)) {
        return -1;
    }

    logf.open(s.logFile);

    if (!s.logFile.empty() && !logf.is_open()) {
        if (!o.silent) fprintf(stderr, "Warning: ofstream is not open. No logging data will be written to the log file.\n");
        if (!o.silent) fprintf(stderr, "         This may be due to an invalid log file path.\n");
    }

    write_line_to_log_file(LOG_INFO, "FST Brute Forcer Started", logf);

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

    initialise_solution_file_stream(wf, s.outFile, &o);

    write_line_to_log_file(LOG_INFO, "Starting Search", logf);

    const float deltaNX = (s.nSamplesNX > 1) ? (s.maxNX - s.minNX) / (s.nSamplesNX - 1) : 0;
    const float deltaNY = (s.nSamplesNY > 1) ? (s.maxNY - s.minNY) / (s.nSamplesNY - 1) : 0;
    const float deltaNXZ = (s.nSamplesNXZ > 1) ? (s.maxNXZ - s.minNXZ) / (s.nSamplesNXZ - 1) : 0;

    for (int j = 0; j < s.nSamplesNXZ; j++) {
        for (int h = 0; h < s.nSamplesNY; h++) {
            printf("Searching: Z=%.10g (%d/%d), Y=%.10g (%d/%d)\n", s.minNXZ + h * deltaNXZ, j + 1, s.nSamplesNXZ, s.minNY + h * deltaNY, h + 1, s.nSamplesNY);
            for (int i = 0; i < s.nSamplesNX; i++) {
                for (int quad = 0; quad < (s.quadMode ? 8 : 1); quad++) {
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

                    if (check_normal(testNormal, &o, &p, wf, logf)) {
                    }
                }
            }
        }
    }

    write_line_to_log_file(LOG_INFO, "Search Completed", logf);

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

    write_line_to_log_file(LOG_INFO, "FST Brute Forcer Finished", logf);

    wf.close();
    logf.close();
}