#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include "CLLoader.hpp"
#include "structs.h.cl"
#include "vmath.hpp"

struct ArgOptions {
  int nThreads      = 256;
  size_t memorySize = 10000000;

  int nPUFrames = 3;
  int maxFrames = 100;

  float minNX = -0.20f;
  float maxNX = -0.22f;
  float minNZ = 0.39f;
  float maxNZ = 0.41f;
  float minNY = 0.80f;
  float maxNY = 0.80f;

  int nSamplesNX = 101;
  int nSamplesNZ = 101;
  int nSamplesNY = 31;

  float deltaX = 0.5f;
  float deltaZ = 0.5f;

  Vec3f platformPos = {-1945.0f, -3225.0f, -715.0f};

  std::string outFile = "outData.csv";

  bool verbose = false;

  static ArgOptions from_args(int argc, char* argv[]) {
    ArgOptions o;
    for (int i = 1; i < argc; i++) {
      if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
        printf("BitFS Platform Max Tilt Brute Forcer.\n");
        printf("This program accepts the following options:\n\n");
        printf("-f <frames>: Maximum frames of platform tilt considered.\n");
        printf("             Default: %d\n", o.maxFrames);
        printf("-p <frames>: Number of frames of PU movement for 10k glitch\n");
        printf("             Default: %d\n", o.nPUFrames);
        printf(
          "-nx <min_nx> <max_nx> <n_samples>: Inclusive range of x normals to be considered, and the number of normals to sample.\n");
        printf(
          "                                   If min_nx==max_nx then n_samples will be set to 1.\n");
        printf(
          "                                   Default: %g %g %d\n", o.minNX,
          o.maxNX, o.nSamplesNX);
        printf(
          "-nz <min_nz> <max_nz> <n_samples>: Inclusive range of z normals to be considered, and the number of normals to sample.\n");
        printf(
          "                                   If min_nz==max_nz then n_samples will be set to 1.\n");
        printf(
          "                                   Default: %g %g %d\n", o.minNZ,
          o.maxNZ, o.nSamplesNZ);
        printf(
          "-ny <min_ny> <max_ny> <n_samples>: Inclusive range of y normals to be considered, and the number of normals to sample.\n");
        printf(
          "                                   If min_ny==max_ny then n_samples will be set to 1.\n");
        printf(
          "                                   Default: %g %g %d\n", o.minNY,
          o.maxNY, o.nSamplesNY);
        printf(
          "-dx <delta_x>: x coordinate spacing of positions on the platform.\n");
        printf("               Default: %g\n", o.deltaX);
        printf(
          "-dz <delta_z>: z coordinate spacing of positions on the platform.\n");
        printf("               Default: %g\n", o.deltaZ);
        printf(
          "-p <platform_x> <platform_y> <platform_z>: Position of the pyramid platform.\n");
        printf(
          "                                           Default: %g %g %g\n",
          o.platformPos[0], o.platformPos[1], o.platformPos[2]);
        printf("-o: Path to the output file.\n");
        printf("    Default: %s\n", o.outFile.c_str());
        printf(
          "-t <threads>: Number of CUDA threads to assign to the program.\n");
        printf("              Default: %d\n", o.nThreads);
        printf("-m <memory>: Amount of GPU memory to assign to the program.\n");
        printf("             Default: %lu\n", o.memorySize);
        printf(
          "-v: Verbose mode. Prints all parameters used in brute force.\n");
        printf("    Default: off\n");
        printf("-h --help: Prints this text.\n");
        exit(0);
      }
      else if (!strcmp(argv[i], "-f")) {
        o.maxFrames = std::strtol(argv[i + 1], nullptr, 10);

        i += 1;
      }
      else if (!strcmp(argv[i], "-p")) {
        o.nPUFrames = std::strtol(argv[i + 1], nullptr, 10);

        i += 1;
      }
      else if (!strcmp(argv[i], "-t")) {
        o.nThreads = std::strtol(argv[i + 1], nullptr, 10);

        i += 1;
      }
      else if (!strcmp(argv[i], "-m")) {
        o.memorySize = std::strtol(argv[i + 1], nullptr, 10);

        i += 1;
      }
      else if (!strcmp(argv[i], "-nx")) {
        o.minNX = std::strtof(argv[i + 1], nullptr);
        o.maxNX = std::strtof(argv[i + 2], nullptr);

        if (o.minNX == o.maxNX) {
          o.nSamplesNX = 1;
        }
        else {
          o.nSamplesNX = std::strtol(argv[i + 3], nullptr, 10);
        }

        i += 3;
      }
      else if (!strcmp(argv[i], "-nz")) {
        o.minNZ = std::strtof(argv[i + 1], nullptr);
        o.maxNZ = std::strtof(argv[i + 2], nullptr);

        if (o.minNZ == o.maxNZ) {
          o.nSamplesNZ = 1;
        }
        else {
          o.nSamplesNZ = std::strtol(argv[i + 3], nullptr, 10);
        }

        i += 3;
      }
      else if (!strcmp(argv[i], "-ny")) {
        o.minNY = std::strtof(argv[i + 1], nullptr);
        o.maxNY = std::strtof(argv[i + 2], nullptr);

        if (o.minNY == o.maxNY) {
          o.nSamplesNY = 1;
        }
        else {
          o.nSamplesNY = std::strtol(argv[i + 3], nullptr, 10);
        }

        i += 3;
      }
      else if (!strcmp(argv[i], "-dx")) {
        o.deltaX = std::strtof(argv[i + 1], nullptr);
        i += 1;
      }
      else if (!strcmp(argv[i], "-dz")) {
        o.deltaZ = std::strtof(argv[i + 1], nullptr);
        i += 1;
      }
      else if (!strcmp(argv[i], "-p")) {
        o.platformPos[0] = std::strtof(argv[i + 1], nullptr);
        o.platformPos[1] = std::strtof(argv[i + 2], nullptr);
        o.platformPos[2] = std::strtof(argv[i + 3], nullptr);
        i += 3;
      }
      else if (!strcmp(argv[i], "-o")) {
        o.outFile = argv[i + 1];
        i += 1;
      }
      else if (!strcmp(argv[i], "-v")) {
        o.verbose = true;
      }
    }
    return o;
  }
};

int main(int argc, char* argv[]) {
  ArgOptions o = ArgOptions::from_args(argc, argv);

  // Initialize GPU kernel
  CLContext cl;
  // init_reverse_atan, init_mag_set, and initialise_floors handled by
  // pregenerated table
  cl.run_kernel(
    "set_platform_pos", 1, 1, o.platformPos[0], o.platformPos[1],
    o.platformPos[2]);

  // allocate CPU buffers
  auto platSolutionsCPU =
    std::make_unique<PlatformSolution[]>(MAX_PLAT_SOLUTIONS);
  auto puSolutionsCPU   = std::make_unique<PUSolution[]>(MAX_PU_SOLUTIONS);
  auto tenKSolutionsCPU = std::make_unique<TenKSolution[]>(MAX_10K_SOLUTIONS);

  // allocate GPU buffers
  auto host_tris  = std::make_unique<short[]>(18);
  auto host_norms = std::make_unique<short[]>(6);
  auto dev_tris   = cl.alloc<short>(18);
  auto dev_norms  = cl.alloc<float>(6);

  const float deltaNX =
    (o.nSamplesNX > 1) ? (o.maxNX - o.minNX) / (o.nSamplesNX - 1) : 0;
  const float deltaNZ =
    (o.nSamplesNZ > 1) ? (o.maxNZ - o.minNZ) / (o.nSamplesNZ - 1) : 0;
  const float deltaNY =
    (o.nSamplesNY > 1) ? (o.maxNY - o.minNY) / (o.nSamplesNY - 1) : 0;
    
  
}