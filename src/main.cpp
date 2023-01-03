#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include "CLLoader.hpp"
#include "Platform.hpp"
#include "SolutionWriter.hpp"
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

  int check() {
    if (nPUFrames != 3) {
      fprintf(
        stderr,
        "Error: This brute forcer currently only supports 3 frame 10k routes. Value selected: %d.",
        nPUFrames);
      return 1;
    }
    if (verbose) {
      printf("Max Frames: %d\n", maxFrames);
      printf("Off Platform Frames: %d\n", nPUFrames);
      printf("X Normal Range: (%g, %g)\n", minNX, maxNX);
      printf("Z Normal Range: (%g, %g)\n", minNZ, maxNZ);
      printf("Y Normal Range: (%g, %g)\n", minNY, maxNY);
      printf("X Normal Samples: %d\n", nSamplesNX);
      printf("Z Normal Samples: %d\n", nSamplesNZ);
      printf("Y Normal Samples: %d\n", nSamplesNY);
      printf("X Spacing: %g\n", deltaX);
      printf("Z Spacing: %g\n", deltaZ);
      printf(
        "Platform Position: (%g, %g, %g)\n", platformPos[0], platformPos[1],
        platformPos[2]);
      printf("\n");
    }

    return 0;
  }
};

struct HostParams {
  // CPU buffers
  std::unique_ptr<PlatformSolution[]> platSolutionsCPU;
  std::unique_ptr<PUSolution[]> puSolutionsCPU;
  std::unique_ptr<TenKSolution[]> tenKSolutionsCPU;

  std::unique_ptr<int16_t[]> host_tris;
  std::unique_ptr<float[]> host_norms;

  // GPU buffers
  cl::Buffer platSolutions;
  cl::Buffer puSolutions;
  cl::Buffer tenKSolutions;

  cl::Buffer solnCounters;

  cl::Buffer dev_tris;
  cl::Buffer dev_norms;

  // loop parameters
  const float deltaNX;
  const float deltaNY;
  const float deltaNZ;

  // Misc values
  SolutionWriter solnWriter;
  std::unordered_map<uint64_t, PUSolution> puSolutionLookup;
};

void copyPlatVectors(const Platform& platform, short* tris, float* norms) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 3; y++) {
      tris[9 * x + 3 * y]     = platform.triangles[x].vectors[y][0];
      tris[9 * x + 3 * y + 1] = platform.triangles[x].vectors[y][1];
      tris[9 * x + 3 * y + 2] = platform.triangles[x].vectors[y][2];
      norms[3 * x + y]        = platform.triangles[x].normal[y];
    }
  }
}

std::tuple<float, float, float, float> platBoundingBox(
  const Platform& platform) {
  short minX = std::numeric_limits<int16_t>::max();
  short maxX = std::numeric_limits<int16_t>::min();
  short minZ = std::numeric_limits<int16_t>::max();
  short maxZ = std::numeric_limits<int16_t>::min();

  for (uint k = 0; k < platform.triangles.size(); k++) {
    minX = std::min(
      {minX, platform.triangles[k].vectors[0][0],
       platform.triangles[k].vectors[1][0],
       platform.triangles[k].vectors[2][0]});
    maxX = std::max(
      {minX, platform.triangles[k].vectors[0][0],
       platform.triangles[k].vectors[1][0],
       platform.triangles[k].vectors[2][0]});
    minZ = std::min(
      {minX, platform.triangles[k].vectors[0][2],
       platform.triangles[k].vectors[1][2],
       platform.triangles[k].vectors[2][2]});
    maxZ = std::max(
      {minX, platform.triangles[k].vectors[0][2],
       platform.triangles[k].vectors[1][2],
       platform.triangles[k].vectors[2][2]});
  }

  return {minX, maxX, minZ, maxZ};
}

void testNormal(
  CLContext& cl, ArgOptions& o, HostParams& h, const Vec3f& startNormal,
  const Vec3ul& index) {
  Platform platform(
    o.platformPos[0], o.platformPos[1], o.platformPos[2], startNormal);

  if (!platform.can_squish())
    return;

  cl.run_kernel_once(
    "set_squish_ceilings", platform.ceilings[0].normal[1],
    platform.ceilings[1].normal[1], platform.ceilings[2].normal[1],
    platform.ceilings[3].normal[1]);
  Vec3f pos {0.0f, 0.0f, 0.0f};

  // allow the platform to untilt for nPUFrames
  platform.platform_logic(pos);
  // copy the platform vectors at the point of PU platform displacement
  copyPlatVectors(platform, h.host_tris.get(), h.host_norms.get());
  for (uint k = 1; k < o.nPUFrames; k++) {
    platform.platform_logic(pos);
  }

  // find platform floor's horizontal bounding box
  auto [minX, maxX, minZ, maxZ] = platBoundingBox(platform);

  int nX = round((maxX - minX) / o.deltaX) + 1;
  int nZ = round((maxZ - minZ) / o.deltaZ) + 1;

  if (nX * nZ > o.memorySize) {
    printf(
      "Warning: GPU buffer too small for normal (%g, %g), skipping.\n",
      startNormal[0], startNormal[2]);
    return;
  }

  // FIND PLATFORM SOLUTIONS
  // =======================

  int nPlatSolutionsCPU = 0;
  // copy counter from CPU to GPU
  cl.write_buffer(
    h.solnCounters, &nPlatSolutionsCPU, sizeof(int),
    PLAT_COUNTER_OFF * sizeof(int));

  cl.run_kernel(
    "search_positions", {nX * nZ + o.nThreads - 1}, {o.nThreads}, minX,
    o.deltaX, minZ, o.deltaZ, nX, nZ, platform.normal[0], platform.normal[1],
    platform.normal[2], o.maxFrames);
  // copy counter from GPU to CPU
  cl.read_buffer(
    h.solnCounters, &nPlatSolutionsCPU, sizeof(int),
    PLAT_COUNTER_OFF * sizeof(int));

  // skip if there are no good platform solutions
  if (nPlatSolutionsCPU <= 0) {
    fprintf(
      stderr, "Note: No platform solutions for normal (%g, %g, %g)",
      startNormal[0], startNormal[1], startNormal[2]);
    return;
  }

  if (nPlatSolutionsCPU >= MAX_PLAT_SOLUTIONS) {
    fputs(
      "Warning: Maximum number of platform solutions found. "
      "Increase the internal maximum to get more solutions.",
      stderr);
    nPlatSolutionsCPU = MAX_PLAT_SOLUTIONS;
  }

  // FIND PU SOLUTIONS
  // =======================

  int nPUSolutionsCPU = 0;

  cl.write_buffer(
    h.solnCounters, &nPUSolutionsCPU, sizeof(int),
    PU_COUNTER_OFF * sizeof(int));

  cl.run_kernel(
    "test_plat_solution", {nPlatSolutionsCPU}, {o.nThreads});

  cl.read_buffer(
    h.solnCounters, &nPUSolutionsCPU, sizeof(int),
    PU_COUNTER_OFF * sizeof(int));

  // skip if there are no good PU solutions
  if (nPUSolutionsCPU <= 0) {
    fprintf(
      stderr, "Note: No PU solutions for normal (%g, %g, %g)", startNormal[0],
      startNormal[1], startNormal[2]);
    return;
  }

  if (nPUSolutionsCPU >= MAX_PU_SOLUTIONS) {
    fputs(
      "Warning: Maximum number of PU solutions found. "
      "Increase the internal maximum to get more solutions.",
      stderr);
    nPUSolutionsCPU = MAX_PU_SOLUTIONS;
  }

  // DEDUPLICATE ON CPU
  // TODO replace with GPU algorithm
  // ===============================

  h.puSolutionLookup.clear();

  // Copy PU solutions from GPU to CPU
  cl.read_buffer(
    h.puSolutions, h.puSolutionsCPU.get(),
    nPUSolutionsCPU * sizeof(PUSolution));

  // insert all values into hashmap
  for (size_t n = 0; n < nPUSolutionsCPU; n++) {
    // derive hash key from platform index and return speed
    uint64_t key =
      (static_cast<uint64_t>(h.puSolutionsCPU[n].platformSolutionIdx) << 32) |
      std::bit_cast<uint32_t>(h.puSolutionsCPU[n].returnSpeed);

    h.puSolutionLookup.emplace(key, h.puSolutionLookup[n]);
  }

  // Copy values from hashmap to CPU buffer
  nPUSolutionsCPU = 0;
  for (const auto& [key, soln] : h.puSolutionLookup) {
    h.puSolutionsCPU[nPUSolutionsCPU] = soln;
    nPUSolutionsCPU++;
  }

  // Copy PU solutions and counter back to GPU
  cl.write_buffer(
    h.solnCounters, &nPUSolutionsCPU, sizeof(int),
    PU_COUNTER_OFF * sizeof(int));
  cl.write_buffer(
    h.puSolutions, h.puSolutionsCPU.get(),
    nPUSolutionsCPU * sizeof(PUSolution));

  // FIND 10K SOLUTIONS
  // ==================

  printf(
    "---------------------------------------\n"
    "Testing Normal: %g, %g, %g\n"
    "        Index: %zu, %zu, %zu\n"
    "        # Platform Solutions: %d\n"
    "        # PU Solutions: %d\n",
    startNormal[0], startNormal[1], startNormal[2], index[0], index[0],
    index[0], nPlatSolutionsCPU, nPUSolutionsCPU);
  
  int n10KSolutionsCPU = 0;
  // Copy counter from CPU to GPU
  cl.write_buffer(
    h.solnCounters, &n10KSolutionsCPU, sizeof(int),
    TENK_COUNTER_OFF * sizeof(int));
    
  cl.run_kernel("test_pu_solution", {8 * nPUSolutionsCPU}, {o.nThreads});
  // Copy counter from GPU to CPU
  cl.read_buffer(
    h.solnCounters, &n10KSolutionsCPU, sizeof(int),
    TENK_COUNTER_OFF * sizeof(int));
  
  if (n10KSolutionsCPU <= 0) {
    fprintf(
      stderr, "Note: No 10K solutions for normal (%g, %g, %g)", startNormal[0],
      startNormal[1], startNormal[2]);
    return;
  }
  
  // skip if there are no good 10k solutions
  if (n10KSolutionsCPU >= MAX_10K_SOLUTIONS) {
    fputs(
      "Warning: Maximum number of 10k solutions found. "
      "Increase the internal maximum to get more solutions.",
      stderr);
    n10KSolutionsCPU = MAX_10K_SOLUTIONS;
  }
  
  
  cl.read_buffer(h.tenKSolutions, h.tenKSolutionsCPU.get(), n10KSolutionsCPU * sizeof(TenKSolution));
  cl.read_buffer(h.platSolutions, h.platSolutionsCPU.get(), n10KSolutionsCPU * sizeof(PlatformSolution));
  
  for (size_t i = 0; i < n10KSolutionsCPU; i++) {
    auto& tenK = h.tenKSolutionsCPU[i];
    auto& pu = h.puSolutionsCPU[tenK.puSolutionIdx];
    auto& plat = h.platSolutionsCPU[pu.platformSolutionIdx];
    
    h.solnWriter.write_soln(tenK, pu, plat, h.host_norms.get(), startNormal[0], startNormal[1], startNormal[2]);
  }
}

int main(int argc, char* argv[]) {
  ArgOptions o = ArgOptions::from_args(argc, argv);

  if (int res = o.check())
    return res;

  // Initialize GPU kernel
  CLContext cl;

  HostParams q {
    .platSolutionsCPU =
      std::make_unique<PlatformSolution[]>(MAX_PLAT_SOLUTIONS),
    .puSolutionsCPU   = std::make_unique<PUSolution[]>(MAX_PU_SOLUTIONS),
    .tenKSolutionsCPU = std::make_unique<TenKSolution[]>(MAX_10K_SOLUTIONS),

    .host_tris  = std::make_unique<int16_t[]>(18),
    .host_norms = std::make_unique<float[]>(6),

    .platSolutions = cl.alloc<PlatformSolution>(MAX_PLAT_SOLUTIONS),
    .puSolutions   = cl.alloc<PUSolution>(MAX_PU_SOLUTIONS),
    .tenKSolutions = cl.alloc<TenKSolution>(MAX_10K_SOLUTIONS),

    .solnCounters = cl.alloc<int16_t>(N_SOLN_COUNTERS),

    .dev_tris  = cl.alloc<short>(18),
    .dev_norms = cl.alloc<float>(6),

    .deltaNX =
      (o.nSamplesNX > 1) ? (o.maxNX - o.minNX) / (o.nSamplesNX - 1) : 0,
    .deltaNY =
      (o.nSamplesNY > 1) ? (o.maxNY - o.minNY) / (o.nSamplesNY - 1) : 0,
    .deltaNZ =
      (o.nSamplesNZ > 1) ? (o.maxNZ - o.minNZ) / (o.nSamplesNZ - 1) : 0,

    .solnWriter              = SolutionWriter(o.outFile),
    .puSolutionLookup = std::unordered_map<uint64_t, PUSolution>(),
  };
  // init_reverse_atan, init_mag_set, and initialise_floors handled by
  // pregenerated table
  cl.run_kernel_once(
    "set_platform_pos", o.platformPos[0], o.platformPos[1], o.platformPos[2]);
  cl.run_kernel_once(
    "set_solution_buffers", q.platSolutions, q.puSolutions, q.tenKSolutions,
    q.solnCounters);

  Vec3ul index;
  Vec3f startNormal;
  for (size_t h = 0; h < o.nSamplesNY; h++) {
    for (size_t i = 0; i < o.nSamplesNX; i++) {
      for (size_t j = 0; j < o.nSamplesNZ; j++) {
        startNormal = Vec3f {
          o.minNX + i * q.deltaNX,
          o.minNY + h * q.deltaNY,
          o.minNZ + j * q.deltaNZ,
        };
        index = Vec3ul {i, h, j};

        testNormal(cl, o, q, startNormal, index);
      }
    }
  }
}