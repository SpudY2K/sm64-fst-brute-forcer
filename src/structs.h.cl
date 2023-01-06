#ifndef STRUCTS_H_CL
#define STRUCTS_H_CL

#if defined(__cplusplus) && !defined(__OPENCL_CPP_VERSION__)
  #include <algorithm>
  #include <cmath>
using namespace std;
#endif

#define MAX_10K_SOLUTIONS 50000
#define MAX_PU_SOLUTIONS 50000000
#define MAX_PLAT_SOLUTIONS 200000

#define N_SOLN_COUNTERS 3
#define PLAT_COUNTER_OFF 0
#define PU_COUNTER_OFF 1
#define TENK_COUNTER_OFF 2

struct TenKSolution {
  int puSolutionIdx;
  int startFloorIdx;
  float startPosition[3];
  float frame1Position[3];
  float frame2Position[3];
  int frame2QSteps;
  float pre10Kspeed;
  float pre10KVel[2];
  float returnVel[2];
  int stick10K[2];
  int cameraYaw10K;
  float startPositionLimits[2][3];
};

struct PUSolution {
  int platformSolutionIdx;
  float returnSpeed;
  int angle;
  float stickMag;
  int intendedDYaw;
};

struct PlatformSolution {
  float returnPosition[3];
  float endPosition[3];
  float endNormal[3];
  short endTriangles[2][3][3];
  float endTriangleNormals[2][3];
  float penultimateFloorNormalY;
  float penultimatePosition[3];
  int pux;
  int puz;
  int nFrames;
};

class SurfaceG {
public:
  short vertices[3][3];
  float normal[3];
  float origin_offset;
  float lower_y;
  float upper_y;

  float min_x;
  float max_x;
  float min_z;
  float max_z;

  SurfaceG(
    short x0, short y0, short z0, short x1, short y1, short z1, short x2,
    short y2, short z2) {
    short verts[3][3] = {{x0, y0, z0}, {x1, y1, z1}, {x2, y2, z2}};
    set_vertices(verts);
  }

  SurfaceG(short verts[3][3]) { set_vertices(verts); }

  SurfaceG() {}

  void set_vertices(short verts[3][3]) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        vertices[i][j] = verts[i][j];
      }
    }

    lower_y = min(min(vertices[0][1], vertices[1][1]), vertices[2][1]) - 5;
    upper_y = max(max(vertices[0][1], vertices[1][1]), vertices[2][1]) + 5;

    min_x = min(min(vertices[0][0], vertices[1][0]), vertices[2][0]);
    max_x = max(max(vertices[0][0], vertices[1][0]), vertices[2][0]);
    min_z = min(min(vertices[0][2], vertices[1][2]), vertices[2][2]);
    max_z = max(max(vertices[0][2], vertices[1][2]), vertices[2][2]);

    calculate_normal();
  }

  void calculate_normal() {
    normal[0] =
      (vertices[1][1] - vertices[0][1]) * (vertices[2][2] - vertices[1][2]) -
      (vertices[1][2] - vertices[0][2]) * (vertices[2][1] - vertices[1][1]);
    normal[1] =
      (vertices[1][2] - vertices[0][2]) * (vertices[2][0] - vertices[1][0]) -
      (vertices[1][0] - vertices[0][0]) * (vertices[2][2] - vertices[1][2]);
    normal[2] =
      (vertices[1][0] - vertices[0][0]) * (vertices[2][1] - vertices[1][1]) -
      (vertices[1][1] - vertices[0][1]) * (vertices[2][0] - vertices[1][0]);

    float mag = sqrt(
      normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

    mag = (float) (1.0 / mag);
    normal[0] *= mag;
    normal[1] *= mag;
    normal[2] *= mag;

    origin_offset =
      -(normal[0] * vertices[0][0] + normal[1] * vertices[0][1] +
        normal[2] * vertices[0][2]);
  }
};
#endif