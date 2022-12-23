#ifndef STRUCTS_H_CL
#define STRUCTS_H_CL

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
    float normal[3];
    float origin_offset;
    float lower_y;
    float upper_y;

    float min_x;
    float max_x;
    float min_z;
    float max_z;

    short vertices[3][3];

    SurfaceG(short verts[3][3]) { set_vertices(verts) }

    void set_vertices(short verts[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                vertices[i][j] = verts[i][j];
            }
        }

        lower_y =
            fminf(fminf(vertices[0][1], vertices[1][1]), vertices[2][1]) - 5;
        upper_y =
            fmaxf(fmaxf(vertices[0][1], vertices[1][1]), vertices[2][1]) + 5;

        min_x = fminf(fminf(vertices[0][0], vertices[1][0]), vertices[2][0]);
        max_x = fmaxf(fmaxf(vertices[0][0], vertices[1][0]), vertices[2][0]);
        min_z = fminf(fminf(vertices[0][2], vertices[1][2]), vertices[2][2]);
        max_z = fmaxf(fmaxf(vertices[0][2], vertices[1][2]), vertices[2][2]);

        calculate_normal();
    }

    void calculate_normal() {
        normal[0] = (vertices[1][1] - vertices[0][1]) *
                (vertices[2][2] - vertices[1][2]) -
            (vertices[1][2] - vertices[0][2]) *
                (vertices[2][1] - vertices[1][1]);
        normal[1] = (vertices[1][2] - vertices[0][2]) *
                (vertices[2][0] - vertices[1][0]) -
            (vertices[1][0] - vertices[0][0]) *
                (vertices[2][2] - vertices[1][2]);
        normal[2] = (vertices[1][0] - vertices[0][0]) *
                (vertices[2][1] - vertices[1][1]) -
            (vertices[1][1] - vertices[0][1]) *
                (vertices[2][0] - vertices[1][0]);

        float mag = sqrtf(
            normal[0] * normal[0] + normal[1] * normal[1] +
            normal[2] * normal[2]);

        mag = (float) (1.0 / mag);
        normal[0] *= mag;
        normal[1] *= mag;
        normal[2] *= mag;

        origin_offset =
            -(normal[0] * vertices[0][0] + normal[1] * vertices[0][1] +
              normal[2] * vertices[0][2]);
    }
};

const float normal_offsets[4][3] = {
    {0.01f, -0.01f, 0.01f},
    {-0.01f, -0.01f, 0.01f},
    {-0.01f, -0.01f, -0.01f},
    {0.01f, -0.01f, -0.01f},
};

const short default_triangles[2][3][3] = {
    {{307, 307, -306}, {-306, 307, -306}, {-306, 307, 307}},
    {{307, 307, -306}, {-306, 307, 307}, {307, 307, 307}},
};

#endif