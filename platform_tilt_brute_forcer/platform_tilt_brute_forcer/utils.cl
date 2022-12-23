bool check_inbounds(const float* mario_pos) {
    short x_mod = (short) (int) mario_pos[0];
    short y_mod = (short) (int) mario_pos[1];
    short z_mod = (short) (int) mario_pos[2];

    return (abs(x_mod) < 8192 & abs(y_mod) < 8192 & abs(z_mod) < 8192);
}

short atan2_lookupG(float z, float x) {
    return (x == 0.0f)? 0x0000 : short(gArctanTableG[uint16_t(float(float(z / x) * 1024.0 + 0.5))]);
}

short atan2sG(float z, float x) {
    short angle = 0;

    if (x >= 0) {
        if (z >= 0) {
            if (z >= x) {
                angle = atan2_lookupG(x, z);
            }
            else {
                angle = 0x4000 - atan2_lookupG(z, x);
            }
        }
        else {
            z = -z;

            if (z < x) {
                angle = 0x4000 + atan2_lookupG(z, x);
            }
            else {
                angle = 0x8000 - atan2_lookupG(x, z);
            }
        }
    }
    else {
        x = -x;

        if (z < 0) {
            z = -z;

            if (z >= x) {
                angle = 0x8000 + atan2_lookupG(x, z);
            }
            else {
                angle = 0xC000 - atan2_lookupG(z, x);
            }
        }
        else {
            if (z < x) {
                angle = 0xC000 + atan2_lookupG(z, x);
            }
            else {
                angle = -atan2_lookupG(x, z);
            }
        }
    }

    return ((angle + 32768) % 65536) - 32768;
}

int find_floor(
    float* pos, short (&triangles)[2][3][3], float (&normals)[2][3],
    float* pheight) {
    int idx = -1;

    int16_t x = static_cast<int16_t>(static_cast<int>(pos[0]));
    int16_t y = static_cast<int16_t>(static_cast<int>(pos[1]));
    int16_t z = static_cast<int16_t>(static_cast<int>(pos[2]));

    for (int i = 0; i < 2; i++) {
        int16_t x1 = triangles[i][0][0];
        int16_t z1 = triangles[i][0][2];
        int16_t x2 = triangles[i][1][0];
        int16_t z2 = triangles[i][1][2];

        // Check that the point is within the triangle bounds.
        if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
            continue;
        }

        // To slightly save on computation time, set this later.
        int16_t x3 = triangles[i][2][0];
        int16_t z3 = triangles[i][2][2];

        if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
            continue;
        }
        if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
            continue;
        }

        float nx = normals[i][0];
        float ny = normals[i][1];
        float nz = normals[i][2];
        float oo = -(nx * x1 + ny * triangles[i][0][1] + nz * z1);

        // Find the height of the floor at a given location.
        float height = -(x * nx + nz * z + oo) / ny;
        // Checks for floor interaction with a 78 unit buffer.
        if (y - (height + -78.0f) < 0.0f) {
            continue;
        }

        *pheight = height;
        idx      = i;
        break;
    }

    //! (Surface Cucking) Since only the first floor is returned and not the
    //! highest,
    //  higher floors can be "cucked" by lower floors.
    return idx;
}

int find_floor(
    float* position, SurfaceG** floor, float& floor_y, SurfaceG floor_set[],
    int n_floor_set) {
    short x = (short) (int) position[0];
    short y = (short) (int) position[1];
    short z = (short) (int) position[2];

    int floor_idx = -1;

    for (int i = 0; i < n_floor_set; ++i) {
        if (x < floor_set[i].min_x || x > floor_set[i].max_x ||
            z < floor_set[i].min_z || z > floor_set[i].max_z) {
            continue;
        }

        if ((floor_set[i].vertices[0][2] -
             z) * (floor_set[i].vertices[1][0] - floor_set[i].vertices[0][0]) -
                (floor_set[i].vertices[0][0] - x) *
                    (floor_set[i].vertices[1][2] -
                     floor_set[i].vertices[0][2]) <
            0) {
            continue;
        }
        if ((floor_set[i].vertices[1][2] -
             z) * (floor_set[i].vertices[2][0] - floor_set[i].vertices[1][0]) -
                (floor_set[i].vertices[1][0] - x) *
                    (floor_set[i].vertices[2][2] -
                     floor_set[i].vertices[1][2]) <
            0) {
            continue;
        }
        if ((floor_set[i].vertices[2][2] -
             z) * (floor_set[i].vertices[0][0] - floor_set[i].vertices[2][0]) -
                (floor_set[i].vertices[2][0] - x) *
                    (floor_set[i].vertices[0][2] -
                     floor_set[i].vertices[2][2]) <
            0) {
            continue;
        }

        float height =
            -(x * floor_set[i].normal[0] + floor_set[i].normal[2] * z +
              floor_set[i].origin_offset) /
            floor_set[i].normal[1];

        if (y - (height + -78.0f) < 0.0f) {
            continue;
        }

        floor_y   = height;
        *floor    = &floor_set[i];
        floor_idx = i;
        break;
    }

    return floor_idx;
}

float find_closest_mag(float target) {
    int minIdx = -1;
    int maxIdx = magCount;

    while (maxIdx > minIdx + 1) {
        int midIdx = (maxIdx + minIdx) / 2;

        if (target < magSet[midIdx]) {
            maxIdx = midIdx;
        }
        else {
            minIdx = midIdx;
        }
    }

    if (minIdx == -1) {
        return magSet[maxIdx];
    }
    else if (maxIdx == magCount) {
        return magSet[minIdx];
    }
    else if (target - magSet[minIdx] < magSet[maxIdx] - target) {
        return magSet[minIdx];
    }
    else {
        return magSet[maxIdx];
    }
}

int atan2b(double z, double x) {
    double A  = 65536 * atan2(x, z) / (2 * M_PI);
    A         = fmod(65536.0 + A, 65536.0);
    int lower = 0;
    int upper = 8192;

    while (upper - lower > 1) {
        int mid = (upper + lower) / 2;

        if (fmod(65536.0 + gArctanTableG[mid], 65536.0) > A) {
            upper = mid;
        }
        else {
            lower = mid;
        }
    }

    return lower;
}