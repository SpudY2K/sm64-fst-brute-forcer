#include "solver.h.cl"
#include "geo.h.cl"
#include "structs.h.cl"

bool test_stick_position(
    int solIdx, int x, int y, float endSpeed, float vel1, float xVel1,
    float zVel1, int angle, int cameraYaw, float* startPosition,
    float* oneUpPlatformPosition, float oneUpPlatformXMin,
    float oneUpPlatformXMax, float oneUpPlatformYMin, float oneUpPlatformYMax,
    float oneUpPlatformZMin, float oneUpPlatformZMax,
    float oneUpPlatformNormalX, float oneUpPlatformNormalY, int f,
    float* frame1Position, float* returnPosition, short (&triangles)[2][3][3],
    float (&normals)[2][3], int d, int q) {
    bool foundSolution = false;

    float testStartPosition[3] = {
        startPosition[0], startPosition[1], startPosition[2]};
    float testFrame1Position[3] = {
        frame1Position[0], frame1Position[1], frame1Position[2]};
    float testOneUpPlatformPosition[3] = {
        oneUpPlatformPosition[0], oneUpPlatformPosition[1],
        oneUpPlatformPosition[2]};

    int trueX = (x == 0) ? 0 : ((x < 0) ? x - 6 : x + 6);
    int trueY = (y == 0) ? 0 : ((y < 0) ? y - 6 : y + 6);

    float mag = sqrt(float(x * x + y * y));

    float xS = x;
    float yS = y;

    if (mag > 64.0f) {
        xS  = xS * (64.0f / mag);
        yS  = yS * (64.0f / mag);
        mag = 64.0f;
    }

    float intendedMag = ((mag / 64.0f) * (mag / 64.0f)) * 32.0f;
    int intendedYaw   = atan2sG(-yS, xS) + cameraYaw;
    int intendedDYaw  = intendedYaw - angle;
    intendedDYaw      = (65536 + (intendedDYaw % 65536)) % 65536;

    float xVel2a = xVel1;
    float zVel2a = zVel1;

    float oldSpeed = sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

    xVel2a +=
        zVel2a * (intendedMag / 32.0f) * gSineTableG[intendedDYaw >> 4] * 0.05f;
    zVel2a -=
        xVel2a * (intendedMag / 32.0f) * gSineTableG[intendedDYaw >> 4] * 0.05f;

    float newSpeed = sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

    xVel2a = xVel2a * oldSpeed / newSpeed;
    zVel2a = zVel2a * oldSpeed / newSpeed;

    xVel2a += 7.0f * oneUpPlatformNormalX;

    float forward = gCosineTableG[intendedDYaw >> 4];
    forward *= 0.5f + 0.5f * vel1 / 100.0f;
    float lossFactor = intendedMag / 32.0f * forward * 0.02f + 0.92f;

    xVel2a *= lossFactor;
    zVel2a *= lossFactor;

    float vel2a = -sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

    bool speedTest = true;

    if (vel2a != endSpeed) {
        double w     = intendedMag * gCosineTableG[intendedDYaw >> 4];
        double eqB   = (50.0 + 147200.0 / w);
        double eqC   = -(320000.0 / w) * endSpeed;
        double eqDet = eqB * eqB - eqC;

        float vel1a;

        if (eqDet > 0) {
            vel1a = sqrt(eqDet) - eqB;

            if (vel1a >= 0) {
                bool searchLoop = true;

                while (searchLoop) {
                    xVel2a = vel1a * gSineTableG[angle >> 4];
                    zVel2a = vel1a * gCosineTableG[angle >> 4];

                    oldSpeed = sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

                    xVel2a += zVel2a * (intendedMag / 32.0f) *
                        gSineTableG[intendedDYaw >> 4] * 0.05f;
                    zVel2a -= xVel2a * (intendedMag / 32.0f) *
                        gSineTableG[intendedDYaw >> 4] * 0.05f;

                    newSpeed = sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

                    xVel2a = xVel2a * oldSpeed / newSpeed;
                    zVel2a = zVel2a * oldSpeed / newSpeed;

                    xVel2a += 7.0f * oneUpPlatformNormalX;

                    forward = gCosineTableG[intendedDYaw >> 4] *
                        (0.5f + 0.5f * vel1a / 100.0f);
                    lossFactor = intendedMag / 32.0f * forward * 0.02f + 0.92f;

                    xVel2a *= lossFactor;
                    zVel2a *= lossFactor;

                    float vel2b = -sqrt(xVel2a * xVel2a + zVel2a * zVel2a);

                    if (vel2b > endSpeed) {
                        vel1a =
                            fmax(vel1a + 0.25f, nextafter(vel1a, INFINITY));
                    }
                    else {
                        if (vel2b == endSpeed) {
                            vel2a      = vel2b;
                            vel1       = vel1a;
                            xVel1      = vel1 * gSineTableG[angle >> 4];
                            zVel1      = vel1 * gCosineTableG[angle >> 4];
                            searchLoop = false;
                        }
                        else {
                            speedTest = false;
                            break;
                        }
                    }
                }

                if (speedTest) {
                    testFrame1Position[0] = testOneUpPlatformPosition[0];
                    testFrame1Position[2] = testOneUpPlatformPosition[2];
                    bool inBoundsTest     = true;

                    for (int q1 = 0; q1 < q; q1++) {
                        testFrame1Position[0] =
                            testFrame1Position[0] - (xVel1 / 4.0f);
                        testFrame1Position[2] =
                            testFrame1Position[2] - (zVel1 / 4.0f);

                        if (!check_inbounds(testFrame1Position)) {
                            inBoundsTest = false;
                            break;
                        }
                    }

                    if (inBoundsTest) {
                        double targetDist =
                            (double) vel1 * (double) startNormals[f][1] / 4.0;
                        double newDist = sqrt(
                            (testFrame1Position[0] - testStartPosition[0]) *
                                (testFrame1Position[0] - testStartPosition[0]) +
                            (testFrame1Position[2] - testStartPosition[2]) *
                                (testFrame1Position[2] - testStartPosition[2]));

                        if (fabs(vel1 * startNormals[f][1] / 4.0 - newDist) >
                            1000.0) {
                            speedTest = false;
                        }

                        testStartPosition[0] = testFrame1Position[0] +
                            (targetDist / newDist) *
                                (testStartPosition[0] - testFrame1Position[0]);
                        testStartPosition[2] = testFrame1Position[2] +
                            (targetDist / newDist) *
                                (testStartPosition[2] - testFrame1Position[2]);

                        if (!check_inbounds(testStartPosition)) {
                            speedTest = false;
                        }

                        int angle2 = atan2sG(
                            testFrame1Position[2] - testStartPosition[2],
                            testFrame1Position[0] - testStartPosition[0]);
                        angle2 = (65536 + angle2) % 65536;

                        if (angle != angle2) {
                            speedTest = false;
                        }
                    }
                    else {
                        speedTest = false;
                    }
                }
            }
            else {
                speedTest = false;
            }
        }
        else {
            speedTest = false;
        }
    }

    float predictedReturnPosition[3] = {
        testOneUpPlatformPosition[0] + (oneUpPlatformNormalY * xVel2a / 4.0f),
        returnPosition[1],
        testOneUpPlatformPosition[2] + (oneUpPlatformNormalY * zVel2a / 4.0f)};

    if (speedTest &&
        fabs(predictedReturnPosition[0] - returnPosition[0]) < 1000 &&
        fabs(predictedReturnPosition[2] - returnPosition[2]) < 1000) {
        float xShift = predictedReturnPosition[0] - returnPosition[0];
        float zShift = predictedReturnPosition[2] - returnPosition[2];

        testOneUpPlatformPosition[0] = testOneUpPlatformPosition[0] - xShift;
        testOneUpPlatformPosition[1] = oneUpPlatformYMin +
            (oneUpPlatformYMax - oneUpPlatformYMin) *
                ((short) (int) testOneUpPlatformPosition[0] -
                 oneUpPlatformXMin) /
                (oneUpPlatformXMax - oneUpPlatformXMin);
        testOneUpPlatformPosition[2] = testOneUpPlatformPosition[2] - zShift;

        if ((int) (short) testOneUpPlatformPosition[0] >= oneUpPlatformXMin &&
            (int) (short) testOneUpPlatformPosition[0] <= oneUpPlatformXMax &&
            (int) (short) testOneUpPlatformPosition[2] >= oneUpPlatformZMin &&
            (int) (short) testOneUpPlatformPosition[2] <= oneUpPlatformZMax &&
            testOneUpPlatformPosition[1] < returnPosition[1] &&
            testOneUpPlatformPosition[1] >
                fmax(-2971.0f, returnPosition[1] - 78.0f)) {
            testStartPosition[0] = testStartPosition[0] - xShift;
            testStartPosition[2] = testStartPosition[2] - zShift;

            float marioMinY =
                fmax(-2971.0f, testOneUpPlatformPosition[1] - 78.0f);

            float intersectionPoints[2][3];
            int intersections = 0;

            double px = testStartPosition[0];
            double pz = testStartPosition[2];
            double qx = testFrame1Position[0];
            double qz = testFrame1Position[2];

            bool intOnSquishEdge[2] = {false, false};

            for (int i = 0; i < 3; i++) {
                double ax = startTriangles[f][i][0];
                double ay = startTriangles[f][i][1];
                double az = startTriangles[f][i][2];
                double bx = startTriangles[f][(i + 1) % 3][0];
                double by = startTriangles[f][(i + 1) % 3][1];
                double bz = startTriangles[f][(i + 1) % 3][2];

                double t = ((pz - qz) * (pz - az) + (px - qx) * (px - ax)) /
                    ((pz - qz) * (bz - az) + (px - qx) * (bx - ax));

                if (t >= 0.0 && t <= 1.0) {
                    intOnSquishEdge[intersections] =
                        (f == 0 &&
                         ((i == 0 && squishCeilings[2]) ||
                          (i == 1 && squishCeilings[0]))) ||
                        (f == 1 &&
                         ((i == 1 && squishCeilings[1]) ||
                          (i == 2 && squishCeilings[3])));
                    intersectionPoints[intersections][0] = ax + (bx - ax) * t;
                    intersectionPoints[intersections][1] = ay + (by - ay) * t;
                    intersectionPoints[intersections][2] = az + (bz - az) * t;
                    intersections++;
                }
            }

            double cutPoints[2];

            double ax = intersectionPoints[0][0];
            double ay = intersectionPoints[0][1];
            double az = intersectionPoints[0][2];
            double bx = intersectionPoints[1][0];
            double by = intersectionPoints[1][1];
            double bz = intersectionPoints[1][2];

            px = testFrame1Position[0];
            pz = testFrame1Position[2];

            int angleIdx = gReverseArctanTable[angle];
            int prevAngle =
                (65536 + gArctanTableG[(angleIdx + 8191) % 8192]) % 65536;
            int nextAngle =
                (65536 + gArctanTableG[(angleIdx + 1) % 8192]) % 65536;

            double m =
                (gSineTableG[nextAngle >> 4] + gSineTableG[angle >> 4]) / 2.0;
            double n =
                (gCosineTableG[nextAngle >> 4] + gCosineTableG[angle >> 4]) /
                2.0;

            cutPoints[0] = ((pz - az) + (n / m) * (ax - px)) /
                ((bz - az) - (n / m) * (bx - ax));

            m = (gSineTableG[prevAngle >> 4] + gSineTableG[angle >> 4]) / 2.0;
            n = (gCosineTableG[prevAngle >> 4] + gCosineTableG[angle >> 4]) /
                2.0;

            cutPoints[1] = ((pz - az) + (n / m) * (ax - px)) /
                ((bz - az) - (n / m) * (bx - ax));

            if (cutPoints[0] > cutPoints[1]) {
                double temp  = cutPoints[0];
                cutPoints[0] = cutPoints[1];
                cutPoints[1] = temp;
            }

            cutPoints[0] = fmax(cutPoints[0], 0.0);
            cutPoints[1] = fmin(cutPoints[1], 1.0);

            if (cutPoints[0] <= cutPoints[1]) {
                intersectionPoints[0][0] = ax + (bx - ax) * cutPoints[0];
                intersectionPoints[0][1] = ay + (by - ay) * cutPoints[0];
                intersectionPoints[0][2] = az + (bz - az) * cutPoints[0];

                intersectionPoints[1][0] = ax + (bx - ax) * cutPoints[1];
                intersectionPoints[1][1] = ay + (by - ay) * cutPoints[1];
                intersectionPoints[1][2] = az + (bz - az) * cutPoints[1];

                if (fmax(intersectionPoints[0][1], intersectionPoints[1][1]) >
                        marioMinY &&
                    fmin(intersectionPoints[0][1], intersectionPoints[1][1]) <
                        testOneUpPlatformPosition[1]) {
                    if (intersectionPoints[0][1] < marioMinY) {
                        double ratio = (marioMinY - intersectionPoints[0][1]) /
                            (intersectionPoints[1][1] -
                             intersectionPoints[0][1]);
                        cutPoints[0] = cutPoints[0] +
                            (cutPoints[1] - cutPoints[0]) * ratio;
                        intersectionPoints[0][0] = intersectionPoints[0][0] +
                            (intersectionPoints[1][0] -
                             intersectionPoints[0][0]) *
                                ratio;
                        intersectionPoints[0][2] = intersectionPoints[0][2] +
                            (intersectionPoints[1][2] -
                             intersectionPoints[0][2]) *
                                ratio;
                        intersectionPoints[0][1] = marioMinY;
                    }
                    else if (intersectionPoints[1][1] < marioMinY) {
                        double ratio = (marioMinY - intersectionPoints[1][1]) /
                            (intersectionPoints[0][1] -
                             intersectionPoints[1][1]);
                        cutPoints[1] = cutPoints[1] +
                            (cutPoints[0] - cutPoints[1]) * ratio;
                        intersectionPoints[1][0] = intersectionPoints[1][0] +
                            (intersectionPoints[0][0] -
                             intersectionPoints[1][0]) *
                                ratio;
                        intersectionPoints[1][2] = intersectionPoints[1][2] +
                            (intersectionPoints[0][2] -
                             intersectionPoints[1][2]) *
                                ratio;
                        intersectionPoints[1][1] = marioMinY;
                    }

                    if (intersectionPoints[0][1] >
                        testOneUpPlatformPosition[1]) {
                        double ratio = (testOneUpPlatformPosition[1] -
                                        intersectionPoints[0][1]) /
                            (intersectionPoints[1][1] -
                             intersectionPoints[0][1]);
                        cutPoints[0] = cutPoints[0] +
                            (cutPoints[1] - cutPoints[0]) * ratio;
                        intersectionPoints[0][0] = intersectionPoints[0][0] +
                            (intersectionPoints[1][0] -
                             intersectionPoints[0][0]) *
                                ratio;
                        intersectionPoints[0][2] = intersectionPoints[0][2] +
                            (intersectionPoints[1][2] -
                             intersectionPoints[0][2]) *
                                ratio;
                        intersectionPoints[0][1] = testOneUpPlatformPosition[1];
                    }
                    else if (
                        intersectionPoints[1][1] <
                        testOneUpPlatformPosition[1]) {
                        double ratio = (testOneUpPlatformPosition[1] -
                                        intersectionPoints[1][1]) /
                            (intersectionPoints[0][1] -
                             intersectionPoints[1][1]);
                        cutPoints[1] = cutPoints[1] +
                            (cutPoints[0] - cutPoints[1]) * ratio;
                        intersectionPoints[1][0] = intersectionPoints[1][0] +
                            (intersectionPoints[0][0] -
                             intersectionPoints[1][0]) *
                                ratio;
                        intersectionPoints[1][2] = intersectionPoints[1][2] +
                            (intersectionPoints[0][2] -
                             intersectionPoints[1][2]) *
                                ratio;
                        intersectionPoints[1][1] = testOneUpPlatformPosition[1];
                    }

                    testStartPosition[0] =
                        (intersectionPoints[0][0] + intersectionPoints[1][0]) /
                        2.0f;
                    testStartPosition[2] =
                        (intersectionPoints[0][2] + intersectionPoints[1][2]) /
                        2.0f;

                    float floorHeight;

                    int floorIdx = find_floor(
                        testStartPosition, startTriangles, startNormals,
                        &floorHeight);

                    if (floorIdx == f && floorHeight > marioMinY &&
                        floorHeight < testOneUpPlatformPosition[1]) {
                        testStartPosition[1] = floorHeight;
                        predictedReturnPosition[0] =
                            testOneUpPlatformPosition[0] +
                            (oneUpPlatformNormalY * xVel2a / 4.0f);
                        predictedReturnPosition[2] =
                            testOneUpPlatformPosition[2] +
                            (oneUpPlatformNormalY * zVel2a / 4.0f);

                        if (predictedReturnPosition[0] == returnPosition[0] &&
                            predictedReturnPosition[2] == returnPosition[2]) {
                            testFrame1Position[0] =
                                testFrame1Position[0] - xShift;
                            testFrame1Position[1] = testStartPosition[1];
                            testFrame1Position[2] =
                                testFrame1Position[2] - zShift;
                            foundSolution           = true;
                            struct PUSolution puSol = puSolutions[solIdx];
                            struct PlatformSolution platSol =
                                platSolutions[puSol.platformSolutionIdx];
                            printf(
                                "---------------------------------------\nFound Solution:\n---------------------------------------\n    Start Position: %.10g, %.10g, %.10g\n    Frame 1 Position: %.10g, %.10g, %.10g\n    Frame 2 Position: %.10g, %.10g, %.10g\n    Return Position: %.10g, %.10g, %.10g\n    PU Route Speed: %.10g (x=%.10g, z=%.10g)\n    PU Return Speed: %.10g (x=%.10g, z=%.10g)\n    Frame 2 Q-steps: %d\n    10k Stick X: %d\n    10k Stick Y: %d\n    10k Camera Yaw: %d\n    Start Floor Normal: %.10g, %.10g, %.10g\n    Start Position Limit 1: %.10g %.10g %.10g\n    Start Position Limit 2: %.10g %.10g %.10g\n",
                                testStartPosition[0], testStartPosition[1],
                                testStartPosition[2], testFrame1Position[0],
                                testFrame1Position[1], testFrame1Position[2],
                                testOneUpPlatformPosition[0],
                                testOneUpPlatformPosition[1],
                                testOneUpPlatformPosition[2], returnPosition[0],
                                returnPosition[1], returnPosition[2], vel1,
                                xVel1, zVel1, endSpeed, xVel2a, zVel2a, q,
                                trueX, trueY, cameraYaw, startNormals[f][0],
                                startNormals[f][1], startNormals[f][2],
                                intersectionPoints[0][0],
                                intersectionPoints[0][1],
                                intersectionPoints[0][2],
                                intersectionPoints[1][0],
                                intersectionPoints[1][1],
                                intersectionPoints[1][2]);
                            printf(
                                "---------------------------------------\n    Tilt Frames: %d\n    Post-Tilt Platform Normal: %.10g, %.10g, %.10g\n    Post-Tilt Position: %.10g, %.10g, %.10g\n    Upwarp PU X: %d\n    Upwarp PU Z: %d\n    Upwarp Slide Facing Angle: %d\n    Upwarp Slide Intended Mag: %.10g\n    Upwarp Slide Indented DYaw: %d\n---------------------------------------\n\n\n",
                                platSol.nFrames, platSol.endNormal[0],
                                platSol.endNormal[1], platSol.endNormal[2],
                                platSol.endPosition[0], platSol.endPosition[1],
                                platSol.endPosition[2], platSol.pux,
                                platSol.puz, puSol.angle, puSol.stickMag,
                                puSol.intendedDYaw);

                            int idx = atomicAdd(&n10KSolutions, 1);

                            if (idx < MAX_10K_SOLUTIONS) {
                                struct TenKSolution solution;
                                solution.puSolutionIdx = solIdx;
                                solution.startFloorIdx = f;
                                solution.startPosition[0] =
                                    testStartPosition[0];
                                solution.startPosition[1] =
                                    testStartPosition[1];
                                solution.startPosition[2] =
                                    testStartPosition[2];
                                solution.startPositionLimits[0][0] =
                                    intersectionPoints[0][0];
                                solution.startPositionLimits[0][1] =
                                    intersectionPoints[0][1];
                                solution.startPositionLimits[0][2] =
                                    intersectionPoints[0][2];
                                solution.startPositionLimits[1][0] =
                                    intersectionPoints[1][0];
                                solution.startPositionLimits[1][1] =
                                    intersectionPoints[1][1];
                                solution.startPositionLimits[1][2] =
                                    intersectionPoints[1][2];
                                solution.frame1Position[0] =
                                    testFrame1Position[0];
                                solution.frame1Position[1] =
                                    testFrame1Position[1];
                                solution.frame1Position[2] =
                                    testFrame1Position[2];
                                solution.frame2Position[0] =
                                    testOneUpPlatformPosition[0];
                                solution.frame2Position[1] =
                                    testOneUpPlatformPosition[1];
                                solution.frame2Position[2] =
                                    testOneUpPlatformPosition[2];
                                solution.frame2QSteps = q;
                                solution.pre10Kspeed  = vel1;
                                solution.pre10KVel[0] = xVel1;
                                solution.pre10KVel[1] = zVel1;
                                solution.returnVel[0] = xVel2a;
                                solution.returnVel[1] = zVel2a;
                                solution.stick10K[0]  = trueX;
                                solution.stick10K[1]  = trueY;
                                solution.cameraYaw10K = cameraYaw;
                                tenKSolutions[idx]    = solution;
                            }
                        }
                    }
                }
            }
        }
    }

    return foundSolution;
}

int calculate_camera_yaw(
    float* currentPosition, float* lakituPosition) {
    short baseCameraYaw       = -16384;
    float baseCameraDist      = 1400.0;
    short baseCameraPitch     = 0x05B0;
    short baseCameraFaceAngle = 24576;

    SurfaceG* floor;
    float floorY;

    float xOff = currentPosition[0] +
        gSineTableG[((65536 + (int) baseCameraYaw) % 65536) >> 4] * 40.f;
    float zOff = currentPosition[2] +
        gCosineTableG[((65536 + (int) baseCameraYaw) % 65536) >> 4] * 40.f;
    float offPos[3] = {xOff, currentPosition[1], zOff};

    int floorIdx = find_floor(offPos, &floor, floorY, floorsG, total_floorsG);
    floorY       = floorY - currentPosition[1];

    if (floorIdx != -1) {
        if (floorY > 0) {
            if (!(floor->normal[2] == 0.f && floorY < 100.f)) {
                baseCameraPitch += atan2sG(40.f, floorY);
            }
        }
    }

    baseCameraPitch = baseCameraPitch + 2304;

    float cameraPos[3] = {
        currentPosition[0] +
            baseCameraDist *
                gCosineTableG[((65536 + (int) baseCameraPitch) % 65536) >> 4] *
                gSineTableG[((65536 + (int) baseCameraYaw) % 65536) >> 4],
        currentPosition[1] + 125.0f +
            baseCameraDist *
                gSineTableG[((65536 + (int) baseCameraPitch) % 65536) >> 4],
        currentPosition[2] +
            baseCameraDist *
                gCosineTableG[((65536 + (int) baseCameraPitch) % 65536) >> 4] *
                gCosineTableG[((65536 + (int) baseCameraYaw) % 65536) >> 4]};

    float pan[3]  = {0, 0, 0};
    float temp[3] = {0, 0, 0};

    // Get distance and angle from camera to Mario.
    float dx = currentPosition[0] - cameraPos[0];
    float dy = currentPosition[1] + 125.0f;
    float dz = currentPosition[2] - cameraPos[2];

    float cameraDist  = sqrt(dx * dx + dy * dy + dz * dz);
    float cameraPitch = atan2sG(sqrt(dx * dx + dz * dz), dy);
    float cameraYaw   = atan2sG(dz, dx);

    // The camera will pan ahead up to about 30% of the camera's distance to
    // Mario.
    pan[2] = gSineTableG[0xC0] * cameraDist;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] *
            gSineTableG[((65536 + (int) baseCameraFaceAngle) % 65536) >> 4] +
        temp[0] *
            gCosineTableG[((65536 + (int) baseCameraFaceAngle) % 65536) >> 4];
    pan[2] = temp[2] *
            gCosineTableG[((65536 + (int) baseCameraFaceAngle) % 65536) >> 4] +
        temp[0] *
            gSineTableG[((65536 + (int) baseCameraFaceAngle) % 65536) >> 4];

    // rotate in the opposite direction
    cameraYaw = -cameraYaw;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[((65536 + (int) cameraYaw) % 65536) >> 4] +
        temp[0] * gCosineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];
    pan[2] = temp[2] * gCosineTableG[((65536 + (int) cameraYaw) % 65536) >> 4] +
        temp[0] * gSineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];

    // Only pan left or right
    pan[2] = 0.f;

    cameraYaw = -cameraYaw;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[((65536 + (int) cameraYaw) % 65536) >> 4] +
        temp[0] * gCosineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];
    pan[2] = temp[2] * gCosineTableG[((65536 + (int) cameraYaw) % 65536) >> 4] +
        temp[0] * gSineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];

    float cameraFocus[3] = {
        currentPosition[0] + pan[0], currentPosition[1] + 125.0f + pan[1],
        currentPosition[2] + pan[2]};

    dx = cameraFocus[0] - lakituPosition[0];
    dy = cameraFocus[1] - lakituPosition[1];
    dz = cameraFocus[2] - lakituPosition[2];

    cameraDist  = sqrt(dx * dx + dy * dy + dz * dz);
    cameraPitch = atan2sG(sqrt(dx * dx + dz * dz), dy);
    cameraYaw   = atan2sG(dz, dx);

    if (cameraPitch > 15872) {
        cameraPitch = 15872;
    }
    if (cameraPitch < -15872) {
        cameraPitch = -15872;
    }

    cameraFocus[0] = lakituPosition[0] +
        cameraDist * gCosineTableG[((65536 + (int) cameraPitch) % 65536) >> 4] *
            gSineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];
    cameraFocus[1] = lakituPosition[1] +
        cameraDist * gSineTableG[((65536 + (int) cameraPitch) % 65536) >> 4];
    cameraFocus[2] = lakituPosition[2] +
        cameraDist * gCosineTableG[((65536 + (int) cameraPitch) % 65536) >> 4] *
            gCosineTableG[((65536 + (int) cameraYaw) % 65536) >> 4];

    return atan2sG(
        lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
}

bool test_one_up_position(
    int solIdx, float* startPosition, float* oneUpPlatformPosition,
    float* returnPosition, float endSpeed, float oneUpPlatformXMin,
    float oneUpPlatformXMax, float oneUpPlatformYMin, float oneUpPlatformYMax,
    float oneUpPlatformZMin, float oneUpPlatformZMax,
    float oneUpPlatformNormalX, float oneUpPlatformNormalY, int f, int d,
    short (&triangles)[2][3][3], float (&normals)[2][3]) {
    float cameraPositions[4][3] = {
        {-8192, -2918, -8192},
        {-8192, -2918, 8191},
        {8191, -2918, -8192},
        {8191, -2918, 8191}};
    bool foundSolution = false;

    int minCameraYaw = INT_MAX;
    int maxCameraYaw = -INT_MAX;

    for (int k = 0; k < 4; k++) {
        int cameraYaw =
            calculate_camera_yaw(oneUpPlatformPosition, cameraPositions[k]);
        cameraYaw    = (65536 + cameraYaw) % 65536;
        minCameraYaw = min(minCameraYaw, cameraYaw);
        maxCameraYaw = max(maxCameraYaw, cameraYaw);
    }

    int minCameraIdx = gReverseArctanTable[minCameraYaw];
    int maxCameraIdx = gReverseArctanTable[maxCameraYaw];

    if (maxCameraYaw - minCameraYaw > 32768) {
        int temp     = maxCameraIdx;
        maxCameraIdx = 8192 + minCameraIdx;
        minCameraIdx = temp;
    }

    for (int cIdx = minCameraIdx; cIdx <= maxCameraIdx; cIdx++) {
        int cameraYaw = gArctanTableG[cIdx % 65536];

        float xVel2 = 4.0f * (returnPosition[0] - oneUpPlatformPosition[0]) /
            oneUpPlatformNormalY;
        float zVel2 = 4.0f * (returnPosition[2] - oneUpPlatformPosition[2]) /
            oneUpPlatformNormalY;

        float s = xVel2 / zVel2;

        int angle = atan2sG(-zVel2, -xVel2);
        angle     = (65536 + angle) % 65536;

        for (int q = 1; q <= 4; q++) {
            double eqA = 1.0 -
                (((double) startNormals[f][1] * (double) startNormals[f][1]) /
                 ((double) q * (double) q));
            double eqB = 2.0 *
                ((startPosition[0] - oneUpPlatformPosition[0]) *
                     gSineTableG[angle >> 4] +
                 (startPosition[2] - oneUpPlatformPosition[2]) *
                     gCosineTableG[angle >> 4]);
            double eqC =
                ((startPosition[0] - oneUpPlatformPosition[0]) *
                     (startPosition[0] - oneUpPlatformPosition[0]) +
                 (startPosition[2] - oneUpPlatformPosition[2]) *
                     (startPosition[2] - oneUpPlatformPosition[2]));
            double eqDet = (eqB * eqB) - (4.0 * eqA * eqC);

            if (eqDet >= 0) {
                float vel1 =
                    4.0f * ((-eqB - sqrt(eqDet)) / (2.0 * eqA)) / (float) q;
                float xVel1 = vel1 * gSineTableG[angle >> 4];
                float zVel1 = vel1 * gCosineTableG[angle >> 4];

                float frame1Position[3] = {
                    oneUpPlatformPosition[0], startPosition[1],
                    oneUpPlatformPosition[2]};
                bool inBoundsTest = true;

                for (int q1 = 0; q1 < q; q1++) {
                    frame1Position[0] = frame1Position[0] - (xVel1 / 4.0f);
                    frame1Position[2] = frame1Position[2] - (zVel1 / 4.0f);

                    if (!check_inbounds(frame1Position)) {
                        inBoundsTest = false;
                        break;
                    }
                }

                if (inBoundsTest) {
                    int angle2 = atan2sG(
                        frame1Position[2] - startPosition[2],
                        frame1Position[0] - startPosition[0]);
                    angle2 = (65536 + angle2) % 65536;

                    if (angle == angle2) {
                        double m  = (double) endSpeed / (double) vel1;
                        double m1 = 32.0 * ((m - 0.92) / 0.02) /
                            (double) (0.5f + (0.5f * vel1 / 100.0f));

                        double t = (double) xVel1 / (double) zVel1;

                        double n;

                        if (zVel2 == 0) {
                            n = zVel1 / xVel2;
                        }
                        else if (zVel1 == 0) {
                            n = -zVel2 / xVel1;
                        }
                        else {
                            n = (-((double) s * (double) t) - 1.0 +
                                 sqrt(
                                     ((double) s * (double) t - 1.0) *
                                         ((double) s * (double) t - 1.0) +
                                     4.0 * (double) s * (double) s)) /
                                (2.0 * (double) s);
                        }

                        double n1 = 32.0 * n / 0.05;

                        double targetDYaw =
                            65536.0 * (atan2(n1, m1) / (2.0 * M_PI));
                        double targetMag = sqrt(m1 * m1 + n1 * n1);

                        double stickAngle = fmod(
                            65536.0 +
                                fmod(targetDYaw + angle - cameraYaw, 65536.0),
                            65536.0);
                        double stickMagnitude = sqrt(128.0 * targetMag);

                        if (stickMagnitude < 70.0) {
                            if (stickMagnitude < 64.0) {
                                double yS = -stickMagnitude *
                                    cos(2.0 * M_PI * (stickAngle / 65536));
                                double xS = stickMagnitude *
                                    sin(2.0 * M_PI * (stickAngle / 65536));

                                for (int x = floor(xS);
                                     x <= ceil(xS) && !foundSolution; x++) {
                                    if (x != -1 && x != 1) {
                                        for (int y = floor(yS);
                                             y <= ceil(yS) && !foundSolution;
                                             y++) {
                                            if (y != -1 && y != 1) {
                                                if (test_stick_position(
                                                        solIdx, x, y, endSpeed,
                                                        vel1, xVel1, zVel1,
                                                        angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            else {
                                double yS = -64.0 *
                                    sin(2.0 * M_PI * (stickAngle / 65536));
                                double xS = 64.0 *
                                    cos(2.0 * M_PI * (stickAngle / 65536));

                                if (xS == 0) {
                                    if (yS < 0) {
                                        for (int y = -64; y <= -128; y--) {
                                            if (test_stick_position(
                                                    solIdx, 0, y, endSpeed,
                                                    vel1, xVel1, zVel1, angle,
                                                    cameraYaw, startPosition,
                                                    oneUpPlatformPosition,
                                                    oneUpPlatformXMin,
                                                    oneUpPlatformXMax,
                                                    oneUpPlatformYMin,
                                                    oneUpPlatformYMax,
                                                    oneUpPlatformZMin,
                                                    oneUpPlatformZMax,
                                                    oneUpPlatformNormalX,
                                                    oneUpPlatformNormalY, f,
                                                    frame1Position,
                                                    returnPosition, triangles,
                                                    normals, d, q)) {
                                                foundSolution = true;
                                                break;
                                            }
                                        }
                                    }
                                    else {
                                        for (int y = 64; y <= 127; y++) {
                                            if (test_stick_position(
                                                    solIdx, 0, y, endSpeed,
                                                    vel1, xVel1, zVel1, angle,
                                                    cameraYaw, startPosition,
                                                    oneUpPlatformPosition,
                                                    oneUpPlatformXMin,
                                                    oneUpPlatformXMax,
                                                    oneUpPlatformYMin,
                                                    oneUpPlatformYMax,
                                                    oneUpPlatformZMin,
                                                    oneUpPlatformZMax,
                                                    oneUpPlatformNormalX,
                                                    oneUpPlatformNormalY, f,
                                                    frame1Position,
                                                    returnPosition, triangles,
                                                    normals, d, q)) {
                                                foundSolution = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                                else if (yS == 0) {
                                    if (xS < 0) {
                                        for (int x = -64; x <= -128; x--) {
                                            if (test_stick_position(
                                                    solIdx, x, 0, endSpeed,
                                                    vel1, xVel1, zVel1, angle,
                                                    cameraYaw, startPosition,
                                                    oneUpPlatformPosition,
                                                    oneUpPlatformXMin,
                                                    oneUpPlatformXMax,
                                                    oneUpPlatformYMin,
                                                    oneUpPlatformYMax,
                                                    oneUpPlatformZMin,
                                                    oneUpPlatformZMax,
                                                    oneUpPlatformNormalX,
                                                    oneUpPlatformNormalY, f,
                                                    frame1Position,
                                                    returnPosition, triangles,
                                                    normals, d, q)) {
                                                foundSolution = true;
                                                break;
                                            }
                                        }
                                    }
                                    else {
                                        for (int x = 64; x <= 127; x++) {
                                            if (test_stick_position(
                                                    solIdx, x, 0, endSpeed,
                                                    vel1, xVel1, zVel1, angle,
                                                    cameraYaw, startPosition,
                                                    oneUpPlatformPosition,
                                                    oneUpPlatformXMin,
                                                    oneUpPlatformXMax,
                                                    oneUpPlatformYMin,
                                                    oneUpPlatformYMax,
                                                    oneUpPlatformZMin,
                                                    oneUpPlatformZMax,
                                                    oneUpPlatformNormalX,
                                                    oneUpPlatformNormalY, f,
                                                    frame1Position,
                                                    returnPosition, triangles,
                                                    normals, d, q)) {
                                                foundSolution = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                                else if (fabs(xS) > fabs(yS)) {
                                    if (xS < 0) {
                                        int maxX = ceil(-128 * xS / yS);

                                        for (int x = floor(xS); x <= maxX;
                                             x++) {
                                            double y = (double) x * (yS / xS);

                                            if (fabs(floor(y)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, x, floor(y),
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }

                                            if (fabs(ceil(y)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, x, ceil(y),
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else {
                                        int maxX = floor(127 * xS / yS);

                                        for (int x = ceil(xS); x <= maxX; x++) {
                                            double y = (double) x * (yS / xS);

                                            if (fabs(floor(y)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, x, floor(y),
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }

                                            if (fabs(ceil(y)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, x, ceil(y),
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                                else {
                                    if (yS < 0) {
                                        int maxY = ceil(-128 * yS / xS);

                                        for (int y = floor(yS); y <= maxY;
                                             y++) {
                                            double x = (double) y * (xS / yS);

                                            if (fabs(floor(x)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, floor(x), y,
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }

                                            if (fabs(ceil(x)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, ceil(x), y,
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else {
                                        int maxY = floor(127 * yS / xS);

                                        for (int y = ceil(yS); y <= maxY; y++) {
                                            double x = (double) y * (xS / yS);

                                            if (fabs(floor(x)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, floor(x), y,
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }

                                            if (fabs(ceil(x)) != 1.0) {
                                                if (test_stick_position(
                                                        solIdx, ceil(x), y,
                                                        endSpeed, vel1, xVel1,
                                                        zVel1, angle, cameraYaw,
                                                        startPosition,
                                                        oneUpPlatformPosition,
                                                        oneUpPlatformXMin,
                                                        oneUpPlatformXMax,
                                                        oneUpPlatformYMin,
                                                        oneUpPlatformYMax,
                                                        oneUpPlatformZMin,
                                                        oneUpPlatformZMax,
                                                        oneUpPlatformNormalX,
                                                        oneUpPlatformNormalY, f,
                                                        frame1Position,
                                                        returnPosition,
                                                        triangles, normals, d,
                                                        q)) {
                                                    foundSolution = true;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return foundSolution;
}

bool find_10k_route(int solIdx, int f, int d, int h) {
    struct PlatformSolution* sol =
        &(platSolutions[puSolutions[solIdx].platformSolutionIdx]);
    float returnSpeed         = puSolutions[solIdx].returnSpeed;
    float oneUpPlatformBuffer = 0.0f;

    bool foundSolution = false;

    float startPosition[3];
    startPosition[0] =
        ((float) startTriangles[f][0][0] + (float) startTriangles[f][1][0] +
         (float) startTriangles[f][2][0]) /
        3.0;
    startPosition[1] =
        ((float) startTriangles[f][0][1] + (float) startTriangles[f][1][1] +
         (float) startTriangles[f][2][1]) /
        3.0;
    startPosition[2] =
        ((float) startTriangles[f][0][2] + (float) startTriangles[f][1][2] +
         (float) startTriangles[f][2][2]) /
        3.0;

    float oneUpPlatformNormalY =
        (d == 0) ? oneUpPlatformNormalYRight : oneUpPlatformNormalYLeft;
    float oneUpPlatformNormalX =
        (d == 0) ? oneUpPlatformNormalXRight : oneUpPlatformNormalXLeft;
    float oneUpPlatformXMin =
        (d == 0) ? oneUpPlatformXMinRight : oneUpPlatformXMinLeft;
    float oneUpPlatformXMax =
        (d == 0) ? oneUpPlatformXMaxRight : oneUpPlatformXMaxLeft;
    float oneUpPlatformYMin =
        (d == 0) ? oneUpPlatformYMinRight : oneUpPlatformYMinLeft;
    float oneUpPlatformYMax =
        (d == 0) ? oneUpPlatformYMaxRight : oneUpPlatformYMaxLeft;
    float oneUpPlatformZMin =
        (d == 0) ? oneUpPlatformZMinRight : oneUpPlatformZMinLeft;
    float oneUpPlatformZMax =
        (d == 0) ? oneUpPlatformZMaxRight : oneUpPlatformZMaxLeft;

    double r = fabs((double) returnSpeed * (double) oneUpPlatformNormalY) / 4.0;

    int maxXPU =
        (int) floor((sol->returnPosition[0] + r - oneUpPlatformXMin) / 65536.0);
    maxXPU = (d == 0) ? min(0, maxXPU) : maxXPU;
    int minXPU =
        (int) ceil((sol->returnPosition[0] - r - oneUpPlatformXMax) / 65536.0);
    minXPU = (d == 0) ? minXPU : max(1, minXPU);

    for (int i = minXPU; i <= maxXPU; i++) {
        double signZ = h == 0 ? 1.0 : -1.0;
        double z0    = signZ *
                sqrt(r * r -
                     (i * 65536.0 + (oneUpPlatformXMin - oneUpPlatformBuffer) -
                      sol->returnPosition[0]) *
                         (i * 65536.0 +
                          (oneUpPlatformXMin - oneUpPlatformBuffer) -
                          sol->returnPosition[0])) +
            sol->returnPosition[2];
        double z1 = signZ *
                sqrt(r * r -
                     (i * 65536.0 + (oneUpPlatformXMax + oneUpPlatformBuffer) -
                      sol->returnPosition[0]) *
                         (i * 65536.0 +
                          (oneUpPlatformXMax + oneUpPlatformBuffer) -
                          sol->returnPosition[0])) +
            sol->returnPosition[2];

        int minZPU = (int) ceil(
            (fmin(z0, z1) - (oneUpPlatformZMax + oneUpPlatformBuffer)) /
            65536.0);
        int maxZPU = (int) floor(
            (fmax(z0, z1) - (oneUpPlatformZMin - oneUpPlatformBuffer)) /
            65536.0);

        for (int j = minZPU; j <= maxZPU; j++) {
            double signX = i > 0 ? 1.0 : -1.0;

            double x0 = signX *
                    sqrt(r * r -
                         (j * 65536.0 +
                          (oneUpPlatformZMin - oneUpPlatformBuffer) -
                          sol->returnPosition[2]) *
                             (j * 65536.0 +
                              (oneUpPlatformZMin - oneUpPlatformBuffer) -
                              sol->returnPosition[2])) +
                sol->returnPosition[0];
            double x1 = signX *
                    sqrt(r * r -
                         (j * 65536.0 +
                          (oneUpPlatformZMax + oneUpPlatformBuffer) -
                          sol->returnPosition[2]) *
                             (j * 65536.0 +
                              (oneUpPlatformZMax + oneUpPlatformBuffer) -
                              sol->returnPosition[2])) +
                sol->returnPosition[0];

            double minX = fmax(
                fmin(x0, x1),
                i * 65536.0 + (oneUpPlatformXMin - oneUpPlatformBuffer));
            double maxX = fmin(
                fmax(x0, x1),
                i * 65536.0 + (oneUpPlatformXMax + oneUpPlatformBuffer));

            double minXY = (double) (oneUpPlatformYMax - oneUpPlatformYMin) *
                    (minX - (65536.0 * i) - oneUpPlatformXMin) /
                    (double) (oneUpPlatformXMax - oneUpPlatformXMin) +
                oneUpPlatformYMin;
            double maxXY = (double) (oneUpPlatformYMax - oneUpPlatformYMin) *
                    (maxX - (65536.0 * i) - oneUpPlatformXMin) /
                    (double) (oneUpPlatformXMax - oneUpPlatformXMin) +
                oneUpPlatformYMin;

            double minXZ = signZ *
                    sqrt(r * r -
                         (minX - sol->returnPosition[0]) *
                             (minX - sol->returnPosition[0])) +
                sol->returnPosition[2];
            double maxXZ = signZ *
                    sqrt(r * r -
                         (maxX - sol->returnPosition[0]) *
                             (maxX - sol->returnPosition[0])) +
                sol->returnPosition[2];

            float oneUpPlatformPosition[3] = {
                (maxX + minX) / 2.0, (maxXY + minXY) / 2.0,
                (maxXZ + minXZ) / 2.0};

            if (test_one_up_position(
                    solIdx, startPosition, oneUpPlatformPosition,
                    sol->returnPosition, returnSpeed, oneUpPlatformXMin,
                    oneUpPlatformXMax, oneUpPlatformYMin, oneUpPlatformYMax,
                    oneUpPlatformZMin, oneUpPlatformZMax, oneUpPlatformNormalX,
                    oneUpPlatformNormalY, f, d, sol->endTriangles,
                    sol->endTriangleNormals)) {
                foundSolution = true;
            }
        }
    }

    return foundSolution;
}

void find_pu_slide_setup(struct PlatformSolution* sol, int solIdx) {
    float floorHeight;
    int floorIdx = find_floor(
        sol->endPosition, sol->endTriangles, sol->endTriangleNormals,
        &floorHeight);

    if (floorIdx != -1) {
        double s = (double) sol->pux / (double) sol->puz;

        float xVel1 =
            (float) (65536.0 * (double) sol->pux / (double) sol->endTriangleNormals[floorIdx][1]);
        float zVel1 =
            (float) (65536.0 * (double) sol->puz / (double) sol->endTriangleNormals[floorIdx][1]);

        for (int i = 0; i < 8192; i++) {
            int angle = gArctanTableG[i];
            angle     = (65536 + angle) % 65536;

            double t =
                tan(2.0 * M_PI * (double) (angle - (angle % 16)) / 65536.0);

            double n = (-(s * t) - 1.0 +
                        sqrt((s * t - 1.0) * (s * t - 1.0) + 4.0 * s * s)) /
                (2.0 * s);
            float nTestX =
                gCosineTableG[angle >> 4] + n * gSineTableG[angle >> 4];

            if ((xVel1 < 0 && nTestX > 0) || (xVel1 > 0 && nTestX < 0)) {
                n = (-(s * t) - 1.0 -
                     sqrt((s * t - 1.0) * (s * t - 1.0) + 4.0 * s * s)) /
                    (2.0 * s);
            }

            double n1 = n / 0.05;

            if (fabs(n1) <= 1.01) {
                int minAngle =
                    (int) round(65536.0 * asin(fabs(n1 / 1.01)) / (2 * M_PI));
                minAngle = minAngle + ((16 - (minAngle % 16)) % 16);

                for (int j = minAngle; j <= 32768 - minAngle; j += 16) {
                    int j1 = (n1 >= 0) ? j : (65536 - j) % 65536;

                    float idealMag = 32.0f * n1 / gSineTableG[j1 >> 4];

                    float mag = find_closest_mag(idealMag);

                    float vel0 =
                        (float) (sqrt((double) xVel1 * (double) xVel1 + (double) zVel1 * (double) zVel1) / ((double) (mag / 32.0f) * (double) gCosineTableG[j1 >> 4] * 0.02 + 0.92));
                    vel0 = (vel0 < 0) ? vel0 : -vel0;

                    float xVel0 = vel0 * gSineTableG[angle >> 4];
                    float zVel0 = vel0 * gCosineTableG[angle >> 4];

                    float xVel1a = xVel0;
                    float zVel1a = zVel0;

                    float oldSpeed = sqrt(xVel1a * xVel1a + zVel1a * zVel1a);

                    xVel1a +=
                        zVel1a * (mag / 32.0f) * gSineTableG[j1 >> 4] * 0.05f;
                    zVel1a -=
                        xVel1a * (mag / 32.0f) * gSineTableG[j1 >> 4] * 0.05f;

                    float newSpeed = sqrt(xVel1a * xVel1a + zVel1a * zVel1a);

                    xVel1a = xVel1a * oldSpeed / newSpeed;
                    zVel1a = zVel1a * oldSpeed / newSpeed;

                    xVel1a *=
                        mag / 32.0f * gCosineTableG[j1 >> 4] * 0.02f + 0.92f;
                    zVel1a *=
                        mag / 32.0f * gCosineTableG[j1 >> 4] * 0.02f + 0.92f;

                    float positionTest[3] = {
                        sol->endPosition[0], sol->endPosition[1],
                        sol->endPosition[2]};

                    for (int s = 0; s < 4; s++) {
                        positionTest[0] = positionTest[0] +
                            sol->endTriangleNormals[floorIdx][1] *
                                (xVel1a / 4.0f);
                        positionTest[2] = positionTest[2] +
                            sol->endTriangleNormals[floorIdx][1] *
                                (zVel1a / 4.0f);
                    }

                    int floorIdx1 = find_floor(
                        positionTest, sol->endTriangles,
                        sol->endTriangleNormals, &floorHeight);

                    if (floorIdx1 != -1 &&
                        fabs(positionTest[1] - floorHeight) < 4.0f) {
                        float prePositionTest[3] = {
                            sol->penultimatePosition[0] +
                                sol->penultimateFloorNormalY * xVel0 / 4.0f,
                            sol->penultimatePosition[1],
                            sol->penultimatePosition[2] +
                                sol->penultimateFloorNormalY * zVel0 / 4.0f};

                        if (!check_inbounds(prePositionTest)) {
                            int idx = atomicAdd(&nPUSolutions, 1);
                            if (idx < MAX_PU_SOLUTIONS) {
                                PUSolution solution;
                                solution.platformSolutionIdx = solIdx;
                                solution.returnSpeed         = vel0;
                                solution.angle               = angle;
                                solution.intendedDYaw        = j1;
                                solution.stickMag            = mag;
                                puSolutions[idx]             = solution;
                            }
                        }
                    }
                }
            }
        }
    }
}

void platform_logic(
    float* platform_normal, float* mario_pos, short (&triangles)[2][3][3],
    float (&normals)[2][3], float (&mat)[4][4], float* platform_pos) {
    float dx;
    float dy;
    float dz;
    float d;

    float dist[3];
    float posBeforeRotation[3];
    float posAfterRotation[3];

    // Mario's position
    float mx = mario_pos[0];
    float my = mario_pos[1];
    float mz = mario_pos[2];

    dist[0] = mx - (float) platform_pos[0];
    dist[1] = my - (float) platform_pos[1];
    dist[2] = mz - (float) platform_pos[2];

    mat[1][0] = platform_normal[0];
    mat[1][1] = platform_normal[1];
    mat[1][2] = platform_normal[2];

    float invsqrt = 1.0f /
        sqrt(mat[1][0] * mat[1][0] + mat[1][1] * mat[1][1] +
              mat[1][2] * mat[1][2]);

    mat[1][0] *= invsqrt;
    mat[1][1] *= invsqrt;
    mat[1][2] *= invsqrt;

    mat[0][0] = mat[1][1] * 1.0f - 0.0f * mat[1][2];
    mat[0][1] = mat[1][2] * 0.0f - 1.0f * mat[1][0];
    mat[0][2] = mat[1][0] * 0.0f - 0.0f * mat[1][1];

    invsqrt = 1.0f /
        sqrt(mat[0][0] * mat[0][0] + mat[0][1] * mat[0][1] +
              mat[0][2] * mat[0][2]);

    mat[0][0] *= invsqrt;
    mat[0][1] *= invsqrt;
    mat[0][2] *= invsqrt;

    mat[2][0] = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
    mat[2][1] = mat[0][2] * mat[1][0] - mat[1][2] * mat[0][0];
    mat[2][2] = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];

    invsqrt = 1.0f /
        sqrt(mat[2][0] * mat[2][0] + mat[2][1] * mat[2][1] +
              mat[2][2] * mat[2][2]);

    mat[2][0] *= invsqrt;
    mat[2][1] *= invsqrt;
    mat[2][2] *= invsqrt;

    mat[3][0] = platform_pos[0];
    mat[3][1] = platform_pos[1];
    mat[3][2] = platform_pos[2];
    mat[0][3] = 0.0f;
    mat[1][3] = 0.0f;
    mat[2][3] = 0.0f;
    mat[3][3] = 1.0f;

    for (int i = 0; i < 3; i++) {
        posBeforeRotation[i] =
            mat[0][i] * dist[0] + mat[1][i] * dist[1] + mat[2][i] * dist[2];
    }

    dx = mx - (float) platform_pos[0];
    dy = 500.0f;
    dz = mz - (float) platform_pos[2];
    d  = sqrt(dx * dx + dy * dy + dz * dz);

    // Normalizing
    d = 1.0 / d;
    dx *= d;
    dy *= d;
    dz *= d;

    // Approach the normals by 0.01f towards the new goal, then create a
    // transform matrix and orient the object. Outside of the other conditionals
    // since it needs to tilt regardless of whether Mario is on.
    platform_normal[0] = (platform_normal[0] <= dx) ?
        ((dx - platform_normal[0] < 0.01f) ? dx :
                                             (platform_normal[0] + 0.01f)) :
        ((dx - platform_normal[0] > -0.01f) ? dx :
                                              (platform_normal[0] - 0.01f));
    platform_normal[1] = (platform_normal[1] <= dy) ?
        ((dy - platform_normal[1] < 0.01f) ? dy :
                                             (platform_normal[1] + 0.01f)) :
        ((dy - platform_normal[1] > -0.01f) ? dy :
                                              (platform_normal[1] - 0.01f));
    platform_normal[2] = (platform_normal[2] <= dz) ?
        ((dz - platform_normal[2] < 0.01f) ? dz :
                                             (platform_normal[2] + 0.01f)) :
        ((dz - platform_normal[2] > -0.01f) ? dz :
                                              (platform_normal[2] - 0.01f));

    mat[1][0] = platform_normal[0];
    mat[1][1] = platform_normal[1];
    mat[1][2] = platform_normal[2];

    invsqrt = 1.0f /
        sqrt(mat[1][0] * mat[1][0] + mat[1][1] * mat[1][1] +
              mat[1][2] * mat[1][2]);

    mat[1][0] *= invsqrt;
    mat[1][1] *= invsqrt;
    mat[1][2] *= invsqrt;

    mat[0][0] = mat[1][1] * 1.0f - 0.0f * mat[1][2];
    mat[0][1] = mat[1][2] * 0.0f - 1.0f * mat[1][0];
    mat[0][2] = mat[1][0] * 0.0f - 0.0f * mat[1][1];

    invsqrt = 1.0f /
        sqrt(mat[0][0] * mat[0][0] + mat[0][1] * mat[0][1] +
              mat[0][2] * mat[0][2]);

    mat[0][0] *= invsqrt;
    mat[0][1] *= invsqrt;
    mat[0][2] *= invsqrt;

    mat[2][0] = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
    mat[2][1] = mat[0][2] * mat[1][0] - mat[1][2] * mat[0][0];
    mat[2][2] = mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1];

    invsqrt = 1.0f /
        sqrt(mat[2][0] * mat[2][0] + mat[2][1] * mat[2][1] +
              mat[2][2] * mat[2][2]);

    mat[2][0] *= invsqrt;
    mat[2][1] *= invsqrt;
    mat[2][2] *= invsqrt;

    mat[3][0] = platform_pos[0];
    mat[3][1] = platform_pos[1];
    mat[3][2] = platform_pos[2];
    mat[0][3] = 0.0f;
    mat[1][3] = 0.0f;
    mat[2][3] = 0.0f;
    mat[3][3] = 1.0f;

    for (int i = 0; i < 3; i++) {
        posAfterRotation[i] =
            mat[0][i] * dist[0] + mat[1][i] * dist[1] + mat[2][i] * dist[2];
    }

    mx += posAfterRotation[0] - posBeforeRotation[0];
    my += posAfterRotation[1] - posBeforeRotation[1];
    mz += posAfterRotation[2] - posBeforeRotation[2];
    mario_pos[0] = mx;
    mario_pos[1] = my;
    mario_pos[2] = mz;
}
