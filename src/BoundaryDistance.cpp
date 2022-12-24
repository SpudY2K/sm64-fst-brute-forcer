#include "BoundaryDistance.hpp"
#include <cmath>
#include <iostream>
#include <vector>

#define M_PI 3.14159265358979323846 /* pi */

// Thanks to Modiseus for most of the logic in this class
BoundaryDistance::BoundaryDistance() {
  listOfLists = prepareContourForWRange();
}

// This is based on https://www.desmos.com/calculator/xwfp0o18iy
// I have combined the 8 equations by adding the variables a,b,c,d and sign.
// After that I removed the fractions and brought everything to the left
// side to get a equation of the form f(x,y,w)=0.
// A part of this tranformation can be found here:
// https://www.desmos.com/calculator/eodn6ddpsb Here I have only added cases for
// the two curves with positive x and y. Since the function is symmetrical it is
// enough to look at positive values of x and y.
float BoundaryDistance::calculateFunction(Vec3f normal, int type) {
  float a;
  float b;
  float c;
  float d;
  float sign;

  float offset = 0.01;

  float v = normal[1] - offset;

  switch (type) {
  case 0:
    a    = normal[0] - offset;
    b    = normal[2] - offset;
    c    = normal[2];
    d    = sqrtf(1 - normal[2] * normal[2]);
    sign = 1;

    break;
  case 1:
    a    = normal[0] - offset;
    b    = normal[2] - offset;
    c    = sqrtf(1 - normal[0] * normal[0]);
    d    = normal[0];
    sign = -1;

    break;
  default:
    throw "Unsupported type " + type;
  }

  float sqrt1 = sqrtf(a * a + v * v);
  float sqrt2 = sqrtf(a * a + b * b + v * v);
  float sqrt3 = sqrtf(normal[1] * normal[1] + normal[0] * normal[0]);
  float sqrt4 = sqrtf(
    normal[1] * normal[1] + normal[0] * normal[0] + normal[2] * normal[2]);

  float result = sign * d * sqrt1 * sqrt3 *
    (d * sqrt2 * (sqrt1 * normal[0] - a * sqrt3) * sqrt4 +
     c * (-sqrt1 * sqrt2 * normal[1] * normal[2] + b * v * sqrt3 * sqrt4));

  return result;
}

bool BoundaryDistance::isPositiveAt(Vec3f normal) {
  // Combine the two functions and make sure that x and y are below w.
  return calculateFunction(normal, 0) >= 0 &&
    calculateFunction(normal, 1) >= 0 && normal[0] <= normal[1] &&
    normal[2] <= normal[1];
}

bool BoundaryDistance::areValuesDifferent(
  bool bool1, bool bool2, bool bool3, bool bool4) {
  bool allTrue  = bool1 && bool2 && bool3 && bool4;
  bool allFalse = !bool1 && !bool2 && !bool3 && !bool4;
  return !allTrue && !allFalse;
}

float BoundaryDistance::gridWToW(int gridW) {
  return minW + gridW * (maxW - minW) / (float) (wSamples);
}

// Thanks to SpudY2K for this method
std::vector<BoundaryDistance::Point> BoundaryDistance::prepareContour(
  float normal_y) {
  std::vector<Point> contourPoints;

  const int nPoints = 1000;

  float maxR     = 1.0;
  float startInc = 0.5;
  float minInc   = 0.0;

  float* cosTable = (float*) malloc(nPoints * sizeof(float));
  float* sinTable = (float*) malloc(nPoints * sizeof(float));

#pragma omp parallel for schedule(dynamic, 1)
  for (int i = 0; i < nPoints; i++) {
    float angle = 0.5 * M_PI * (float) (nPoints - i) / (float) (nPoints + 1);
    cosTable[i] = cos(angle);
    sinTable[i] = sin(angle);
  }

  float* radii = (float*) malloc(nPoints * sizeof(float));
  float* rInc  = (float*) malloc(nPoints * sizeof(float));

#pragma omp parallel for schedule(dynamic, 1)
  for (int i = 0; i < nPoints; i++) {
    radii[i] = 0.0;
    rInc[i]  = startInc;
  }

  bool converge = false;

  while (!converge) {
    converge = true;

#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < nPoints; i++) {
      if (rInc[i] > 0) {
        float newR = radii[i] + rInc[i];

        float x0 = (float) (newR * cosTable[i]);
        float z0 = (float) (newR * sinTable[i]);

        if (!isPositiveAt(Vec3f {x0, normal_y, z0})) {
          rInc[i] = rInc[i] / 2.0;

          if (rInc[i] < minInc || (radii[i] + rInc[i]) == radii[i]) {
            rInc[i] = 0.0;
          }
          else {
            converge = false;
          }
        }
        else {
          radii[i] = newR;

          if (radii[i] >= maxR) {
            rInc[i] = 0.0;
          }

          converge = false;
        }
      }
    }
  }

  for (int i = 0; i < nPoints; i++) {
    float nx = (float) (radii[i] * cosTable[i]);
    float nz = (float) (radii[i] * sinTable[i]);

    Point point = {nx, nz};

    contourPoints.push_back(point);
  }

  return contourPoints;
}

float BoundaryDistance::calculateDistance(
  const std::vector<BoundaryDistance::Point>& contourPoints, Vec3f normal) {
  // The function is symmetric. It is therefore enough to look at the absolute
  // values.
  normal[0] = abs(normal[0]);
  normal[2] = abs(normal[2]);

  float minDist = INFINITY;

  long index        = 0;
  long size         = contourPoints.size();
  long increment    = sqrt(size) + 1;
  float prevDist    = -1.0;
  float distDir     = 0;
  float currDistDir = 0;
  int minIndex      = 0;

  // find a point close to the true min
  for (index; index < size; index += increment) {
    float dx = normal[0] - contourPoints[index].x;
    float dz = normal[2] - contourPoints[index].z;

    float dist = std::sqrt(dx * dx + dz * dz);

    if (dist < minDist) {
      minDist  = dist;
      minIndex = index;
    }
  }

  // Check between the adjacent previous indices for the "true" min
  long firstIndex = minIndex - increment;
  if (firstIndex < 0)
    firstIndex = 0;
  long lastIndex = minIndex + increment;
  if (lastIndex >= size)
    lastIndex = size - 1;

  for (index = firstIndex; index <= lastIndex; index += 1) {
    float dx = normal[0] - contourPoints[index].x;
    float dz = normal[2] - contourPoints[index].z;

    float dist = std::sqrt(dx * dx + dz * dz);

    if (dist < minDist) {
      minDist  = dist;
      minIndex = index;
    }
  }

  // Positive values are valid for upwarp given sufficient speed
  minDist *= isPositiveAt(normal) ? -1.0 : 1.0;

  return minDist;
}

std::vector<std::pair<float, std::vector<BoundaryDistance::Point>>>
BoundaryDistance::prepareContourForWRange() {
  std::vector<std::pair<float, std::vector<Point>>> listOfLists;

  for (int w = 0; w < wSamples + 1; w++) {
    float gridW             = gridWToW(w);
    std::vector<Point> list = prepareContour(gridW);
    listOfLists.emplace_back(gridW, std::move(list));
  }

  return listOfLists;
}

const std::vector<BoundaryDistance::Point>& BoundaryDistance::getClosestWList(
  const std::vector<std::pair<float, std::vector<Point>>>& listOfLists,
  float normal_y) {
  if (listOfLists.size() == 1)
    return listOfLists[0].second;

  long index = lround((normal_y - minW) * (float) wSamples / (maxW - minW));

  if (index < 0)
    index = 0;
  else if (index > wSamples)
    index = wSamples;

  return listOfLists[index].second;
}

float BoundaryDistance::getDistance(Vec3f normal) {
  // std::vector<Point> contourPoints = prepareContour(normal_y);
  std::vector<Point> contourPoints = getClosestWList(listOfLists, normal[1]);

  float dist = calculateDistance(contourPoints, normal);

  // std::cout << dist << std::endl;

  return dist;
}

void BoundaryDistance::testWithWRange() {
  Vec3f normal = Vec3f {0.4, 0.75, 0.5};

  std::vector<std::pair<float, std::vector<Point>>> listOfLists =
    prepareContourForWRange();

  std::vector<Point> list = getClosestWList(listOfLists, normal[1]);

  float dist = calculateDistance(list, normal);

  std::cout << dist << std::endl;
}
