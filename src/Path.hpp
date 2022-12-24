#pragma once
#include "vmath.hpp"
#include "Platform.hpp"
#include "BoundaryDistance.hpp"

#ifndef PATH_H
#define PATH_H

class Path
{
public:
	Vec3f StartMarioPos;
	Vec3f StartNormal;
	Vec3f BestNormal;
	int BestFrame = -1;
	Vec3f BestMarioPos;
	int CurrentFrame = 0;
	Vec3f CurrentMarioPos;
	float BestBoundaryDist = -10.0f;

	Platform platform;
	BoundaryDistance* boundaryDistance;

	Path(Vec3f startMarioPos, Vec3f startNormal, BoundaryDistance* boundaryDistance);

	void update();
};

#endif