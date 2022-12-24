#include "Path.hpp"
#include <iostream>
#include "BoundaryDistance.hpp"

Path::Path(Vec3f startMarioPos, Vec3f startNormal, BoundaryDistance* boundaryDistance)
{
	BestMarioPos = StartMarioPos = CurrentMarioPos = startMarioPos;
	BestNormal = StartNormal = startNormal;

	this->boundaryDistance = boundaryDistance;

	platform = Platform(-1945.0f, -3225.0f, -715.0f, startNormal);
	platform.platform_logic(startMarioPos);

	float floor_height = 0.0;
	Surface const* floor = find_floor(startMarioPos, platform.triangles, &floor_height);

	platform = Platform(-1945.0f, -3225.0f, -715.0f, startNormal);

	if (floor && floor_height > -3071.0f)
	{
		StartMarioPos[1] = floor_height;
		CurrentMarioPos[1] = floor_height;
	}
	else
		CurrentFrame = -1;
}

void Path::update()
{
	CurrentFrame++;

	platform.platform_logic(CurrentMarioPos);
	float floor_height = 0.0;
	Surface const* floor = find_floor(CurrentMarioPos, platform.triangles, &floor_height);

	//Check if Mario is under the lava, or too far below the platform for it to conceivably be in reach later
	if (CurrentMarioPos[1] <= -3071.0f && (!floor || floor_height <= -3071.0f)
		|| (floor != NULL && CurrentMarioPos[1] - floor_height < -20.0f))
	{
		CurrentFrame = -1;
		return;
	}

	//This is the metric we use to determine how good the path is. Higher generally = better, and it must be above 0 to get an upwarp
	float boundaryDist = boundaryDistance->getDistance(platform.normal);

	if (boundaryDist >= BestBoundaryDist)
	{
		BestBoundaryDist = boundaryDist;
		BestMarioPos = CurrentMarioPos;
		BestNormal = platform.normal;
		BestFrame = CurrentFrame;
	}
}