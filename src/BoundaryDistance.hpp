#pragma once

#ifndef BOUNDARYDISTANCE_H
#define BOUNDARYDISTANCE_H

#include <iostream>
#include <vector>
#include <cmath>
#include "vmath.hpp"

//Thanks to Modiseus for this
class BoundaryDistance
{
public:
	int wSamples = 1000;

	float minX = 0.0f;//std::nextafter(0.0, INFINITY);
	float maxX = 1.0f;

	float minY = 0.0f;
	float maxY = 1.0f;

	float minW = 0.7f;
	float maxW = 1.0f;

	struct Point
	{
		float x;
		float z;
	};

	std::vector<std::pair<float, std::vector<Point>>> listOfLists;

	BoundaryDistance();

	float calculateFunction(Vec3f normal, int type);

	bool isPositiveAt(Vec3f normal);

	bool areValuesDifferent(bool bool1, bool bool2, bool bool3, bool bool4);

	float gridWToW(int gridW);

	std::vector<Point> prepareContour(float w);

	float calculateDistance(const std::vector<Point>& contourPoints, Vec3f normal);

	std::vector<std::pair<float, std::vector<Point>>> prepareContourForWRange();

	const std::vector<Point>& getClosestWList(const std::vector<std::pair<float, std::vector<Point>>>& listOfLists, float w);

	float getDistance(Vec3f normal);

	void testWithWRange();
};

#endif