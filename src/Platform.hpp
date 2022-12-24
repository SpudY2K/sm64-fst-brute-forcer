#pragma once

#include <vector>
#include "Surface.hpp"
#include "vmath.hpp"

using namespace std;

#ifndef PLATFORM_H
#define PLATFORM_H

class Platform
{
public:
	Vec3f pos = { 0, 0, 0 };
	Vec3f normal = { 0, 1, 0 };
	Mat4 transform = { {} };
	bool oTiltingPyramidMarioOnPlatform = false;
	bool onPlatform = false;
	Vec2S triangles = { Surface(0), Surface(1) };
	std::array<Surface, 4> ceilings = { Surface(2), Surface(3), Surface(4), Surface(5) };

	Platform() { };

	Platform(int16_t x, int16_t y, int16_t z, Vec3f startNormal) {
		pos[0] = x;
		pos[1] = y;
		pos[2] = z;
		normal = startNormal;

		create_transform_from_normals();

		triangles[0].rotate(transform);
		triangles[1].rotate(transform);

		ceilings[0].rotate(transform);
		ceilings[1].rotate(transform);
		ceilings[2].rotate(transform);
		ceilings[3].rotate(transform);
	}

	void create_transform_from_normals();
	void platform_logic(Vec3f& marioPos);
};

float approach_by_increment(float goal, float src, float inc);

#endif