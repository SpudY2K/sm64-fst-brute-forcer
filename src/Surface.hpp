#pragma once

#include <sstream>
#include <stdexcept>
#include <vector>
#include <cstdio>
#include <cstdint>

#include "vmath.hpp"

using namespace std;

#ifndef SURFACE_H
#define SURFACE_H

class Surface
{
public:
	VecVec3s vectors;
	Vec3f normal;
	float originOffset;
	int plat_idx;
	int idx;

	Surface(int idx) {
		this->idx = idx;

		reset_vectors();

		normal[0] = 0.0f;
		normal[1] = 1.0f;
		normal[2] = 0.0f;

		originOffset = -vectors[0][1];
	}

	void reset_vectors() {
		if (idx == 0) {
			vectors[0][0] = 307;
			vectors[0][1] = 307;
			vectors[0][2] = -306;
			vectors[1][0] = -306;
			vectors[1][1] = 307;
			vectors[1][2] = -306;
			vectors[2][0] = -306;
			vectors[2][1] = 307;
			vectors[2][2] = 307;
		}
		else if (idx == 1) {
			vectors[0][0] = 307;
			vectors[0][1] = 307;
			vectors[0][2] = -306;
			vectors[1][0] = -306;
			vectors[1][1] = 307;
			vectors[1][2] = 307;
			vectors[2][0] = 307;
			vectors[2][1] = 307;
			vectors[2][2] = 307;
		}
		else if (idx == 2) {
			vectors[0][0] = -306;
			vectors[0][1] = 307;
			vectors[0][2] = 307;
			vectors[1][0] = -306;
			vectors[1][1] = 307;
			vectors[1][2] = -306;
			vectors[2][0] = 0;
			vectors[2][1] = 0;
			vectors[2][2] = 0;
		}
		else if (idx == 3) {
			vectors[0][0] = 0;
			vectors[0][1] = 0;
			vectors[0][2] = 0;
			vectors[1][0] = 307;
			vectors[1][1] = 307;
			vectors[1][2] = 307;
			vectors[2][0] = -306;
			vectors[2][1] = 307;
			vectors[2][2] = 307;
		}
		else if (idx == 4) {
			vectors[0][0] = 307;
			vectors[0][1] = 307;
			vectors[0][2] = -306;
			vectors[1][0] = 0;
			vectors[1][1] = 0;
			vectors[1][2] = 0;
			vectors[2][0] = -306;
			vectors[2][1] = 307;
			vectors[2][2] = -306;
		}
		else if (idx == 5) {
			vectors[0][0] = 0;
			vectors[0][1] = 0;
			vectors[0][2] = 0;
			vectors[1][0] = 307;
			vectors[1][1] = 307;
			vectors[1][2] = -306;
			vectors[2][0] = 307;
			vectors[2][1] = 307;
			vectors[2][2] = 307;
		}
		else {
			std::ostringstream oss;
      oss << "Unsupported surface " << idx;
      throw std::runtime_error(oss.str());
		}
	}

	void rotate(const Mat4& m);
	void repr();
};

typedef std::array<Surface, 2> Vec2S;

Surface const* find_floor(Vec3f& pos, Vec2S& triangles, float* height);

#endif