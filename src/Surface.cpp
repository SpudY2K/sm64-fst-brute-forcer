#include "Surface.hpp"

void Surface::rotate(const Mat4& m) {
	reset_vectors();

	for (int i = 0; i < 3; i++) {
		float vx = vectors[i][0];
		float vy = vectors[i][1];
		float vz = vectors[i][2];

		vectors[i][0] = (short)(int)(vx * m[0][0] + vy * m[1][0] + vz * m[2][0] + m[3][0]);
		vectors[i][1] = (short)(int)(vx * m[0][1] + vy * m[1][1] + vz * m[2][1] + m[3][1]);
		vectors[i][2] = (short)(int)(vx * m[0][2] + vy * m[1][2] + vz * m[2][2] + m[3][2]);
	}

	normal[0] = ((vectors[1][1] - vectors[0][1]) * (vectors[2][2] - vectors[1][2])) - ((vectors[1][2] - vectors[0][2]) * (vectors[2][1] - vectors[1][1]));
	normal[1] = ((vectors[1][2] - vectors[0][2]) * (vectors[2][0] - vectors[1][0])) - ((vectors[1][0] - vectors[0][0]) * (vectors[2][2] - vectors[1][2]));
	normal[2] = ((vectors[1][0] - vectors[0][0]) * (vectors[2][1] - vectors[1][1])) - ((vectors[1][1] - vectors[0][1]) * (vectors[2][0] - vectors[1][0]));

	vec3f_normalize(normal);

	originOffset = -(normal[0] * vectors[0][0] + normal[1] * vectors[0][1] + normal[2] * vectors[0][2]);
}

void Surface::repr() {
	printf("vector1: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[0][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}

	printf("vector2: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[1][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}

	printf("vector3: (");

	for (int i = 0; i < 3; i++) {
		printf("%d", vectors[2][i]);

		if (i != 2) {
			printf(", ");
		}
		else {
			printf(")\n");
		}
	}
}

Surface const* find_floor(Vec3f& pos, Vec2S& triangles, float* pheight) {
	Surface const* floor = NULL;

	int16_t x = static_cast<int16_t>(static_cast<int>(pos[0]));
	int16_t y = static_cast<int16_t>(static_cast<int>(pos[1]));
	int16_t z = static_cast<int16_t>(static_cast<int>(pos[2]));

	for (int i = 0; i < 2; i++) {
		const Surface& surf = triangles[i];

		int16_t x1 = surf.vectors[0][0];
		int16_t z1 = surf.vectors[0][2];
		int16_t x2 = surf.vectors[1][0];
		int16_t z2 = surf.vectors[1][2];

		// Check that the point is within the triangle bounds.
		if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
			continue;
		}

		// To slightly save on computation time, set this later.
		int16_t x3 = surf.vectors[2][0];
		int16_t z3 = surf.vectors[2][2];

		if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
			continue;
		}
		if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
			continue;
		}

		float nx = surf.normal[0];
		float ny = surf.normal[1];
		float nz = surf.normal[2];
		float oo = -(nx * x1 + ny * surf.vectors[0][1] + nz * z1);

		// Find the height of the floor at a given location.
		float height = -(x * nx + nz * z + oo) / ny;
		// Checks for floor interaction with a 78 unit buffer.
		if (y - (height + -78.0f) < 0.0f) {
			continue;
		}

		*pheight = height;
		floor = &surf;
		break;
	}

	//! (Surface Cucking) Since only the first floor is returned and not the highest,
	//  higher floors can be "cucked" by lower floors.
	return floor;
}