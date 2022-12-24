#include "Trig.hpp"
#include "vmath.hpp"
#include <cmath>

float euclidean_distance(Vec3f& a, Vec3f& b) {
	float dx = a[0] - b[0], dz = a[2] - b[2];
	return sqrtf(dx * dx + dz * dz);
}
float euclidean_distance_squared(Vec3f& a, Vec3f& b) {
	float dx = a[0] - b[0], dz = a[2] - b[2];
	return dx * dx + dz * dz;
}

void create_transform_from_normal(Vec3f& normal, Vec3f& position, Mat4& mat) {
	mtxf_align_terrain_normal(mat, normal, position, 0);
}

void apply_tform(VecVec3f& vertices, Mat4& mat, VecVec3f& tform_vertices) {
	for (int i = 0; i < 3; i++) {
		tform_vertices[i][0] = (short)(int)(vertices[i][0] * mat[0][0] + vertices[i][1] * mat[1][0] + vertices[i][2] * mat[2][0] + mat[3][0]);
		tform_vertices[i][1] = (short)(int)(vertices[i][0] * mat[0][1] + vertices[i][1] * mat[1][1] + vertices[i][2] * mat[2][1] + mat[3][1]);
		tform_vertices[i][2] = (short)(int)(vertices[i][0] * mat[0][2] + vertices[i][1] * mat[1][2] + vertices[i][2] * mat[2][2] + mat[3][2]);
	}
}

/**
 * Multiply a vector by a matrix of the form
 * | ? ? ? 0 |
 * | ? ? ? 0 |
 * | ? ? ? 0 |
 * | 0 0 0 1 |
 * i.e. a matrix representing a linear transformation over 3 space.
 */
void linear_mtxf_mul_vec3f(Vec3f& dst, const Mat4& m, const Vec3f& v) {
	for (int i = 0; i < 3; i++) {
		dst[i] = m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2];
	}
}

/// Set vector 'dest' to (x, y, z)
void vec3f_set(Vec3f& dst, float x, float y, float z) {
	dst[0] = x;
	dst[1] = y;
	dst[2] = z;
}

/// Make vector 'dest' the cross product of vectors a and b.
void vec3f_cross(Vec3f& dest, const Vec3f& a, const Vec3f& b) {
	dest[0] = a[1] * b[2] - b[1] * a[2];
	dest[1] = a[2] * b[0] - b[2] * a[0];
	dest[2] = a[0] * b[1] - b[0] * a[1];
}

/// Scale vector 'dest' so it has length 1
void vec3f_normalize(Vec3f& dest) {
	//! Possible division by zero
	float invsqrt = 1.0f / sqrtf(dest[0] * dest[0] + dest[1] * dest[1] + dest[2] * dest[2]);

	dest[0] *= invsqrt;
	dest[1] *= invsqrt;
	dest[2] *= invsqrt;
}

/**
 * Set 'dest' to a transformation matrix that aligns an object with the terrain
 * based on the normal. Used for enemies.
 * 'upDir' is the terrain normal
 * 'yaw' is the angle which it should face
 * 'pos' is the object's position in the world
 */
void mtxf_align_terrain_normal(Mat4& dest, Vec3f upDir, const Vec3f& pos, int16_t yaw) {
	Vec3f lateralDir;
	Vec3f leftDir;
	Vec3f forwardDir;

	vec3f_set(lateralDir, gSineTable[yaw], 0, gCosineTable[yaw]);
	vec3f_normalize(upDir);

	vec3f_cross(leftDir, upDir, lateralDir);
	vec3f_normalize(leftDir);

	vec3f_cross(forwardDir, leftDir, upDir);
	vec3f_normalize(forwardDir);

	dest[0][0] = leftDir[0];
	dest[0][1] = leftDir[1];
	dest[0][2] = leftDir[2];
	dest[3][0] = pos[0];

	dest[1][0] = upDir[0];
	dest[1][1] = upDir[1];
	dest[1][2] = upDir[2];
	dest[3][1] = pos[1];

	dest[2][0] = forwardDir[0];
	dest[2][1] = forwardDir[1];
	dest[2][2] = forwardDir[2];
	dest[3][2] = pos[2];

	dest[0][3] = 0.0f;
	dest[1][3] = 0.0f;
	dest[2][3] = 0.0f;
	dest[3][3] = 1.0f;
}