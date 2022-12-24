#include "Platform.hpp"
#include <cmath>

/**
 * Returns a value that is src incremented/decremented by inc towards goal
 * until goal is reached. Does not overshoot.
 */
float approach_by_increment(float goal, float src, float inc) {
	float newVal;

	if (src <= goal) {
		if (goal - src < inc) {
			newVal = goal;
		}
		else {
			newVal = src + inc;
		}
	}
	else if (goal - src > -inc) {
		newVal = goal;
	}
	else {
		newVal = src - inc;
	}

	return newVal;
}

/**
 * Creates a transform matrix on a variable passed in from given normals
 * and the object's position.
 */
void Platform::create_transform_from_normals() {
	mtxf_align_terrain_normal(transform, normal, pos, 0);
}

void Platform::platform_logic(Vec3f& marioPos) {
	float dx;
	float dy;
	float dz;
	float d;

	Vec3f dist;
	Vec3f posBeforeRotation;
	Vec3f posAfterRotation;

	//create_transform_from_normals();

	// Mario's position
	float mx;
	float my;
	float mz;

	int marioOnPlatform = 0;

	// pretty sure you can always assume if here, then mario is on the floor
	create_transform_from_normals();

	if (onPlatform)
	{
		mx = marioPos[0];
		my = marioPos[1];
		mz = marioPos[2];

		dist[0] = mx - (float)pos[0];
		dist[1] = my - (float)pos[1];
		dist[2] = mz - (float)pos[2];
		linear_mtxf_mul_vec3f(posBeforeRotation, transform, dist);

		dx = mx - (float)pos[0];
		dy = 500.0f;
		dz = mz - (float)pos[2];
		d = sqrtf(dx * dx + dy * dy + dz * dz);

		//! Always true since dy = 500, making d >= 500.
		if (d != 0.0f) {
			// Normalizing
			d = 1.0 / d;
			dx *= d;
			dy *= d;
			dz *= d;
		}
		else {
			dx = 0.0f;
			dy = 1.0f;
			dz = 0.0f;
		}

		if (oTiltingPyramidMarioOnPlatform == true)
			marioOnPlatform++;
		oTiltingPyramidMarioOnPlatform = true;
	}
	else
	{
		dx = 0.0f;
		dy = 1.0f;
		dz = 0.0f;
		oTiltingPyramidMarioOnPlatform = false;
	}

	// Approach the normals by 0.01f towards the new goal, then create a transform matrix and orient the object. 
	// Outside of the other conditionals since it needs to tilt regardless of whether Mario is on.
	normal[0] = approach_by_increment(dx, normal[0], 0.01f);
	normal[1] = approach_by_increment(dy, normal[1], 0.01f);
	normal[2] = approach_by_increment(dz, normal[2], 0.01f);
	create_transform_from_normals();

	triangles[0].rotate(transform);
	triangles[1].rotate(transform);

	ceilings[0].rotate(transform);
	ceilings[1].rotate(transform);
	ceilings[2].rotate(transform);
	ceilings[3].rotate(transform);

	// If Mario is on the platform, adjust his position for the platform tilt.
	if (marioOnPlatform) {
		linear_mtxf_mul_vec3f(posAfterRotation, transform, dist);
		mx += posAfterRotation[0] - posBeforeRotation[0];
		my += posAfterRotation[1] - posBeforeRotation[1];
		mz += posAfterRotation[2] - posBeforeRotation[2];
		marioPos[0] = mx;
		marioPos[1] = my;
		marioPos[2] = mz;
	}

	float floor_height = 0.0;
	Surface const* floor = find_floor(marioPos, triangles, &floor_height);
	onPlatform = floor && fabsf(marioPos[1] - floor_height) <= 4.0;
}