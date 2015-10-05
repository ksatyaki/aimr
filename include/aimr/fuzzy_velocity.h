#ifndef FUZZY_VELOCITY_H_
#define FUZZY_VELOCITY_H_

#include <aimr/fuzzy.h>
#include <cmath>

class FuzzyVelocity
{
protected:
	fuzzy::Set rotational_velocity;
	fuzzy::Set linear_velocity;

	float linear_vel_ratio;
	float rotational_vel_ratio;

public:
	FuzzyVelocity(int l_set_size, int r_set_size);

	inline float getLinear() { return linear_vel_ratio; }
	inline float getRotational() { return rotational_vel_ratio; }

	void printSets();

	void runFuzzyBehaviour(fuzzy::FBehaviour behaviour);
	void resetFuzzySets();
	void deFuzzifyVelocity();
	void setRotationalVelocity(int rot);
	void setLinearVelocity(int lin);
};

#endif
