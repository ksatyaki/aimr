#include <aimr/fuzzy_velocity.h>

FuzzyVelocity::FuzzyVelocity (int l_set_size, int r_set_size)
{
	for(int i = 0; i < l_set_size; i++)
		linear_velocity.push_back(0.0);
	for(int i = 0; i < r_set_size; i++)
		rotational_velocity.push_back(0.0);

	rotational_vel_ratio = 0.0;
	linear_vel_ratio = 0.0;
}

void FuzzyVelocity::setRotationalVelocity (int rot)
{
	//Set rotation speed
	rotational_velocity[rot] = MAX(rotational_velocity[rot], fuzzy::ante);
}

void FuzzyVelocity::setLinearVelocity (int lin)
{
	//Set linear speed
	linear_velocity[lin] = MAX(linear_velocity[lin], fuzzy::ante);
}

void FuzzyVelocity::runFuzzyBehaviour(fuzzy::FBehaviour behaviour)
{
	resetFuzzySets();

	behaviour();

	deFuzzifyVelocity();
}

void FuzzyVelocity::printSets()
{
	printf("\nThe linear set is: ");
	for(int i = 0; i < linear_velocity.size(); i++)
	{
		printf(" %lf,", linear_velocity[i]);
	}
	printf("\n");
	printf("\nThe angular set is: ");
	for(int i = 0; i < rotational_velocity.size(); i++)
	{
		printf(" %lf,", rotational_velocity[i]);
	}
}

void FuzzyVelocity::resetFuzzySets()
{
	for(int i = 0; i < rotational_velocity.size(); i++)
		rotational_velocity[i] = 0.0;

	for(int i = 0; i < linear_velocity.size(); i++)
		linear_velocity[i] = 0.0;
}

void FuzzyVelocity::deFuzzifyVelocity()
{
	linear_vel_ratio = fuzzy::DeFuzzify(linear_velocity);
	rotational_vel_ratio = fuzzy::DeFuzzify(rotational_velocity) - 2.0;
}
