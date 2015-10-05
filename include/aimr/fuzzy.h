/**
 * fuzzy.h
 */
#ifndef fuzzy_h_DEFINED
#define fuzzy_h_DEFINED

#include <cmath>
#include <stdio.h>
#include <vector>

#define DEFBEHAVIOUR(x) void x (void)
#define F_SET_SIZE 4

#define RULESET
#define RULEEND

#define MIN(x,y)      x > y ? y : x
#define MAX(x,y)      x < y ? y : x

namespace fuzzy
{

typedef double Predicate;
typedef void (*FBehaviour) (void);
typedef std::vector<double> Set;

extern Predicate ante;

/**
 * Creates a rising ramp with value 0 at Start and 1 at End.
 */
Predicate RampUp(const double& Value, const double& RampStart, const double& RampEnd);

/**
 * Creates a falling ramp with value 1 at Start and 0 at End.
 */
Predicate RampDown(const double& Value, const double& RampStart, const double& RampEnd);

/**
 * Fuzzified AND. or Conjunction.
 */
Predicate AND (const Predicate& a, const Predicate& b);

/**
 * Fuzzified OR, or Disjunction.
 */
Predicate OR  (const Predicate& a, const Predicate& b);

/**
 * Fuzzified NOT, or Complement.
 */
Predicate NOT (const Predicate& a);

void IF(const Predicate& a);

double DeFuzzify(const Set& fset);

double DeFuzzify(const Set& fset, const double& begin);
	
}

#endif //#ifndef fuzzy_h_DEFINED

