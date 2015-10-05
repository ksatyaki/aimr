/**
 * fuzzy.cpp
 */
#include "aimr/fuzzy.h"

namespace fuzzy
{

Predicate ante = 0;

Predicate RampUp(const double& Value, const double& RampStart, const double& RampEnd)
{
  if (Value > RampEnd) return(1.0);
  if (Value < RampStart) return(0.0);
  if (RampStart == RampEnd) return(0.0);
  return((Value - RampStart) / (RampEnd - RampStart));
}

Predicate RampDown(const double& Value, const double& RampStart, const double& RampEnd)
{
  if (Value < RampStart) return(1.0);
  if (Value > RampEnd) return(0.0);
  if (RampStart == RampEnd) return(0.0);
  return((RampEnd - Value) / (RampEnd - RampStart));
}


Predicate AND (const Predicate& a, const Predicate& b)
{
	return MIN(a, b);
}

Predicate OR  (const Predicate& a, const Predicate& b)
{
	return MAX(a, b);
}

Predicate NOT (const Predicate& a)
{
	return (1.0 - a);
}

void IF (const Predicate& a)
{
	ante = a;
}

double DeFuzzify(const Set& fset)
{
  //Defuzzify using Center of Gravity (CoG)
  double sum;
  double wsum;
  int i;

  sum = 0.0;
  wsum = 0.0;
  for (i = 0; i < fset.size(); i++)
  {
    sum += fset[i];
    wsum += fset[i] * (double)i;
  }

  double result;
  //Prevent division by zero
  if (sum > 0.0001)
  {
  	 result = wsum / sum;
  }
  else
  {
    printf("fuzzy.cpp: DeFuzzify: warning: empty fuzzy set, returning 0\n");
    result = 0.0;
  }

  return result;
}

}
