#pragma once
#include "Vec2.h"

namespace Physix2D
{
	static float FloatMin(float a, float b)
	{
		return a < b ? a : b;
	}

	static bool FloatNearlyEqual(float a, float b, float smallAmount = 0.0005f)
	{
		return fabs(a - b) < smallAmount;
	}
}