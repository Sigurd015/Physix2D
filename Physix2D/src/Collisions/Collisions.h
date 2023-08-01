#pragma once
#include "Math/Math.h"
#include "Collider/Collider.h"

#include <cstdint>
#include <vector>

namespace Physix2D
{
	class Collisions
	{
	public:
		struct ContactParams
		{
			Vec2 Normal;
			float Depth;
			std::vector<Vec2> ContactPoints;
		};
		static bool IntersectCircles(const CircleCollider& circleA, const CircleCollider& circleB, ContactParams* contactParams);
		static bool IntersectPolygons(const BoxCollider& boxA, const BoxCollider& boxB, ContactParams* contactParams);
		static bool IntersectCirclePolygon(const CircleCollider& circleA, const BoxCollider& boxB, ContactParams* contactParams);
		static bool IntersectAABB(const AABB& aabbA, const AABB& aabbB);
	};
}