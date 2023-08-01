#include "Collisions.h"

namespace Physix2D
{
	bool Collisions::IntersectCircles(const CircleCollider& circleA, const CircleCollider& circleB, ContactParams* contactParams)
	{
		return false;
	}
	bool Collisions::IntersectPolygons(const BoxCollider& boxA, const BoxCollider& boxB, ContactParams* contactParams)
	{
		return false;
	}
	bool Collisions::IntersectCirclePolygon(const CircleCollider& circleA, const BoxCollider& boxB, ContactParams* contactParams)
	{
		return false;
	}
	bool Collisions::IntersectAABB(const AABB& aabbA, const AABB& aabbB)
	{
		if (aabbA.Max.x <= aabbB.Min.x || aabbB.Max.x <= aabbA.Min.x ||
			aabbA.Max.y <= aabbB.Min.y || aabbB.Max.y <= aabbA.Min.y)
		{
			return false;
		}

		return true;
	}
}