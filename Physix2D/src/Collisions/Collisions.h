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
			Vec2 Normal = Vec2();
			float Depth = 0.0f;
			uint32_t ContactPointsCount = 0;
			std::vector<Vec2> ContactPoints;
		};
		static bool IntersectCircles(const CircleCollider& circleA, const CircleCollider& circleB, ContactParams& contactParams);
		static bool IntersectBoxs(const BoxCollider& boxA, const BoxCollider& boxB, ContactParams& contactParams);
		static bool IntersectCirclePolygon(const CircleCollider& circleA, const BoxCollider& boxB, ContactParams& contactParams);
		static bool IntersectAABB(const AABB& aabbA, const AABB& aabbB);

	private:
		static bool IntersectPolygons(const Vec2* vertices1, uint32_t count1, const Vec2& center1,
			const Vec2* vertices2, uint32_t count2, const Vec2& center2, ContactParams& contactParams);
		static bool IntersectCirclePolygon(const Vec2& circleCenter, float circleRadius,
			const Vec2& polygonCenter, const Vec2* vertices, uint32_t count, ContactParams& contactParams);

		static void FindPolygonsContactPoints(const Vec2* vertices1, uint32_t count1, const Vec2* vertices2, uint32_t count2, ContactParams& contactParams);
		static uint32_t FindClosestPointOnPolygon(const Vec2& circleCenter, const Vec2* vertices, uint32_t count);
	};
}