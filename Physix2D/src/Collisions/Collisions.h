#pragma once
#include "Math/Math.h"
#include "Collider/Collider.h"

#include <cstdint>
#include <array>

#define MAX_CONTACT_POINTS 2

namespace Physix2D
{
	class Collisions
	{
	public:
		struct ContactParams
		{
			Vec2 Normal = Vec2();
			float Depth = FLT_MAX;
			uint32_t ContactPointsCount = 0;
			std::array<Vec2, MAX_CONTACT_POINTS> ContactPoints;
		};
		static bool IntersectCircles(const Vec2& circleCenter1, float circleRadius1,
			const Vec2& circleCenter2, float circleRadius2, ContactParams& contactParams);
		static bool IntersectPolygons(const Vec2& center1, const Vec2* vertices1, uint32_t count1,
			const Vec2& center2, const Vec2* vertices2, uint32_t count2, ContactParams& contactParams);
		static bool IntersectCirclePolygon(const Vec2& circleCenter, float circleRadius,
			const Vec2& polygonCenter, const Vec2* vertices, uint32_t count, ContactParams& contactParams);
		static bool IntersectAABB(const AABB& aabbA, const AABB& aabbB);

	private:
		static void FindPolygonsContactPoints(const Vec2* vertices1, uint32_t count1, const Vec2* vertices2, uint32_t count2, ContactParams& contactParams);
		static uint32_t FindClosestPointOnPolygon(const Vec2& circleCenter, const Vec2* vertices, uint32_t count);
	};
}