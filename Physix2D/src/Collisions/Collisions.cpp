#include "pch.h"
#include "Collisions.h"
#include "Rigidbody/Rigidbody.h"

namespace Physix2D
{
	namespace Utils
	{
		void ProjectVertices(const Vec2* vertices, uint32_t count, Vec2& axis, float& min, float& max)
		{
			min = FLT_MAX;
			max = FLT_MIN;

			for (size_t i = 0; i < count; i++)
			{
				Vec2 v = vertices[i];
				float proj = Vec2::Dot(v, axis);

				if (proj < min)
					min = proj;

				if (proj > max)
					max = proj;
			}
		}

		void ProjectCircle(const Vec2& center, float radius, const  Vec2& axis, float& min, float& max)
		{
			Vec2 direction = Vec2::Normalize(axis);
			Vec2 directionAndRadius = direction * radius;

			Vec2 p1 = center + directionAndRadius;
			Vec2 p2 = center - directionAndRadius;

			min = Vec2::Dot(p1, axis);
			max = Vec2::Dot(p2, axis);

			if (min > max)
			{
				// swap the min and max values.
				float t = min;
				min = max;
				max = t;
			}
		}

		void PointSegmentDistance(const Vec2& p, const Vec2& a, const Vec2& b, float& distanceSquared, Vec2& contactPoint)
		{
			Vec2 ab = b - a;
			Vec2 ap = p - a;

			float proj = Vec2::Dot(ap, ab);
			float abLenSq = ab.LengthSquared();
			float d = proj / abLenSq;

			if (d <= 0.0f)
			{
				contactPoint = a;
			}
			else if (d >= 1.0f)
			{
				contactPoint = b;
			}
			else
			{
				contactPoint = a + (ab * d);
			}

			distanceSquared = Vec2::DistanceSquared(p, contactPoint);
		}
	}

	bool Collisions::IntersectCircles(const Vec2& circleCenter1, float circleRadius1,
		const Vec2& circleCenter2, float circleRadius2, ContactParams& contactParams)
	{
		float distence = Vec2::Distance(circleCenter1, circleCenter2);
		float sumRadius = circleRadius1 + circleRadius2;

		if (distence > sumRadius)
			return false;

		Vec2 dir = Vec2::Normalize(circleCenter2 - circleCenter1);
		contactParams.Normal = dir;
		contactParams.Depth = sumRadius - distence;
		contactParams.ContactPoints[0] = circleCenter1 + (dir * circleRadius1);
		contactParams.ContactPointsCount = 1;

		return true;
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

	bool Collisions::IntersectPolygons(const Vec2& center1, const Vec2* vertices1, uint32_t count1,
		const Vec2& center2, const Vec2* vertices2, uint32_t count2, ContactParams& contactParams)
	{
		{
			for (size_t i = 0; i < count1; i++)
			{
				Vec2 va = vertices1[i];
				Vec2 vb = vertices1[(i + 1) % 4];

				Vec2 edge = vb - va;
				Vec2 axis = Vec2::Normalize({ -edge.y, edge.x });

				float minA, maxA, minB, maxB;
				Utils::ProjectVertices(vertices1, 4, axis, minA, maxA);
				Utils::ProjectVertices(vertices2, 4, axis, minB, maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				float axisDepth = FloatMin(maxB - minA, maxA - minB);

				if (axisDepth < contactParams.Depth)
				{
					contactParams.Depth = axisDepth;
					contactParams.Normal = axis;
				}
			}
		}

		{
			for (size_t i = 0; i < count2; i++)
			{
				Vec2 va = vertices2[i];
				Vec2 vb = vertices2[(i + 1) % count2];

				Vec2 edge = vb - va;
				Vec2 axis = Vec2::Normalize({ -edge.y, edge.x });

				float minA, maxA, minB, maxB;
				Utils::ProjectVertices(vertices1, count1, axis, minA, maxA);
				Utils::ProjectVertices(vertices2, count2, axis, minB, maxB);

				if (minA >= maxB || minB >= maxA)
				{
					return false;
				}

				float axisDepth = FloatMin(maxB - minA, maxA - minB);

				if (axisDepth < contactParams.Depth)
				{
					contactParams.Depth = axisDepth;
					contactParams.Normal = axis;
				}
			}
		}

		Vec2 direction = center2 - center1;

		if (Vec2::Dot(direction, contactParams.Normal) < 0.0f)
		{
			contactParams.Normal = contactParams.Normal * -1;
		}

		FindPolygonsContactPoints(vertices1, count1, vertices2, count2, contactParams);

		return true;
	}

	bool Collisions::IntersectCirclePolygon(const Vec2& circleCenter, float circleRadius,
		const Vec2& polygonCenter, const Vec2* vertices, uint32_t count, ContactParams& contactParams)
	{
		contactParams.ContactPointsCount = 1;
		float minDistSq = FLT_MAX;

		Vec2 axis = Vec2();
		float axisDepth = 0;
		float minA, maxA, minB, maxB;

		for (size_t i = 0; i < count; i++)
		{
			Vec2 va = vertices[i];
			Vec2 vb = vertices[(i + 1) % count];

			Vec2 edge = vb - va;
			axis = Vec2::Normalize({ -edge.y, edge.x });

			Utils::ProjectVertices(vertices, count, axis, minA, maxA);
			Utils::ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

			if (minA >= maxB || minB >= maxA)
			{
				return false;
			}

			axisDepth = FloatMin(maxB - minA, maxA - minB);

			if (axisDepth < contactParams.Depth)
			{
				contactParams.Depth = axisDepth;
				contactParams.Normal = axis;
			}

			Vec2 contact = Vec2();
			float distSq = 0;
			Utils::PointSegmentDistance(circleCenter, va, vb, distSq, contact);

			if (distSq < minDistSq)
			{
				minDistSq = distSq;
				contactParams.ContactPoints[0] = contact;
			}
		}

		uint32_t cpIndex = FindClosestPointOnPolygon(circleCenter, vertices, count);
		Vec2 cp = vertices[cpIndex];

		axis = cp - circleCenter;
		axis = axis.Normalize();

		Utils::ProjectVertices(vertices, count, axis, minA, maxA);
		Utils::ProjectCircle(circleCenter, circleRadius, axis, minB, maxB);

		if (minA >= maxB || minB >= maxA)
		{
			return false;
		}

		axisDepth = FloatMin(maxB - minA, maxA - minB);

		if (axisDepth < contactParams.Depth)
		{
			contactParams.Depth = axisDepth;
			contactParams.Normal = axis;
		}

		Vec2 direction = polygonCenter - circleCenter;

		if (Vec2::Dot(direction, contactParams.Normal) < 0)
		{
			contactParams.Normal = contactParams.Normal * -1;
		}

		return true;
	}

	void Collisions::FindPolygonsContactPoints(const Vec2* vertices1, uint32_t count1,
		const Vec2* vertices2, uint32_t count2, ContactParams& contactParams)
	{
		float minDistSq = FLT_MAX;

		for (size_t i = 0; i < count1; i++)
		{
			Vec2 p = vertices1[i];

			for (size_t j = 0; j < count2; j++)
			{
				Vec2 va = vertices2[j];
				Vec2 vb = vertices2[(j + 1) % count2];

				Vec2 contact = Vec2();
				float distSq = 0;
				Utils::PointSegmentDistance(p, va, vb, distSq, contact);

				if (FloatNearlyEqual(distSq, minDistSq))
				{
					if (!Vec2::NearlyEqual(contact, contactParams.ContactPoints[0]))
					{
						contactParams.ContactPoints[1] = contact;
						contactParams.ContactPointsCount = 2;
					}
				}
				else if (distSq < minDistSq)
				{
					minDistSq = distSq;
					contactParams.ContactPointsCount = 1;
					contactParams.ContactPoints[0] = contact;
				}
			}
		}

		for (size_t i = 0; i < count2; i++)
		{
			Vec2 p = vertices2[i];

			for (size_t j = 0; j < count1; j++)
			{
				Vec2 va = vertices1[j];
				Vec2 vb = vertices1[(j + 1) % count1];

				Vec2 contact = Vec2();
				float distSq = 0;
				Utils::PointSegmentDistance(p, va, vb, distSq, contact);

				if (FloatNearlyEqual(distSq, minDistSq))
				{
					if (!Vec2::NearlyEqual(contact, contactParams.ContactPoints[0]))
					{
						contactParams.ContactPoints[1] = contact;
						contactParams.ContactPointsCount = 2;
					}
				}
				else if (distSq < minDistSq)
				{
					minDistSq = distSq;
					contactParams.ContactPointsCount = 1;
					contactParams.ContactPoints[0] = contact;
				}
			}
		}
	}

	uint32_t Collisions::FindClosestPointOnPolygon(const Vec2& circleCenter, const Vec2* vertices, uint32_t count)
	{
		uint32_t result = 0;
		float minDistance = FLT_MAX;

		for (size_t i = 0; i < count; i++)
		{
			Vec2 v = vertices[i];
			float distance = Vec2::Distance(v, circleCenter);

			if (distance < minDistance)
			{
				minDistance = distance;
				result = i;
			}
		}

		return result;
	}
}