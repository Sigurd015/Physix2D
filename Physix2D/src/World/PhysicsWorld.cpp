#include "PhysicsWorld.h"

#define MAX_CONTACT_POINTS 2

namespace Physix2D
{
	static Vec2 s_Gravity = { 0.0f, -9.8f };

	PhysicsWorld2D::PhysicsWorld2D(const PhysicsWorld2DSpecification& spec)
	{
		s_Gravity = spec.Gravity;
		m_Bodies.clear();

		m_ContactParams.ContactPoints.clear();
		for (size_t i = 0; i < MAX_CONTACT_POINTS; i++)
		{
			m_ContactParams.ContactPoints.push_back(Vec2());
		}
	}

	Rigidbody2D* PhysicsWorld2D::CreateBody(const Rigidbody2DSpecification& spec)
	{
		Rigidbody2D body(spec);
		m_Bodies.push_back(body);
		return &m_Bodies.back();
	}

	Vec2 PhysicsWorld2D::GetGravity()
	{
		return s_Gravity;
	}

	void PhysicsWorld2D::Step(float timeStep, uint32_t iterations)
	{
		for (size_t currentIterations = 0; currentIterations < iterations; currentIterations++)
		{
			// Rigidbody2D Step
			{
				for (auto& body : m_Bodies)
				{
					body.Step(timeStep / (float)iterations);
				}
			}

			// Collide
			{
				m_ContactPairs.clear();
				BroadPhase();
				NarrowPhase();
			}
		}
	}

	void PhysicsWorld2D::BroadPhase()
	{
		for (size_t i = 0; i < m_Bodies.size(); i++)
		{
			Rigidbody2D& body1 = m_Bodies[i];

			if (!body1.m_Enabled)
			{
				continue;
			}

			for (size_t j = i + 1; j < m_Bodies.size(); j++)
			{
				Rigidbody2D& body2 = m_Bodies[j];

				if (!body2.m_Enabled)
				{
					continue;
				}

				if (body1.IsStatic() && body2.IsStatic())
				{
					continue;
				}

				if (body1.m_UpdateRequired)
				{
					body1.m_Collider->Update();
				}

				if (body2.m_UpdateRequired)
				{
					body2.m_Collider->Update();
				}

				if (!Collisions::IntersectAABB(body1.m_Collider->AABB, body2.m_Collider->AABB))
				{
					continue;
				}

				ContactPair contactPair = { i, j };
				m_ContactPairs.push_back(contactPair);
			}
		}
	}

	void PhysicsWorld2D::NarrowPhase()
	{
		for (auto& pair : m_ContactPairs)
		{
			Rigidbody2D& body1 = m_Bodies[pair.BodyIndex1];
			Rigidbody2D& body2 = m_Bodies[pair.BodyIndex2];

			if (Collide(body1, body2, m_ContactParams))
			{
				if (body1.m_Collider->IsTrigger || body2.m_Collider->IsTrigger)
				{
					// TODO:: Trigger Callback

					continue;
				}

				if (body1.IsStatic())
				{
					body2.Move(m_ContactParams.Normal * m_ContactParams.Depth);
				}
				else if (body2.IsStatic())
				{
					body1.Move(m_ContactParams.Normal * -m_ContactParams.Depth);
				}
				else
				{
					body1.Move(m_ContactParams.Normal * -(m_ContactParams.Depth / 2.0f));
					body2.Move(m_ContactParams.Normal * (m_ContactParams.Depth / 2.0f));
				}

				ResolveCollision(body1, body2, m_ContactParams);
			}
		}
	}

	bool PhysicsWorld2D::Collide(Rigidbody2D& body1, Rigidbody2D& body2, Collisions::ContactParams& contactParams)
	{
		ShapeType shape1 = body1.m_Collider->GetShapeType();
		ShapeType shape2 = body2.m_Collider->GetShapeType();

		if (shape1 == ShapeType::Box)
		{
			BoxCollider* box1Collider = static_cast<BoxCollider*>(body1.m_Collider);
			Vec2 box1Center = body1.m_Position + body1.m_Collider->Offset;

			if (shape2 == ShapeType::Circle)
			{
				CircleCollider* circleCollider = static_cast<CircleCollider*>(body2.m_Collider);
				Vec2 circleCenter = body2.m_Position + body2.m_Collider->Offset;

				bool result = Collisions::IntersectCirclePolygon(circleCenter, circleCollider->Radius,
					box1Center, box1Collider->Vertices, 4, contactParams);
				contactParams.Normal *= -1;
				return result;
			}
			else if (shape2 == ShapeType::Box)
			{
				BoxCollider* box2Collider = static_cast<BoxCollider*>(body2.m_Collider);
				Vec2 box2Center = body2.m_Position + body2.m_Collider->Offset;

				return Collisions::IntersectPolygons(box1Center, box1Collider->Vertices, 4,
					box2Center, box2Collider->Vertices, 4, contactParams);
			}
		}
		else if (shape1 == ShapeType::Circle)
		{
			CircleCollider* circle1Collider = static_cast<CircleCollider*>(body1.m_Collider);
			Vec2 circle1Center = body1.m_Position + body1.m_Collider->Offset;

			if (shape2 == ShapeType::Circle)
			{
				CircleCollider* circle2Collider = static_cast<CircleCollider*>(body2.m_Collider);
				Vec2 circle2Center = body2.m_Position + body2.m_Collider->Offset;

				return Collisions::IntersectCircles(circle1Center, circle1Collider->Radius,
					circle2Center, circle2Collider->Radius, contactParams);
			}
			else if (shape2 == ShapeType::Box)
			{
				BoxCollider* box2Collider = static_cast<BoxCollider*>(body2.m_Collider);
				Vec2 box2Center = body2.m_Position + body2.m_Collider->Offset;

				return Collisions::IntersectCirclePolygon(circle1Center, circle1Collider->Radius,
					box2Center, box2Collider->Vertices, 4, contactParams);
			}
		}

		return false;
	}

	void PhysicsWorld2D::ResolveCollision(Rigidbody2D& body1, Rigidbody2D& body2, Collisions::ContactParams& contactParams)
	{
		//// With rotation and friction version, but not working correctly
		//float 	InvInertia = 0.1f;

		//float e = FloatMin(body1.m_Restitution, body2.m_Restitution);

		//float StaticFriction = 0.4f;
		//float DynamicFriction = 0.6f;

		////float sf = (body1.m_StaticFriction + body2.m_StaticFriction) * 0.5f;
		////float df = (body1.m_DynamicFriction + body2.m_DynamicFriction) * 0.5f;
		//float sf = (StaticFriction + StaticFriction) * 0.5f;
		//float df = (DynamicFriction + DynamicFriction) * 0.5f;

		//Vec2 impulseList[2] = { };
		//Vec2 raList[2] = { };
		//Vec2 rbList[2] = { };
		//Vec2 raPerpList[2] = { };
		//Vec2 rbPerpList[2] = { };
		//Vec2 frictionImpulseList[2] = { };
		//float jList[2] = { 0.0f, 0.0f };

		//for (size_t i = 0; i < contactParams.ContactPointsCount; i++)
		//{
		//	Vec2 ra = contactParams.ContactPoints[i] - body1.m_Position;
		//	Vec2 rb = contactParams.ContactPoints[i] - body2.m_Position;

		//	raList[i] = ra;
		//	rbList[i] = rb;

		//	Vec2 raPerp = { -ra.y, ra.x };
		//	Vec2 rbPerp = { -rb.y, rb.x };

		//	raPerpList[i] = raPerp;
		//	rbPerpList[i] = rbPerp;

		//	Vec2 angularLinearVelocityA = raPerp * body1.m_AngularVelocity;
		//	Vec2 angularLinearVelocityB = rbPerp * body2.m_AngularVelocity;

		//	Vec2 relativeVelocity =
		//		(body2.m_LinearVelocity + angularLinearVelocityB) -
		//		(body1.m_LinearVelocity + angularLinearVelocityA);

		//	float contactVelocityMag = Vec2::Dot(relativeVelocity, contactParams.Normal);

		//	if (contactVelocityMag > 0.0f)
		//	{
		//		continue;
		//	}

		//	float raPerpDotN = Vec2::Dot(raPerp, contactParams.Normal);
		//	float rbPerpDotN = Vec2::Dot(rbPerp, contactParams.Normal);

		//	//float denom = body1.m_InvMass + body2.m_InvMass +
		//	//	(raPerpDotN * raPerpDotN) * body1.m_InvInertia +
		//	//	(rbPerpDotN * rbPerpDotN) * body2.m_InvInertia;

		//	float denom = body1.m_InvMass + body2.m_InvMass +
		//		(raPerpDotN * raPerpDotN) * InvInertia +
		//		(rbPerpDotN * rbPerpDotN) * InvInertia;

		//	float j = -(1.0f + e) * contactVelocityMag;
		//	j /= denom;
		//	j /= (float)contactParams.ContactPointsCount;

		//	jList[i] = j;

		//	Vec2 impulse = contactParams.Normal * j;
		//	impulseList[i] = impulse;
		//}

		//for (size_t i = 0; i < contactParams.ContactPointsCount; i++)
		//{
		//	Vec2 impulse = impulseList[i];
		//	Vec2 ra = raList[i];
		//	Vec2 rb = rbList[i];

		//	body1.m_LinearVelocity += impulse * -body1.m_InvMass;
		//	//body1.m_AngularVelocity += Vec2::Cross(ra, impulse) * -body1.m_InvInertia;
		//	body1.m_AngularVelocity += Vec2::Cross(ra, impulse) * -InvInertia;

		//	body2.m_LinearVelocity += impulse * body2.m_InvMass;
		//	//body2.m_AngularVelocity += Vec2::Cross(rb, impulse) * body2.m_InvInertia;
		//	body2.m_AngularVelocity += Vec2::Cross(rb, impulse) * InvInertia;
		//}

		//for (size_t i = 0; i < contactParams.ContactPointsCount; i++)
		//{
		//	Vec2 ra = contactParams.ContactPoints[i] - body1.m_Position;
		//	Vec2 rb = contactParams.ContactPoints[i] - body2.m_Position;

		//	raList[i] = ra;
		//	rbList[i] = rb;

		//	Vec2 raPerp = { -ra.y, ra.x };
		//	Vec2 rbPerp = { -rb.y, rb.x };

		//	Vec2 angularLinearVelocityA = raPerp * body1.m_AngularVelocity;
		//	Vec2 angularLinearVelocityB = rbPerp * body2.m_AngularVelocity;

		//	Vec2 relativeVelocity = (body2.m_LinearVelocity + angularLinearVelocityB) -
		//		(body1.m_LinearVelocity + angularLinearVelocityA);

		//	Vec2 tangent = relativeVelocity - (contactParams.Normal * Vec2::Dot(relativeVelocity, contactParams.Normal));

		//	if (Vec2::NearlyEqual(tangent, Vec2()))
		//		continue;
		//	else
		//	{
		//		tangent = Vec2::Normalize(tangent);
		//	}

		//	float raPerpDotT = Vec2::Dot(raPerp, tangent);
		//	float rbPerpDotT = Vec2::Dot(rbPerp, tangent);

		//	//float denom = body1.m_InvMass + body2.m_InvMass +
		//	//	(raPerpDotT * raPerpDotT) * body1.m_InvInertia +
		//	//	(rbPerpDotT * rbPerpDotT) * body2.m_InvInertia;

		//	float denom = body1.m_InvMass + body2.m_InvMass +
		//		(raPerpDotT * raPerpDotT) * InvInertia +
		//		(rbPerpDotT * rbPerpDotT) * InvInertia;

		//	float jt = -Vec2::Dot(relativeVelocity, tangent);
		//	jt /= denom;
		//	jt /= (float)contactParams.ContactPointsCount;

		//	Vec2 frictionImpulse;
		//	float j = jList[i];

		//	if (fabs(jt) <= j * sf)
		//	{
		//		frictionImpulse = tangent * jt;
		//	}
		//	else
		//	{
		//		frictionImpulse = tangent * -j * df;
		//	}

		//	frictionImpulseList[i] = frictionImpulse;
		//}

		//for (size_t i = 0; i < contactParams.ContactPointsCount; i++)
		//{
		//	Vec2 frictionImpulse = frictionImpulseList[i];
		//	Vec2 ra = raList[i];
		//	Vec2 rb = rbList[i];

		//	body1.m_LinearVelocity += frictionImpulse * -body1.m_InvMass;
		//	//body1.m_AngularVelocity += Vec2::Cross(ra, frictionImpulse) * -body1.m_InvInertia;
		//	body1.m_AngularVelocity += Vec2::Cross(ra, frictionImpulse) * -InvInertia;
		//	
		//	body2.m_LinearVelocity += frictionImpulse * body2.m_InvMass;
		//	//body2.m_AngularVelocity += Vec2::Cross(rb, frictionImpulse) * body2.m_InvInertia;
		//	body2.m_AngularVelocity += Vec2::Cross(rb, frictionImpulse) * InvInertia;
		//}

		// No rotation and friction version
		Vec2 relativeVelocity = body2.m_LinearVelocity + body1.m_LinearVelocity;

		float contactVelocityMag = Vec2::Dot(relativeVelocity, contactParams.Normal);
		if (contactVelocityMag > 0)
		{
			return;
		}

		float e = FloatMin(body1.m_Restitution, body2.m_Restitution);

		float j = -(1.0f + e) * contactVelocityMag;
		j /= body1.m_InvMass + body2.m_InvMass;

		Vec2 impulse = contactParams.Normal * j;

		body1.m_LinearVelocity -= impulse * body1.m_InvMass;
		body2.m_LinearVelocity += impulse * body2.m_InvMass;
	}
}