#include "PhysicsWorld.h"

namespace Physix2D
{
	PhysicsWorld2D::PhysicsWorld2D(const PhysicsWorld2DSpecification& spec)
	{
		s_Gravity = spec.Gravity;
	}

	Rigidbody2D* PhysicsWorld2D::CreateBody(const RigidBody2DSpecification& spec)
	{
		Rigidbody2D body(spec);
		m_Bodies.push_back(body);
		return &m_Bodies.back();
	}

	void PhysicsWorld2D::Step(float timeStep, uint32_t iterations)
	{
		for (size_t currentIterations = 0; currentIterations < iterations; currentIterations++)
		{
			// Rigidbody2D Step
			{
				for (size_t i = 0; i < m_Bodies.size(); i++)
				{
					Rigidbody2D& rigidbody = m_Bodies[i];
					rigidbody.Step(timeStep / (float)iterations);
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
	{}

	void PhysicsWorld2D::NarrowPhase()
	{}

	void PhysicsWorld2D::ResolveCollision(Rigidbody2D& body1, Rigidbody2D& body2, Collisions::ContactParams& contactParams)
	{}
}