#pragma once
#include "Math/Math.h"
#include "Rigidbody/Rigidbody.h"
#include "Collisions/Collisions.h"

#include <vector>

namespace Physix2D
{
	struct PhysicsWorld2DSpecification
	{
		Vec2 Gravity = { 0.0f,-9.8f };
	};

	class PhysicsWorld2D
	{
	public:
		PhysicsWorld2D(const PhysicsWorld2DSpecification& spec);
		~PhysicsWorld2D() = default;

		void Step(float timeStep, uint32_t iterations);

		Rigidbody2D* CreateBody(const RigidBody2DSpecification& spec);
		static Vec2 GetGravity() { return s_Gravity; }
	private:
		struct ContactPair
		{
			uint32_t BodyIndex1;
			uint32_t BodyIndex2;
		};
		std::vector<ContactPair> m_ContactPairs;
		std::vector<Rigidbody2D> m_Bodies;

		void BroadPhase();
		void NarrowPhase();
		void ResolveCollision(Rigidbody2D& body1, Rigidbody2D& body2, Collisions::ContactParams& contactParams);

		static Vec2 s_Gravity;
	};
}