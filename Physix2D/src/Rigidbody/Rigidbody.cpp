#include "Rigidbody.h"
#include "World/PhysicsWorld.h"

#include <iostream>

namespace Physix2D
{
	Rigidbody2D::Rigidbody2D(const RigidBody2DSpecification& spec) :m_Type(spec.Type), m_Position(spec.Position), m_Angle(spec.Angle)
	{}

	void Rigidbody2D::Step(float timeStep)
	{
		if (m_Type == RigidBodyType::Static)
			return;

		// Integrate velocity
		Vec2 acceleration = m_Force / m_Mass;
		m_LinearVelocity += acceleration * timeStep;
		m_AngularVelocity += m_Torque * timeStep;

		Vec2 gravity = PhysicsWorld2D::GetGravity();

		gravity *= m_GravityScale;

		m_LinearVelocity += gravity * timeStep;

		// Integrate position
		m_Position += m_LinearVelocity * timeStep;
		m_Angle += m_AngularVelocity * timeStep;

		// Clear forces
		m_Force = Vec2();
		m_Torque = 0.0f;
		m_UpdateRequired = true;
	}

	const Collider* Rigidbody2D::CreateShape(const ShapeSpecification& spec)
	{
		switch (spec.Collider->GetShapeType())
		{
		case ShapeType::Circle:
		{
			CircleCollider* srcCollider = static_cast<CircleCollider*>(spec.Collider);
			CircleCollider* circleCollider = new CircleCollider();
			circleCollider->Offset = srcCollider->Offset;
			circleCollider->IsTrigger = srcCollider->IsTrigger;
			circleCollider->Radius = srcCollider->Radius;
			m_Collider = circleCollider;
			break;
		}
		case ShapeType::Box:
		{
			BoxCollider* srcCollider = static_cast<BoxCollider*>(spec.Collider);
			BoxCollider* boxCollider = new BoxCollider();
			boxCollider->Offset = srcCollider->Offset;
			boxCollider->IsTrigger = srcCollider->IsTrigger;
			boxCollider->Size = srcCollider->Size;
			m_Collider = boxCollider;
			break;
		}
		}

		m_Density = spec.Density;
		m_Friction = spec.Friction;
		m_Restitution = spec.Restitution;
		m_RestitutionThreshold = spec.RestitutionThreshold;

		return m_Collider;
	}
}