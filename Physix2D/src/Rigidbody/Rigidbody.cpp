#include "pch.h"
#include "Rigidbody.h"
#include "World/PhysicsWorld.h"

#include <iostream>

namespace Physix2D
{
	Rigidbody2D::Rigidbody2D(const Rigidbody2DSpecification& spec) :m_Type(spec.Type), m_Position(spec.Position), m_Angle(spec.Angle),
		m_FixedRotation(spec.FixedRotation), m_Entity(spec.Entity), m_GravityScale(spec.GravityScale), m_Enabled(spec.Enabled), m_UpdateRequired(true)
	{}

	void Rigidbody2D::Step(float timeStep)
	{
		if (IsStatic())
			return;

		// Integrate velocity
		if (IsDynamic())
		{
			Vec2 gravity = PhysicsWorld2D::GetGravity();
			gravity *= m_GravityScale;
			m_LinearVelocity += gravity * timeStep;
		}

		Vec2 acceleration = m_Force / m_Mass;
		m_LinearVelocity += acceleration * timeStep;
		m_AngularVelocity += m_Torque * timeStep;

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
		if (spec.Collider == nullptr)
			return nullptr;

		m_Density = spec.Density;
		m_Friction = spec.Friction;
		m_Restitution = spec.Restitution;
		m_RestitutionThreshold = spec.RestitutionThreshold;

		switch (spec.Collider->GetShapeType())
		{
		case ShapeType::Circle:
		{
			CircleCollider* srcCollider = static_cast<CircleCollider*>(spec.Collider);
			Ref<CircleCollider> circleCollider = CreateRef<CircleCollider>();
			circleCollider->Offset = srcCollider->Offset;
			circleCollider->IsTrigger = srcCollider->IsTrigger;
			circleCollider->Radius = srcCollider->Radius;
			m_Collider = circleCollider;

			m_Mass = (srcCollider->Radius * srcCollider->Radius) * 3.14f * m_Density;
			break;
		}
		case ShapeType::Box:
		{
			BoxCollider* srcCollider = static_cast<BoxCollider*>(spec.Collider);
			Ref<BoxCollider> boxCollider = CreateRef<BoxCollider>();
			boxCollider->Offset = srcCollider->Offset;
			boxCollider->IsTrigger = srcCollider->IsTrigger;
			boxCollider->Size = srcCollider->Size;
			m_Collider = boxCollider;

			m_Mass = (srcCollider->Size.x * 2) * (srcCollider->Size.y * 2) * m_Density;
			break;
		}
		}

		m_Collider->Body = this;

		m_UpdateRequired = true;

		if (IsStatic())
			m_InvMass = 0.0f;
		else
			m_InvMass = 1.0f / m_Mass;

		return m_Collider.get();
	}
}