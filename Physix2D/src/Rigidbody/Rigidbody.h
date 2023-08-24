#pragma once
#include "Math/Math.h"
#include "Collider/Collider.h"

namespace Physix2D
{
	enum class RigidBodyType
	{
		Static,
		Dynamic,
		Kinematic
	};

	struct RigidBody2DSpecification
	{
		RigidBodyType Type;

		float Angle;
		Vec2 Position;
	};

	struct ShapeSpecification
	{
		Collider* Collider;

		float Density = 1.0f;
		float Friction = 0.5f;
		float Restitution = 0.0f;
		float RestitutionThreshold = 0.5f;
	};

	class Rigidbody2D
	{
	public:
		Rigidbody2D(const RigidBody2DSpecification& spec);
		~Rigidbody2D() = default;

		void SetGravityScale(float scale) { m_GravityScale = scale; }
		void SetFixedRotation(bool fixed) { m_FixedRotation = fixed; }

		void Step(float timeStep);

		void Move(const Vec2& translation) { m_Position += translation; m_UpdateRequired = true; }
		void ApplyForce(const Vec2& force) { m_Force += force; m_UpdateRequired = true; }

		bool IsFixedRotation() const { return m_FixedRotation; }
		bool NeedsUpdate() const { return m_UpdateRequired; }
		const Collider* CreateShape(const ShapeSpecification& spec);

		RigidBodyType GetType() const { return m_Type; }
	private:
		Vec2 m_Position;
		float m_Angle;

		Vec2 m_LinearVelocity;
		float m_AngularVelocity;

		Vec2 m_Force;
		float m_Torque;

		bool m_FixedRotation;

		float m_GravityScale;
		float m_Mass;
		float m_InvMass;
		float m_Density = 1.0f;
		float m_Friction = 0.5f;
		float m_Restitution = 0.0f;
		float m_RestitutionThreshold = 0.5f;

		bool m_UpdateRequired = true;

		RigidBodyType m_Type;
		ShapeType m_ShapeType;
		Collider* m_Collider;

		friend class PhysicsWorld2D;
		friend class BoxCollider;
		friend class CircleCollider;
		friend class Collisions;
	};
}