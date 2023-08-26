#pragma once
#include "Math/Math.h"
#include "Collider/Collider.h"
#include "Utils/Ref.h"

namespace Physix2D
{
	enum class RigidbodyType
	{
		Static,
		Dynamic,
		Kinematic
	};

	struct Rigidbody2DSpecification
	{
		RigidbodyType Type;

		float Angle;
		Vec2 Position;
		bool FixedRotation;
		float GravityScale;
		bool Enabled;
		void* Entity;
	};

	struct ShapeSpecification
	{
		Collider* Collider = nullptr;

		float Density = 1.0f;
		float Friction = 0.5f;
		float Restitution = 0.0f;
		float RestitutionThreshold = 0.5f;
	};

	class Rigidbody2D
	{
	public:
		Rigidbody2D(const Rigidbody2DSpecification& spec);
		~Rigidbody2D() = default;

		void SetGravityScale(float scale) { m_GravityScale = scale; }
		void SetFixedRotation(bool fixed) { m_FixedRotation = fixed; }

		void Step(float timeStep);

		void Move(const Vec2& translation) { m_Position += translation; m_UpdateRequired = true; }
		void ApplyForce(const Vec2& force) { m_Force += force; m_UpdateRequired = true; }
		void ApplyTorque(float torque) { m_Torque += torque; m_UpdateRequired = true; }

		Vec2 GetPosition() const { return m_Position; }
		float GetAngle() const { return m_Angle; }

		bool IsFixedRotation() const { return m_FixedRotation; }
		bool NeedsUpdate() const { return m_UpdateRequired; }
		const Collider* CreateShape(const ShapeSpecification& spec);

		void SetEnabled(bool enabled) { m_Enabled = enabled; }
		bool IsEnabled() const { return m_Enabled; }

		RigidbodyType GetType() const { return m_Type; }
		bool IsStatic() const { return m_Type == RigidbodyType::Static; }
		bool IsDynamic() const { return m_Type == RigidbodyType::Dynamic; }
		bool IsKinematic() const { return m_Type == RigidbodyType::Kinematic; }

		template<typename T>
		Ref<T> GetShape() const { return std::static_pointer_cast<T>(m_Collider); }
	private:
		void* m_Entity = nullptr;

		bool m_Enabled = true;

		Vec2 m_Position = { 0,0 };
		float m_Angle = 0.0f;

		Vec2 m_LinearVelocity = { 0,0 };
		float m_AngularVelocity = 0.0f;

		Vec2 m_Force = { 0,0 };
		float m_Torque = 0.0f;

		bool m_FixedRotation = false;

		float m_GravityScale = 0.0f;
		float m_Mass = 0.0f;
		float m_InvMass = 0.0f;
		float m_Density = 1.0f;
		float m_Friction = 0.5f;
		float m_Restitution = 0.0f;
		float m_RestitutionThreshold = 0.5f;

		bool m_UpdateRequired = true;

		RigidbodyType m_Type = RigidbodyType::Static;
		Ref<Collider> m_Collider = nullptr;

		friend class PhysicsWorld2D;
		friend class BoxCollider;
		friend class CircleCollider;
		friend class Collisions;
	};
}