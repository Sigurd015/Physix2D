#pragma once
#include "Math/Math.h"

namespace Physix2D
{
	struct AABB
	{
		Vec2 Min = { 0.0f, 0.0f };
		Vec2 Max = { 0.0f, 0.0f };
	};

	enum class ShapeType
	{
		Box, Circle
	};

	// Forward declaration
	class Rigidbody2D;

	class Collider
	{
	public:
		Vec2 Offset = { 0.0f, 0.0f };
		AABB AABB;

		bool IsTrigger = false;

		Rigidbody2D* Body = nullptr;

		virtual ShapeType GetShapeType() = 0;
		virtual void Update() = 0;
	};

	class BoxCollider :public Collider
	{
	public:
		Vec2 Size = { 0.0f, 0.0f };
		Vec2 Vertices[4] = { { 0.0f, 0.0f }, { 0.0f, 0.0f },{ 0.0f, 0.0f }, { 0.0f, 0.0f } };

		void Update() override;

		ShapeType GetShapeType() override { return ShapeType::Box; }
	};

	class CircleCollider :public Collider
	{
	public:
		float Radius = 0.0f;

		void Update() override;

		ShapeType GetShapeType() override { return ShapeType::Circle; }
	};
}