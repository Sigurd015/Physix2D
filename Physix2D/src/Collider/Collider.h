#pragma once
#include "Math/Math.h"

namespace Physix2D
{
	struct AABB
	{
		Vec2 Min = { 0.0f, 0.0f };
		Vec2 Max = { 0.0f, 0.0f };
	};

	struct Collider
	{
		Vec2 Offset = { 0.0f, 0.0f };
		Vec2 Center = { 0.0f, 0.0f };
		AABB AABB;
	};

	struct BoxCollider :public Collider
	{
		Vec2 Size = { 0.0f, 0.0f };
		Vec2 Vertices[4] = { { 0.0f, 0.0f }, { 0.0f, 0.0f },{ 0.0f, 0.0f }, { 0.0f, 0.0f } };
	};

	struct CircleCollider :public Collider
	{
		float Radius = 0.0f;
	};
}