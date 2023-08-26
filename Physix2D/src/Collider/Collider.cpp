#include "pch.h"
#include "Collider.h"
#include "Rigidbody/Rigidbody.h"

namespace Physix2D
{
	void BoxCollider::Update()
	{
		Vec2 position = Body->m_Position + Offset;
		Vec2 boxLeftTop = { position.x - Size.x,position.y + Size.y };
		Vec2 boxRightBottom = { position.x + Size.x,position.y - Size.y };

		Vertices[0] = { boxLeftTop.x, boxLeftTop.y };
		Vertices[1] = { boxRightBottom.x, boxLeftTop.y };
		Vertices[2] = { boxRightBottom.x, boxRightBottom.y };
		Vertices[3] = { boxLeftTop.x, boxRightBottom.y };

		for (size_t i = 0; i < 4; i++)
		{
			Vertices[i] = Vertices[i].Rotate(position, Body->m_Angle);

			if (Vertices[i].x < AABB.Min.x)
				AABB.Min.x = Vertices[i].x;
			else if (Vertices[i].x > AABB.Max.x)
				AABB.Max.x = Vertices[i].x;

			if (Vertices[i].y < AABB.Min.y)
				AABB.Min.y = Vertices[i].y;
			else if (Vertices[i].y > AABB.Max.y)
				AABB.Max.y = Vertices[i].y;
		}

		Body->m_UpdateRequired = false;
	}

	void CircleCollider::Update()
	{
		Vec2 position = Body->m_Position + Offset;
		AABB.Min.x = position.x - Radius;
		AABB.Min.y = position.y - Radius;
		AABB.Max.x = position.x + Radius;
		AABB.Max.y = position.y + Radius;

		Body->m_UpdateRequired = false;
	}
}