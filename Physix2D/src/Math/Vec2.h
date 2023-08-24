#pragma once

#include <cstdlib>

namespace Physix2D
{
	class Vec2
	{
	public:
		float x, y;

		Vec2() :x(0.0f), y(0.0f) {}
		Vec2(float x, float y) :x(x), y(y) {}
		Vec2(const Vec2& other) :x(other.x), y(other.y) {}

		Vec2 operator+(const Vec2& other) const
		{
			return Vec2(x + other.x, y + other.y);
		}

		Vec2 operator-(const Vec2& other) const
		{
			return Vec2(x - other.x, y - other.y);
		}

		Vec2 operator*(float scalar) const
		{
			return Vec2(x * scalar, y * scalar);
		}

		Vec2 operator/(float scalar) const
		{
			return Vec2(x / scalar, y / scalar);
		}

		Vec2& operator+=(const Vec2& other)
		{
			x += other.x;
			y += other.y;
			return *this;
		}

		Vec2& operator-=(const Vec2& other)
		{
			x -= other.x;
			y -= other.y;
			return *this;
		}

		Vec2& operator*=(float scalar)
		{
			x *= scalar;
			y *= scalar;
			return *this;
		}

		Vec2& operator/=(float scalar)
		{
			x /= scalar;
			y /= scalar;
			return *this;
		}

		bool operator==(const Vec2& other) const
		{
			return x == other.x && y == other.y;
		}

		bool operator!=(const Vec2& other) const
		{
			return !(*this == other);
		}

		float Length() const
		{
			return sqrtf(x * x + y * y);
		}

		float LengthSquared() const
		{
			return x * x + y * y;
		}

		Vec2 Normalize() const
		{
			float length = Length();
			return Vec2(x / length, y / length);
		}

		float Dot(const Vec2& other) const
		{
			return x * other.x + y * other.y;
		}

		Vec2 Project(const Vec2& other) const
		{
			float scalar = Dot(other) / LengthSquared();
			return Vec2(x * scalar, y * scalar);
		}

		Vec2 Reflect(const Vec2& normal) const
		{
			return *this + normal * (2.0f * Dot(normal));
		}

		Vec2 Rotation(float angle) const
		{
			float cosAngle = cosf(angle);
			float sinAngle = sinf(angle);
			return Vec2(x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle);
		}

		static float Distance(const Vec2& a, const Vec2& b)
		{
			return (a - b).Length();
		}

		static Vec2 Normalize(const Vec2& vec)
		{
			return vec.Normalize();
		}

		static float Dot(const Vec2& a, const Vec2& b)
		{
			return a.Dot(b);
		}

		static float DistanceSquared(const Vec2& a, const Vec2& b)
		{
			return (a - b).LengthSquared();
		}

		static bool NearlyEqual(const Vec2& a, const Vec2& b, float smallAmount = 0.0005f)
		{
			return DistanceSquared(a, b) < smallAmount * smallAmount;
		}
	};
}