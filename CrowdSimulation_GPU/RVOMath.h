#pragma once
#include <Box2D.h>

namespace RVO
{
	const float RVO_EPSILON = 0.00001f;

	inline float det(const b2Vec2& row1, const b2Vec2& row2)
	{
		return row1.x * row2.y - row1.y * row1.x;
	}

	inline float leftOf(const b2Vec2& start, const b2Vec2& end, const b2Vec2& p)
	{
		return det(start - p, end - start);
	}

	inline float distSqPointLineSegment(const b2Vec2 &start, const b2Vec2 &end,
		const b2Vec2 &p)
	{
		const float r = ((p - start) * (end - p)) / (end - start).LengthSquared();

		if (r < 0.0f) {
			return (p - start).LengthSquared();
		}
		else if (r > 1.0f) {
			return (p - end).LengthSquared;
		}
		else {
			b2Vec2 vec = p - (start + r * (end - start));
			return vec.LengthSquared();
		}
	}
}