#pragma once

#include <Box2D/Box2D.h>

namespace RVO {
	class RVOLine {
	public:
		b2Vec2 point;
		b2Vec2 direction;
	};
}