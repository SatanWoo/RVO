#pragma once
#include <Box2D/Box2D.h>

namespace RVO {
	class RVOObstacles {
	public:
		RVOObstacles(){isConvex = false; next = NULL; prev = NULL; ID = 0;}

		bool isConvex;
		RVOObstacles *prev;
		RVOObstacles *next;

		b2Vec2 point;
		b2Vec2 unitDir;

		size_t ID;
	};
}