//
// B2Vec2DHelper.h
// Crowd Simulation
//
// Created by z on 15-5-3.
// Copyright (c) 2015Äê SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_B2Vec2DHelper_h
#define Crowd_Simulation_B2Vec2DHelper_h

#include "Box2D.h"
#include <math.h>

double _round(double x)
{
	return (double)((int)(x));
}

class B2Vec2DHelper
{
public:
	static b2Vec2 roundV(const b2Vec2& vec)
	{
		return b2Vec2(_round(vec.x), _round(vec.y));
	}

	static b2Vec2 floorV(const b2Vec2& vec)
	{
		return b2Vec2(floor(vec.x), floor(vec.y));
	}

	static float32 distanceTo(const b2Vec2& from, const b2Vec2& to)
	{
		return (from - to).Length();
	}
};

#endif