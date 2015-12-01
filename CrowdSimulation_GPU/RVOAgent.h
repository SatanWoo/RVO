#pragma once

#include <iostream>
#include <vector>
#include <Box2D/Box2D.h>

namespace RVO {
	class RVOSimualtor;
	class RVOLine;
	class RVOObstacles;

	class RVOAgent {
		typedef std::pair<float, RVOAgent *> AgentHash;
		typedef std::vector<AgentHash> AgentNeighbours;

		typedef std::pair<float, RVOObstacles *> ObstacleHash;
		typedef std::vector<ObstacleHash> ObstacleNeighbours;

		typedef std::vector<RVOLine> Line;

	public :
		RVOAgent(RVO::RVOSimualtor *simualtor);

		bool linearProgram1(const Line& lines, size_t
			 lineID, float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result);
		bool linearProgram2(const Line& lines, 
			float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result);
		bool linearProgram3();

		int maxNeighbours;
		float maxSpeed;
		float neightDist;
		b2Vec2 velocity;
		size_t ID;
		b2Vec2 position;
		b2Vec2 prefVelocity;
		float radius;
		float timeHorizon;
		float timeHorizonObst;
		b2Vec2 velocity;

	private:
		void computeNeighbours();
		void computeNewVelocity();
		void insertNewNeighbour(const RVOAgent* agent, float &rangeSq);
		void update();

	public:
		RVOSimualtor *simualtor;
		
	};

	
}
