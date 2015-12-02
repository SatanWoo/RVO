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

		typedef std::vector<RVOLine> Lines;

		friend class RVOSimualtor;

	public :
		RVOAgent(RVO::RVOSimualtor *simualtor);
		RVOAgent(const RVOAgent *agent);
		RVOAgent(const RVOAgent& agent);

		bool linearProgram1(const Lines& lines, size_t
			 lineID, float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result);
		size_t linearProgram2(const Lines& lines, 
			float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result);
		void linearProgram3(const Lines &lines, size_t numObstLines, size_t beginLine, float radius, b2Vec2 &result);

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
		b2Vec2 newVelocity;

	private:
		void computeNeighbours();
		void computeNewVelocity();
		void insertNewNeighbour(const RVOAgent* agent, float &rangeSq);
		void insertObstacleNeighbour(const RVOObstacles *ob ,float rangeSq);
		void update();

		AgentNeighbours agentNeighbours;
		ObstacleNeighbours obstacleNeighbours;
		Lines lines;

	public:
		RVOSimualtor *simualtor;
		
	};

	
}
