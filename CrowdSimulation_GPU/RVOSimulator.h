#pragma once

#include <Box2D/Box2D.h>
#include <vector>

namespace RVO
{
	class RVOLine;
	class RVOAgent;
	class RVOObstacles;
	class RVOTree;

	class RVOSimualtor {
		friend class RVOTree;
	public:

		RVOSimualtor();
		RVOSimualtor(float timeStep, float neighborDist, size_t maxNeighbors,
			float timeHorizon, float timeHorizonObst, float radius,
			float maxSpeed, const b2Vec2 &velocity = b2Vec2_zero);

		virtual ~RVOSimualtor();

		// ·µ»ØID
		size_t addAgent(const b2Vec2& position);
		size_t addAgent(const b2Vec2& positon, float neighDist, size_t maxNeighbours, float timeHorizon,
			float timeHorizonObst, float radius, float maxSpeed, const b2Vec2 &velocity = b2Vec2_zero);

		size_t addObstacles(const std::vector<b2Vec2>& vertices);

		void update();

		RVOTree *tree;
		float timeStep;

		/*size_t getAgentNeighbours(size_t agentID, size_t neighbourID)const;
		size_t getAgentMaxNeighbour(size_t agentID)const;
		size_t getAgentMaxSpeed(size_t agentID)const;
		float  getAgentNeighborDist(size_t agentID)const;
		size_t getAgentInfluencedNeighbors(size_t agentID)const;
		size_t getAgentInfluenceObstacles(size_t agentID)const;

		const RVOLine& getAgentORCALine(size_t agentID, size_t lineID)const;
		const b2Vec2& getAgentPosition(size_t agentID)const;
		const b2Vec2& getAgentPrefVelocity(size_t agentID)const;

		float getAgentRadius(size_t agentID)const;
		float getAgentTimeHorizon(size_t agentID)const;
		float getAgentTimeHorizonObst(size_t agentID)const;
		const b2Vec2& getAgentVelocity(size_t agentID)const;

		float getGlobalTime()const;*/
	private:
		std::vector<RVOAgent *> agents;
		std::vector<RVOObstacles *> obstacles;

		RVOAgent *defaultAgent;

		float globalTime;
	};
}
