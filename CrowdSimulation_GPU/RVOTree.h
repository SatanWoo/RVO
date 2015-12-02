#pragma once

#include <vector>
#include <Box2D/Box2D.h>

namespace RVO
{
	class RVOObstacles;
	class RVOAgent;
	class RVOSimualtor;
	class RVOTree
	{
		struct AgentNode {
			// All size_t represent ID
			size_t begin;
			size_t end;
			size_t left;
			size_t right;

			float maxX;
			float maxY;
			float minX;
			float minY;
		};

		struct ObstacleNode {
			ObstacleNode *left;
			ObstacleNode *right;

			const RVOObstacles *obstacle;
		};

	public:
		RVOTree(RVOSimualtor *simualtor);
		virtual ~RVOTree();

		void buildAgentTree();
		void buildObstacleTree();

		void computeAgentNeighbours(RVOAgent *agent, float &range)const;
		void computeObstacleNeighbours(RVOAgent *agent, float &range)const;
		void queryAgentTreeRecursive(RVOAgent *agent, float &range, size_t node)const;
		void queryObstacleTreeRecursive(RVOAgent *agent, float rang, const ObstacleNode *node)const;
		bool queryVisibility(const b2Vec2& v1, const b2Vec2& v2, float radius)const;

		static const size_t MAX_LEAF_SIZE = 10;
		
	private:
		void buildAgentTreeRecursively(size_t begin, size_t end, size_t node);
		ObstacleNode* buildObstacleTreeRecursively(const std::vector<RVOObstacles *>& obstacles);
		bool queryVisibilityRecursively(const b2Vec2& v1, const b2Vec2& v2, float radius, const ObstacleNode *node)const;
		void deleteObstacleTree(ObstacleNode *node);

		RVOSimualtor *simulator;
		ObstacleNode *root;
		std::vector<RVOAgent *> agents;
		std::vector<AgentNode> agentTree;
	};
}