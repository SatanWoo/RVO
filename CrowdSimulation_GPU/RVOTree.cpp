#include "RVOTree.h"
#include "RVOSimulator.h"
#include "RVOAgent.h"
#include "RVOObstacles.h"

namespace RVO
{
	RVOTree::RVOTree(RVOSimualtor *simualtor):root(NULL), simulator(simualtor)
	{}

	RVOTree::~RVOTree()
	{
		deleteObstacleTree(root);
	}

	void RVOTree::buildAgentTree()
	{
		if (agents.size() < simulator->agents.size()) {
			for (size_t i = agents.size(); i < simulator->agents.size(); i++)
			{
				agents.push_back(simulator->agents[i]);
			}

			agents.resize(agents.size() * 2 - 1);
		}

		if (!agents.empty())
		{
			buildAgentTreeRecursively(0, agents.size(), 0);
		}
	}

	void RVOTree::buildAgentTreeRecursively(size_t begin, size_t end, size_t node)
	{
		agentTree[node],begin = begin;
		agentTree[node].end = end;
		agentTree[node].maxX = agentTree[node].minX = agents[begin]->position.x;
		agentTree[node].maxY = agentTree[node].minY = agents[begin]->position.y;

		for (size_t i = begin + 1; i < end; i++)
		{
			agentTree[node].maxX = std::max(agentTree[node].maxX, agents[i]->position.x);
			agentTree[node].maxY = std::max(agentTree[node].maxY, agents[i]->position.y);
			agentTree[node].minX = std::min(agentTree[node].minX, agents[i]->position.x);
			agentTree[node].minY = std::min(agentTree[node].minY, agents[i]->position.y);
		}

		if (end - begin > MAX_LEAF_SIZE)
		{
			const bool isVertical = (agentTree[node].maxX - agentTree[node].minX) > (agentTree[node].maxY - agentTree[node].minY);
			const float splitValue = isVertical ? 0.5 * (agentTree[node].maxX + agentTree[node].minX) : 0.5 * (agentTree[node].minY + agentTree[node].maxY);

			size_t left = begin, right = end;
			while (left < right) 
			{
				while (left < right && (isVertical ? agents[left]->position.x : agents[left]->position.y) < splitValue) 
				{
					++left;
				}

				while (right > left && (isVertical ? agents[right - 1]->position.x : agents[right - 1]->position.y) < splitValue)
				{
					--right;
				}

				if (left < right)
				{
					std::swap(agents[left], agents[right - 1]);
					++left;
					--right;
				}
			}

			if (left == begin) 
			{
				++left;
				++right;
			}

			agentTree[node].left = node + 1;
			agentTree[node].right = node + 2 * (left - begin);
			
			buildAgentTreeRecursively(begin, left, agentTree[node].left);
			buildAgentTreeRecursively(left, end, agentTree[node].right);
 		}
	}

	void RVOTree::buildObstacleTree()
	{
		deleteObstacleTree(root);

		std::vector<RVOObstacles *>obstacles(simulator->obstacles.begin(), simulator->obstacles.end());
		root = buildObstacleTreeRecursively(obstacles);
	}

	RVOTree::ObstacleNode* RVOTree::buildObstacleTreeRecursively(const std::vector<RVOObstacles *>& obstacles)
	{
		if (obstacles.empty()) return NULL;


	}
}