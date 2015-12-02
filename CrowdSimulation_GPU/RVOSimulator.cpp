#include "RVOSimulator.h"
#include "RVOAgent.h"
#include "RVOTree.h"
#include "RVOObstacles.h"
#include "RVOMath.h"
#include <omp.h>

namespace RVO {

	RVOSimualtor::RVOSimualtor():defaultAgent(NULL), globalTime(0), tree(NULL), timeStep(0.0)
	{
		tree = new RVOTree(this);
	}

	RVOSimualtor::RVOSimualtor(float timeStep, float neighborDist, size_t maxNeighbors,
		float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const b2Vec2 &velocity /* = b2Vec2_zero */)
	{
		tree = new RVOTree(this);
		defaultAgent = new RVOAgent(this);

		defaultAgent->maxNeighbours = maxNeighbors;
		defaultAgent->maxSpeed = maxSpeed;
		defaultAgent->radius = radius;
		defaultAgent->timeHorizon = timeHorizon;
		defaultAgent->timeHorizonObst = timeHorizonObst;
		defaultAgent->velocity = velocity;
		defaultAgent->neightDist = neighborDist;
	}

	RVOSimualtor::~RVOSimualtor()
	{
		if (defaultAgent != NULL) {
			delete defaultAgent;
			defaultAgent = NULL;
		}

		if (tree != NULL) {
			delete tree;
			tree = NULL;
		}
	}

	size_t RVOSimualtor::addAgent(const b2Vec2& position)
	{
		RVOAgent *agent = new RVOAgent(this);

		agent->position = position;
		agent->maxNeighbours = defaultAgent->maxNeighbours;
		agent->maxSpeed = defaultAgent->maxSpeed;
		agent->neightDist = defaultAgent->neightDist;
		agent->radius = defaultAgent->radius;
		agent->velocity = defaultAgent->velocity;
		agent->timeHorizon = defaultAgent->timeHorizon;
		agent->timeHorizonObst = agent->timeHorizonObst;
		agent->ID = agents.size();

		agents.push_back(agent);
		return agents.size() - 1;
	}

	size_t RVOSimualtor::addAgent(const b2Vec2& positon, float neighDist, size_t maxNeighbours, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const b2Vec2 &velocity /* = b2Vec2_zero */)
	{
		RVOAgent *agent = new RVOAgent(this);
		agent->position = positon;
		agent->maxNeighbours = maxNeighbours;
		agent->maxSpeed = maxSpeed;
		agent->neightDist = neighDist;
		agent->radius = radius;
		agent->timeHorizon = timeHorizon;
		agent->timeHorizonObst = timeHorizonObst;
		agent->velocity = velocity;

		agent->ID = agents.size();
		agents.push_back(agent);

		return agents.size() - 1;
	}

	size_t RVOSimualtor::addObstacles(const std::vector<b2Vec2>& vertices)
	{
		const size_t obCount = obstacles.size();

		for (size_t i = 0; i < vertices.size(); i++) 
		{
			RVOObstacles *obstacle = new RVOObstacles();
			obstacle->point = vertices[i];

			if (i != 0) 
			{
				obstacle->prev = obstacles.back();
				obstacle->prev->next = obstacle;
			}

			if (i == vertices.size() - 1) 
			{
				obstacle->next = obstacles[obCount];
				obstacle->next->prev = obstacle;
			}

			b2Vec2 dir = vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i];
			dir.Normalize();
			obstacle->unitDir = dir;
			
			if (vertices.size() <= 2) {
				obstacle->isConvex = true;
			} else {
				obstacle->isConvex = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]));
			}

			obstacle->ID = obstacles.size();
			obstacles.push_back(obstacle);
		}

		return obCount;
	}

	void RVOSimualtor::update()
	{
		int size = static_cast<int>(agents.size());

#pragma omp parallel for
		for (int i = 0; i < size; i++)
		{
			agents[i]->computeNeighbours();
			agents[i]->computeNewVelocity();
		}

#pragma omp parallel for 
		for (int i = 0; i < size; i++)
		{
			agents[i]->update();
		}

		globalTime += timeStep;
	}
}