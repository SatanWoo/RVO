#include "RVOAgent.h"
#include "RVOSimulator.h"
#include "RVOObstacles.h"
#include "RVOTree.h"
#include "RVOMath.h"
#include <Box2D/Box2D.h>
#include "RVOShape.h"

namespace RVO
{
	RVOAgent::RVOAgent(RVO::RVOSimualtor *simualtor):simualtor(simualtor), maxSpeed(0), maxNeighbours(0), neightDist(0), radius(0), timeHorizon(0), timeHorizonObst(0), ID(0)
	{}

	RVOAgent::RVOAgent(const RVOAgent& agent)
	{
		maxNeighbours = agent.maxNeighbours;
		maxNeighbours = maxSpeed;
		radius = agent.radius;
		neightDist = agent.neightDist;
		timeHorizon = agent.timeHorizon;
		timeHorizonObst = agent.timeHorizonObst;
	}

	RVOAgent::RVOAgent(const RVOAgent *agent)
	{
		maxNeighbours = agent->maxNeighbours;
		maxSpeed = agent->maxSpeed;
		radius = agent->radius;
		neightDist = agent->neightDist;
		timeHorizon = agent->timeHorizon;
		timeHorizonObst = agent->timeHorizonObst;
	}

	void RVOAgent::computeNeighbours()
	{
		obstacleNeighbours.clear();

		float factor = timeHorizonObst * maxSpeed + radius;
		float range = factor * factor;

		simualtor->tree->computeObstacleNeighbours(this, range);

		agentNeighbours.clear();
		if (maxNeighbours > 0) {
			range = neightDist * neightDist;
			simualtor->tree->computeAgentNeighbours(this, range);
		}
	}

	void RVOAgent::computeNewVelocity()
	{
		lines.clear();

		const float invTimeHorizonObst = 1.0f/timeHorizonObst;
		for (size_t i = 0; i < obstacleNeighbours.size(); ++i)
		{
			const RVOObstacles *ob1 = obstacleNeighbours[i].second;
			const RVOObstacles *ob2 = ob1->next;

			const b2Vec2 relativePosition1 = ob1->point - position;
			const b2Vec2 relativePosition2 = ob2->point - position;

			bool alreadyCoverd = false;
			for (size_t j = 0; j < lines.size(); ++j)
			{
				if (det(invTimeHorizonObst * relativePosition1 - lines[j].point, lines[j].direction) - invTimeHorizonObst * radius >= RVO_EPSILON &&
					det(invTimeHorizonObst * relativePosition2 - lines[j].point, lines[j].direction) - invTimeHorizonObst *radius >= RVO_EPSILON)
				{
					alreadyCoverd = true;
					break;
				}
			}

			if (alreadyCoverd) continue;

			const float distSq1 = relativePosition1.LengthSquared();
			const float distSq2 = relativePosition2.LengthSquared();
			const float radiusSq = radius * radius;
			
			const b2Vec2 obstacleVec = ob2->point - ob1->point;
			const float s = (-relativePosition1 * obstacleVec) / obstacleVec.LengthSquared();
			const float distSqLine = (-relativePosition1 - s * obstacleVec).LengthSquared();

			RVOLine line;
			if (s < 0 && distSq1 < radiusSq) 
			{
				if (ob1->isConvex) 
				{
					line.point = b2Vec2_zero;
					b2Vec2 dir = b2Vec2(-relativePosition1.y, relativePosition1.x);
					dir.Normalize();
					line.direction = dir;
					lines.push_back(line);
				}

				continue;
			}
			else if (s > 1.0 && distSq2 <= radiusSq) 
			{
				if (ob2->isConvex && det(relativePosition2, ob2->unitDir) >= 0)
				{
					line.point = b2Vec2_zero;
					b2Vec2 dir = b2Vec2(-relativePosition2.y, relativePosition2.x);
					dir.Normalize();
					line.direction = dir;
					lines.push_back(line);
				}
				continue;
			}
			else if (s >= 0 && s < 1.0 && distSqLine <= radiusSq)
			{
				line.point = b2Vec2_zero;
				line.direction = -ob1->unitDir;
				lines.push_back(line);
				continue;
			}

			b2Vec2 leftLegDirection, rightLegDirection;
			if (s < 0 && distSqLine <= radiusSq)
			{
				if (!ob1->isConvex) continue;

				ob2 = ob1;
				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = b2Vec2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1);
				leftLegDirection *= (1 / distSq2);
				rightLegDirection = b2Vec2(relativePosition1.x * leg1 + relativePosition1.y * radius, -relativePosition1.x * radius + relativePosition1.y * leg1);
				rightLegDirection *= (1 / distSq2);
			}
			else if (s > 1 && distSqLine <= radiusSq)
			{
				if (!ob2->isConvex) continue;
				ob1 = ob2;
				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = b2Vec2(relativePosition2.x * leg2 - relativePosition2.y * radius, relativePosition2.x * radius + relativePosition2.y * leg2);
				leftLegDirection *= (1 / distSq2);
				rightLegDirection = b2Vec2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2);
				rightLegDirection *= (1 / distSq2);
			}
			else
			{
				if (ob1->isConvex)
				{
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = b2Vec2(relativePosition1.x * leg1 - relativePosition2.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1);
					leftLegDirection *= (1/distSq1);
				}
				else
				{
					leftLegDirection = -ob1->unitDir;
				}

				if (ob2->isConvex)
				{
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = b2Vec2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2);
					rightLegDirection *= (1 / distSq2);
				}
				else
				{
					rightLegDirection = -ob2->unitDir;
				}
			}

			const RVOObstacles *const leftNeighbor = ob1->prev;
			bool isLeftLegForeign = false, isRightLegForeign = false;

			if (ob1->isConvex && det(leftLegDirection, -leftNeighbor->unitDir) >= 0)
			{
				leftLegDirection = -leftNeighbor->unitDir;
				isLeftLegForeign = true;
			}
			
			if (ob2->isConvex && det(rightLegDirection, ob2->unitDir) <= 0)
			{
				rightLegDirection = ob2->unitDir;
				isRightLegForeign = true;
			}

			const b2Vec2 leftCutOff = invTimeHorizonObst * (ob1->point - position);
			const b2Vec2 rightCutOff = invTimeHorizonObst * (ob2->point - position);
			const b2Vec2 cutOffVec = rightCutOff - leftCutOff;

			const float t = (ob1 == ob2 ? 0.5f : ((velocity - leftCutOff) * cutOffVec) / cutOffVec.LengthSquared());
			const float tLeft = (velocity - leftCutOff) * leftLegDirection;
			const float tRight = (velocity - rightCutOff) * rightLegDirection;

			//
		}
	}

	void RVOAgent::insertNewNeighbour(const RVOAgent* agent, float &rangeSq)
	{
		if (this != agent)
		{
			const float distSq = (position - agent->position).LengthSquared();
			if (distSq < rangeSq)
			{
				if (agentNeighbours.size() < maxNeighbours) 
				{
					AgentHash hash = std::make_pair(distSq, agent);
					agentNeighbours.push_back(hash);
				}

				size_t i = agentNeighbours.size() - 1;

				while(i != 0 && distSq < agentNeighbours[i - 1].first) {
					agentNeighbours[i] = agentNeighbours[i - 1];
					--i;
				}

				agentNeighbours[i] = std::make_pair(distSq, agent);
				if (agentNeighbours.size() == maxNeighbours)
				{
					rangeSq = agentNeighbours.back().first;
				}
			}
		}
	}

	void RVOAgent::insertObstacleNeighbour(const RVOObstacles *ob ,float rangeSq)
	{
		const RVOObstacles *const nextObstacle = ob->next;

		const float distSq = distSqPointLineSegment(ob->point, nextObstacle->point, position);

		if (distSq < rangeSq) {
			obstacleNeighbours.push_back(std::make_pair(distSq, ob));

			size_t i = obstacleNeighbours.size() - 1;

			while (i != 0 && distSq < obstacleNeighbours[i - 1].first) {
				obstacleNeighbours[i] = obstacleNeighbours[i - 1];
				--i;
			}

			obstacleNeighbours[i] = std::make_pair(distSq, ob);
		}
	}

	void RVOAgent::update()
	{
		velocity = newVelocity;
		position += velocity * simualtor->timeStep;
	}

	bool RVOAgent::linearProgram1(const Lines& lines, size_t lineID, float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result)
	{
		const float dotProduct = lines[lineID].point.dot(lines[lineID].direction);
		const float discriminant = dotProduct * dotProduct + radius *radius - lines[lineID].point.LengthSquared();

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);
		float tLeft = -dotProduct - sqrtDiscriminant;
		float tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < lineID; ++i) {
			const float denominator = det(lines[lineID].direction, lines[i].direction);
			const float numerator = det(lines[i].direction, lines[lineID].point - lines[i].point);

			if (std::fabs(denominator) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (numerator < 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const float t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * lines[lineID].direction > 0.0f) {
				/* Take right extreme. */
				result = lines[lineID].point + tRight * lines[lineID].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineID].point + tLeft * lines[lineID].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineID].direction * (optVelocity - lines[lineID].point);

			if (t < tLeft) {
				result = lines[lineID].point + tLeft * lines[lineID].direction;
			}
			else if (t > tRight) {
				result = lines[lineID].point + tRight * lines[lineID].direction;
			}
			else {
				result = lines[lineID].point + t * lines[lineID].direction;
			}
		}

		return true;
	}

	size_t RVOAgent::linearProgram2(const Lines& lines, float radius, const b2Vec2& optVelocity, bool directionOpt, b2Vec2& result)
	{
		if (directionOpt) 
		{
			result = optVelocity * radius;
		}
		else if (optVelocity.LengthSquared() > radius * radius) 
		{
			b2Vec2 opt = optVelocity;
			opt.Normalize();
			result = opt * radius;
		}
		else 
		{
			result = optVelocity;
		}

		for (size_t i = 0; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const b2Vec2 tempResult = result;

				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return lines.size();
	}

	void RVOAgent::linearProgram3(const Lines &lines, size_t numObstLines, size_t beginLine, float radius, b2Vec2 &result)
	{
		float distance = 0.0f;

		for (size_t i = beginLine; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				Lines projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

				for (size_t j = numObstLines; j < i; ++j) {
					RVOLine line;

					float determinant = det(lines[i].direction, lines[j].direction);

					if (std::fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are parallel. */
						if (lines[i].direction.dot(lines[j].direction) > 0.0f) 
						{
							continue;
						}
						else 
						{
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}

					b2Vec2 vec = (lines[j].direction - lines[i].direction);
					vec.Normalize();
					line.direction = vec;
					projLines.push_back(line);
				}

				const b2Vec2 tempResult = result;
				-lines[i];
				if (linearProgram2(projLines, radius, b2Vec2(-lines[i].direction.y, lines[i].direction.x), true, result) < projLines.size()) {
					result = tempResult;
				}

				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}