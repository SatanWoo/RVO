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

		const float invTimeHorizonObst = 1.0f / timeHorizonObst;

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbours.size(); ++i) {

			const RVOObstacles *obstacle1 = obstacleNeighbours[i].second;
			const RVOObstacles *obstacle2 = obstacle1->next;

			const b2Vec2 relativePosition1 = obstacle1->point - position;
			const b2Vec2 relativePosition2 = obstacle2->point - position;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < lines.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - lines[j].point, lines[j].direction) - invTimeHorizonObst * radius >= -RVO_EPSILON &&
					det(invTimeHorizonObst * relativePosition2 - lines[j].point, lines[j].direction) - invTimeHorizonObst * radius >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/* Not yet covered. Check for collisions. */

			const float distSq1 = relativePosition1.LengthSquared();
			const float distSq2 = relativePosition2.LengthSquared();

			const float radiusSq = radius * radius;

			const b2Vec2 obstacleVector = obstacle2->point - obstacle1->point;
			const float s = (-relativePosition1 * obstacleVector) / obstacleVector.LengthSquared();
			const float distSqLine = (-relativePosition1 - s * obstacleVector).LengthSquared();

			RVOLine line;

			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. Ignore if non-convex. */
				if (obstacle1->isConvex) {
					line.point = b2Vec2_zero;
					b2Vec2 vec = b2Vec2(-relativePosition1.y, relativePosition1.x);
					vec.Normalize();

					line.direction = vec;
					lines.push_back(line);
				}

				continue;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. Ignore if non-convex
				 * or if it will be taken care of by neighoring obstace */
				if (obstacle2->isConvex && det(relativePosition2, obstacle2->unitDir) >= 0.0f) {
					line.point = b2Vec2_zero;
					b2Vec2 vec = b2Vec2(-relativePosition2.y, relativePosition2.x);
					vec.Normalize();
					line.direction = vec;
					lines.push_back(line);
				}

				continue;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				line.point = b2Vec2_zero;
				line.direction = -obstacle1->unitDir;
				lines.push_back(line);
				continue;
			}

			/*
			 * No collision.
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs extend cut-off line when nonconvex vertex.
			 */

			b2Vec2 leftLegDirection, rightLegDirection;

			if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);
				leftLegDirection = b2Vec2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) * (1 / distSq1);
				rightLegDirection = b2Vec2(relativePosition1.x * leg1 + relativePosition1.y * radius, -relativePosition1.x * radius + relativePosition1.y * leg1) * (1 / distSq1);
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * Obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);
				leftLegDirection = b2Vec2(relativePosition2.x * leg2 - relativePosition2.y * radius, relativePosition2.x * radius + relativePosition2.y * leg2) * (1 / distSq2);
				rightLegDirection = b2Vec2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) * (1 / distSq2);
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = b2Vec2(relativePosition1.x * leg1 - relativePosition1.y * radius, relativePosition1.x * radius + relativePosition1.y * leg1) * (1 / distSq1);
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -obstacle1->unitDir;
				}

				if (obstacle2->isConvex) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = b2Vec2(relativePosition2.x * leg2 + relativePosition2.y * radius, -relativePosition2.x * radius + relativePosition2.y * leg2) * (1 / distSq2);
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = obstacle1->unitDir;
				}
			}

			/*
			 * Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added.
			 */

			const RVOObstacles *const leftNeighbor = obstacle1->prev;

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex && det(leftLegDirection, -leftNeighbor->unitDir) >= 0.0f) {
				/* Left leg points into obstacle. */
				leftLegDirection = -leftNeighbor->unitDir;
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex && det(rightLegDirection, obstacle2->unitDir) <= 0.0f) {
				/* Right leg points into obstacle. */
				rightLegDirection = obstacle2->unitDir;
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const b2Vec2 leftCutoff = invTimeHorizonObst * (obstacle1->point - position);
			const b2Vec2 rightCutoff = invTimeHorizonObst * (obstacle2->point - position);
			const b2Vec2 cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */

			/* Check if current velocity is projected on cutoff circles. */
			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity - leftCutoff) * cutoffVec) / cutoffVec.LengthSquared());
			const float tLeft = ((velocity - leftCutoff) * leftLegDirection);
			const float tRight = ((velocity - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				b2Vec2 vec = velocity - leftCutoff;
				vec.Normalize();
				const b2Vec2 unitW = vec;

				line.direction = b2Vec2(unitW.y, -unitW.x);
				line.point = leftCutoff + radius * invTimeHorizonObst * unitW;
				lines.push_back(line);
				continue;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				b2Vec2 vec = velocity - rightCutoff;
				vec.Normalize();
				const b2Vec2 unitW = vec;

				line.direction = b2Vec2(unitW.y, -unitW.x);
				line.point = rightCutoff + radius * invTimeHorizonObst * unitW;
				lines.push_back(line);
				continue;
			}

			/*
			 * Project on left leg, right leg, or cut-off line, whichever is closest
			 * to velocity.
			 */
			const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : (velocity - (leftCutoff + t * cutoffVec)).LengthSquared());
			const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : (velocity - (leftCutoff + tLeft * leftLegDirection)).LengthSquared());
			const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : (velocity - (rightCutoff + tRight * rightLegDirection)).LengthSquared());

			if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
				/* Project on cut-off line. */
				line.direction = -obstacle1->unitDir;
				line.point = leftCutoff + radius * invTimeHorizonObst * b2Vec2(-line.direction.y, line.direction.x);
				lines.push_back(line);
				continue;
			}
			else if (distSqLeft <= distSqRight) {
				/* Project on left leg. */
				if (isLeftLegForeign) {
					continue;
				}

				line.direction = leftLegDirection;
				line.point = leftCutoff + radius * invTimeHorizonObst * b2Vec2(-line.direction.y, line.direction.x);
				lines.push_back(line);
				continue;
			}
			else {
				/* Project on right leg. */
				if (isRightLegForeign) {
					continue;
				}

				line.direction = -rightLegDirection;
				line.point = rightCutoff + radius * invTimeHorizonObst * b2Vec2(-line.direction.y, line.direction.x);
				lines.push_back(line);
				continue;
			}
		}

		const size_t numObstLines = lines.size();

		const float invTimeHorizon = 1.0f / timeHorizon;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbours.size(); ++i) {
			const RVOAgent *const other = agentNeighbours[i].second;

			const b2Vec2 relativePosition = other->position - position;
			const b2Vec2 relativeVelocity = velocity - other->velocity;
			const float distSq = relativePosition.LengthSquared();
			const float combinedRadius = radius + other->radius;
			const float combinedRadiusSq = combinedRadius * combinedRadius;

			RVOLine line;
			b2Vec2 u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const b2Vec2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = w.LengthSquared();

				const float dotProduct1 = w.dot(relativePosition);

				if (dotProduct1 < 0.0f && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const b2Vec2 unitW = w * (1 /wLength);

					line.direction = b2Vec2(unitW.y, -unitW.x);
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = b2Vec2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) * (1/distSq);
					}
					else {
						/* Project on right leg. */
						line.direction = -b2Vec2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) * (1/distSq);
					}

					const float dotProduct2 = relativeVelocity * line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. Project on cut-off circle of time timeStep. */
				const float invTimeStep = 1.0f / simualtor->timeStep;

				/* Vector from cutoff center to relative velocity. */
				const b2Vec2 w = relativeVelocity - invTimeStep * relativePosition;

				const float wLength = w.Length();
				const b2Vec2 unitW = w / wLength;

				line.direction = b2Vec2(unitW.y, -unitW.x);
				u = (combinedRadius * invTimeStep - wLength) * unitW;
			}

			line.point = velocity + 0.5f * u;
			lines.push_back(line);
		}

		size_t lineFail = linearProgram2(lines, maxSpeed, prefVelocity, false, newVelocity);

		if (lineFail < lines.size()) {
			linearProgram3(lines, numObstLines, lineFail, maxSpeed, newVelocity);
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