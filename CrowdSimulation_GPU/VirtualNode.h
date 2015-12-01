//
//  VirtualNode.h
//  Crowd Simulation
//
//  Created by z on 15/6/2.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_VirtualNode_h
#define Crowd_Simulation_VirtualNode_h

#include <vector>
#include "Box2D.h"
#include "Agent.h"

struct VirtualNode
{
    double maxX;
    double maxY;
    
    double minX;
    double minY;
    
    double radius;
    
    b2Vec2 center;
    b2Vec2 velocity;
    b2Vec2 ff;
    b2Vec2 force;
    
    int group;
    std::vector<Agent *> allNodes;
    
    void build()
    {
        maxX = INT_MIN;
        maxY = INT_MIN;
        
        minX = INT_MAX;
        minY = INT_MAX;
        
        center = b2Vec2_zero;
        velocity = b2Vec2_zero;
        
        int size = allNodes.size();
        for (int i = 0; i < size; i++)
        {
            Agent *ai = allNodes[i];
            
            b2Vec2 pos = ai->getPosition();
            b2Vec2 vec = ai->getVelocity();
            center += pos;
            velocity += vec;
            
            if (pos.x > maxX) maxX = pos.x;
            if (pos.x < minX) minX = pos.x;
            
            if (pos.y > maxY) maxY = pos.y;
            if (pos.y < minY) minY = pos.y;
        }
        
        center *= 1 / size;
        velocity *= 1 / size;
        
        double xDiff = maxX - minX;
        double yDiff = maxY - minY;
        radius = sqrt(xDiff * xDiff + yDiff * yDiff);
    }
    
    void dispatch(double delta)
    {
        int size = allNodes.size();
        for (int i = 0; i < size; i++)
        {
            Agent *ai = allNodes[i];
            ai->body->ApplyLinearImpulse(force * delta, ai->getPosition());
        }
    }
};

#endif
