//
//  KDObstacle.h
//  Crowd Simulation
//
//  Created by z on 15/6/2.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef __Crowd_Simulation__KDObstacle__
#define __Crowd_Simulation__KDObstacle__

#include <stdio.h>
#include "Box2D.h"


class KDTree;
class Agent;

class KDObstacle {
private:
    KDObstacle():_isConvex(false), _nextObstacle(NULL), _preObstacle(NULL), _id(0){};
    
    bool _isConvex;
    KDObstacle *_nextObstacle;
    KDObstacle *_preObstacle;
    
    b2Vec2 _unitDir;
    
    size_t _id;
    
    friend class Agent;
    friend class KDtree;
    //friend class MapContro
};


#endif /* defined(__Crowd_Simulation__KDObstacle__) */
