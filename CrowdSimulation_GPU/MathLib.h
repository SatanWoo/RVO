//
//  MathLib.h
//  Crowd Simulation
//
//  Created by z on 15-4-29.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_MathLib_h
#define Crowd_Simulation_MathLib_h
#include <math.h>

class MathLib
{
public:
    static float min4(float a, float b, float c, float d)
    {
        float minVal = std::min(a, b);
        minVal = std::min(minVal, c);
        minVal = std::min(minVal, d);
        
        return minVal;
    }
};

#endif
