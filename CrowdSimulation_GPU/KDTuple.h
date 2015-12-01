//
//  KDTuple.h
//  Crowd Simulation
//
//  Created by z on 15/5/26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#ifndef Crowd_Simulation_KDTuple_h
#define Crowd_Simulation_KDTuple_h

struct KDTuple {
    KDTuple(double x = 0.0, double y = 0.0, int ID = -1, int groupID = -1):xPos(x), yPos(y), _ID(ID), _groupID(groupID)
    {
    }
    
    double operator[](int x)const
    {
        if (x == 0) return xPos;
        else if (x == 1) return yPos;
        return 0.0;
    }
    
    double xPos;
    double yPos;
    
    int _ID;
    int _groupID;
};


#endif
