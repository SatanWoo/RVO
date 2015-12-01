//
//  KDTree.cpp
//  Crowd Simulation
//
//  Created by z on 15/5/26.
//  Copyright (c) 2015年 SatanWoo. All rights reserved.
//

#include <stdio.h>
#include "KDTree.h"
#include <math.h>
#include <algorithm>

KDTree::KDTree(std::vector<KDTuple> points, int k)
{
    this->dimension = k;
    this->root = new KDNode(KDSplitAxisX);
    this->buildKDTree(this->root, points, KDSplitAxisX);
}

KDTree::~KDTree()
{
    recursiveClearTree(root);
}

void KDTree::searchKNearestNeighbours(const KDTuple &goal, std::vector<KDTuple> &neighbours, double radius)
{
    if (root == NULL) return;
    
    std::deque<KDNode *> path;
    path.push_back(root);
    search(goal, neighbours, radius, path, false);
}

void KDTree::search(const KDTuple& goal, std::vector<KDTuple>& neighbours, double radius, std::deque<KDNode *> searchPath, bool isBack)
{
    if (searchPath.empty()) return;
    
    KDNode *last = searchPath.back();
    
    KDTuple p = last->points.front();
    
    double dis = distance(last->points.front(), goal);
    
    //std::cout << last->points.front()[0] << " || " << last->points.front()[1] << std::endl;
    
    if (last->isLeaf())
    {
        searchPath.pop_back();
        
        if (dis <= radius && last->points.front()._groupID == goal._groupID)
        {
            neighbours.push_back(last->points.front());
        }
        
        return search(goal, neighbours, radius, searchPath, true);
    }
    
    if (isBack)
    {
        if (dis <= radius && last->points.front()._groupID == goal._groupID)
        {
            neighbours.push_back(last->points.front());
        }
        
        searchPath.pop_back();
        double plane = p[last->splitAxis];
        double planeDiff = fabs(plane - goal[last->splitAxis]);
        
        //std::cout << planeDiff << std::endl;
        
        if (planeDiff < radius)
        {
            if (p[last->splitAxis] < goal[last->splitAxis])
            {
                // Go to Left Child;
                if (last->left != NULL)
                {
                    searchPath.push_back(last->left);
                }
            }
            else
            {
                // Go to Right Child
                if (last->right != NULL)
                {
                    searchPath.push_back(last->right);
                }
            }
        }
    }
    else
    {
        if (p[last->splitAxis] > goal[last->splitAxis])
        {
            // Go to Left Child;
            if (last->left != NULL)
            {
                searchPath.push_back(last->left);
                search(goal, neighbours, radius, searchPath, isBack);
            } else {
                search(goal, neighbours, radius, searchPath, true);
            }
        }
        else
        {
            if (last->right != NULL)
            {
                // Go to Right Child
                searchPath.push_back(last->right);
                search(goal, neighbours, radius, searchPath, isBack);
            } else {
                search(goal, neighbours, radius, searchPath, true);
            }
        }
    }
}

void KDTree::buildKDTree(KDNode *root, const std::vector<KDTuple> &points, KDSplitAxis axis)
{
    if (root == NULL || points.size() <= 0) return;
    if (points.size() == 1)
    {
        root->points.push_back(points[0]);
        return;
    }
    
    std::vector<double> splitValues;
    for (size_t i = 0; i < points.size(); i++)
    {
        splitValues.push_back(points[i][axis]);
    }
    std::sort(splitValues.begin(), splitValues.end());
    
    double middle = findMiddleValue(splitValues);
    
    KDSplitAxis next = (KDSplitAxis)(1 - axis);
    
    std::vector<KDTuple> leftPoints;
    std::vector<KDTuple> rightPoints;
    for (size_t i = 0; i < points.size(); i++)
    {
        double val = points[i][axis];
        if (val < middle)
        {
            leftPoints.push_back(points[i]);
        }
        else if (val > middle)
        {
            rightPoints.push_back(points[i]);
        }
        else
        {
            root->points.push_back(points[i]);
        }
    }
    
    if (leftPoints.size() > 0)
    {
        root->left = new KDNode(next);
        root->left->parent = root;
        buildKDTree(root->left, leftPoints, next);
    }
    
    if (rightPoints.size() > 0) {
        root->right = new KDNode(next);
        root->right->parent = root;
        buildKDTree(root->right, rightPoints, next);
    }
}

double KDTree::distance(const KDTuple &a, const KDTuple &b)
{
    double xDiff = a[0] - b[0];
    double yDiff = a[1] - b[1];
    
    return sqrt(xDiff * xDiff + yDiff * yDiff);
}

double KDTree::findMiddleValue(const std::vector<double> values)
{
    size_t size = values.size();
    return values[size/2];
}

void KDTree::recursiveClearTree(KDNode *node)
{
    if (node == NULL) return;
    
    if (node->left != NULL) recursiveClearTree(node->left);
    if (node->right != NULL) recursiveClearTree(node->right);
    
    delete node;
    node = NULL;
}
