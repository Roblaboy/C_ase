#ifndef IPLANNINGALGORITHM_H
#define IPLANNINGALGORITHM_H
#include "ExplorationTaskState.h"
#include "ExplorationPlanner/kinematics/PlanarPose.h"
#include <vector>

/**
  * class PlanningAlgorithm
  *
  */
class IPlanningAlgorithm
{
public:  
    virtual ~IPlanningAlgorithm() {}//基类析构函数一般定义为虚析构函数，以确保函数间的正确关联

    /**
   * @return PlanarPose
   * @param  initial_state
   */
    virtual bool plan(const ExplorationTaskState& state) = 0;
    virtual void getPlan(std::vector<PlanarPose> &plan) const = 0;
};

#endif // IPLANNINGALGORITHM_H
