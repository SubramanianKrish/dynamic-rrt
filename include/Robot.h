/*
    Robot class contains the planner and creates path and destinations
    for world update. Connected to the world, position updated by the
    world and time can be queried from the world. Omni-directional motion.
*/

#pragma once

#include <utility>
#include <mutex>
#include <iostream>

#include "RRT.h"
// #include "World.h" - Using forward declaration to break circular dependency
#include "Node.h"

using namespace std;

class World;

class Robot{
private:
    double x, y;            // in m
    const double vel;       // in m/s
    Node* next_destination; // goal buffer
    Node* goal;             // goal node

    World* world;   // handle to the world
    Map* map;       // handle to the map
    
    vector<Node*> plan;     // current plan
    RRT* rrt;

    std::mutex poseMtx;
    std::mutex destMtx;

public:
    // list of ctors
    Robot();
    Robot(double x, double y, double vel);
    Robot(double x, double y, double vel, double x_goal, double y_goal, Map* ptrMap);
    
    // Getters
    bool getGoal(double& gx, double& gy);
    void getRobotPose(double &rx, double &ry);
    bool getDestination(double &dx, double& dy);
    double getVel();
    vector<Node*> getCurrentTree();

    // Setters
    void setWorld(World* world);                            // main links robot to world
    void setRobotPose(const double& x, const double& y);    // world updates robot pose
    void setDestination(Node* dest);                        // planner should update destination

    bool robotAtGoal();
    bool robotAtDestination();
    void generatePlan();
    void replan();
    void generateReplan(Node* Replan_Start, Node* Replan_Goal, double radius_zone, vector<Node*> replan_plan, unsigned long int cur_time, double time_hor)
    bool isValidDynamic(double radius_zone, double qrand_x, double qrand_y, unsigned long int cur_time, double time_hor);
};
