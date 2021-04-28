#include "Robot.h"

Robot::Robot()
    : x(0), y(0), vel(0), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel)
    : x(x), y(y), vel(vel), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel, double x_goal, double y_goal)
    : x(x), y(y), vel(vel), next_destination(NULL), world(NULL) 
    {
        goal = new Node(x_goal, y_goal);
    }

bool Robot::getGoal(double& gx, double & gy){
    if(goal == NULL){
        cout << "ERR: Goal is empty. Could not get goal" << endl;
        return false;
    } 
    gx = goal->x;
    gy = goal->y;
}

void Robot::getRobotPose(double &rx, double &ry){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    rx = x;
    ry = y;
}

bool Robot::getDestination(double &dx, double &dy){
    std::unique_lock<std::mutex> DestLock(destMtx);
    if(next_destination == NULL){
        // cout << "WARN: Next destination is queried but found to be NULL. Maybe still planning..?" << endl;
        return false;
    }

    dx = next_destination->x;
    dy = next_destination->y;
}

double Robot::getVel() {return vel;}

void Robot::setWorld(World* world){
    this->world = world;
}

void Robot::setRobotPose(const double& x_new, const double& y_new){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    this->x = x_new;
    this->y = y_new;
}

void Robot::setDestination(Node* dest){
    std::unique_lock<std::mutex> DestLock(destMtx);
    this->next_destination = dest;
}
