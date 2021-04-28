/*
    World class updates time and moves robot
*/

#pragma once

#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>

#include "Map.h"
#include "Robot.h"

class World{
private:
    Map* map;
    Robot* robot;

    int updateRatems; // in millisecond  
    
    unsigned long int system_time;
    std::mutex timeMtx;

public:
    
    // ctor
    World(Map* ptrMap, Robot* ptrRobot, int rate);

    // Update function to bind to thread
    void update();

    // update time
    void updateTime();

    // time query
    unsigned long int get_system_time();
    Map* getMap();
};
