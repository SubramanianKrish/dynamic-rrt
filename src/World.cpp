#include "World.h"

World::World(Map* ptrMap, Robot* ptrRobot, int rate)
    : map(ptrMap), robot(ptrRobot), updateRatems(rate), system_time(0) {}

unsigned long int World::get_system_time(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    return system_time;
}

void World::updateTime(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    system_time += 1;
}

Map* World::getMap() {return map;}

void World::update(){
    double dest_x, dest_y, robot_x, robot_y;
    double new_robot_x, new_robot_y;
    double robot_vel = robot->getVel();

    while(get_system_time() < 1e20){
        
        // increment sytem time
        updateTime();
         
        // Move robot if robot not at destination pose - simple position control const vel
        robot->getRobotPose(robot_x, robot_y); 
        if(robot->getDestination(dest_x, dest_y) && (robot_x != dest_x || robot_y != dest_y)) // if destination exists
        {
            // update robot position
            double vdt = robot_vel*updateRatems/1000;
            double dist = sqrt(pow(dest_x - robot_x, 2) + pow(dest_y - robot_y, 2));
            if(dist < vdt)
            {
                new_robot_x = dest_x;
                new_robot_y = dest_y;
            }
            else{
                new_robot_x = robot_x + vdt*(dest_x - robot_x)/dist;
                new_robot_y = robot_y + vdt*(dest_y - robot_y)/dist;
            }
            
            robot->setRobotPose(new_robot_x, new_robot_y);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(updateRatems)); // sleep
    }
}
