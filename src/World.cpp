#include "World.h"

World::World(Map* ptrMap, int rate): map(ptrMap), updateRatems(rate), system_time(0) {}

unsigned long int World::get_system_time(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    return system_time;
}

void World::updateTime(){
    std::unique_lock<std::mutex> timeLock(timeMtx);
    system_time += 1;
}

void World::update(){
    while(get_system_time() < 1000){
        updateTime();
        // cout << system_time << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(updateRatems)); // sleep for 1 second
         
        // update Robot pose
    }
}
