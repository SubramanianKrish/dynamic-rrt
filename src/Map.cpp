#include "Map.h"
using namespace std;

Map::Map(int id){
    // Map1 
    if(id == 0){
    // Static Obstacles
    Obstacle* b0 = new Obstacle(0, true, 50, 10, 10, 20, 0); // sitting at the center of lower wall
    Obstacle* b1 = new Obstacle(1, true, 50, 80, 20, 40, 0); // hanging from the center of upper wall
    Obstacle* b2 = new Obstacle(2, true, 10, 50, 20, 20, 0); // hanging from the center of left wall
    
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    // static_obs_vec.push_back(b0);
    // static_obs_vec.push_back(b1);
    // static_obs_vec.push_back(b2);
    
    // Dynamic Obsracles
    DynamObstacle* d0 = new DynamObstacle(0, false, 35, 35, 3, 65, 35, 2000);
    DynamObstacle* d1 = new DynamObstacle(1, false, 65, 65, 3, 35, 65, 2000);
    DynamObstacle* d2 = new DynamObstacle(2, false, 40, 40, 3, 60, 60, 1000);

    dynam_obs_vec.push_back(d0);
    dynam_obs_vec.push_back(d1);
    dynam_obs_vec.push_back(d2);
    }
    
    //  Map2 - random purely moving circular obstacles
    // Static Obstacles
    else if(id == 1){
    Obstacle* b0 = new Obstacle(0, true, 75, 50, 20, 15, 0);
    Obstacle* b1 = new Obstacle(1, true, 30, 17.5, 10, 35, 0);
    Obstacle* b2 = new Obstacle(2, true, 30, 82.5, 10, 35, 0);

    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    static_obs_vec.push_back(b0);
    static_obs_vec.push_back(b1);
    static_obs_vec.push_back(b2);
  
    
    // Dynamic Obstacles
    DynamObstacle* d0 = new DynamObstacle(0, false, 40, 10, 2.5, 60, 30, 1000);
    // DynamObstacle* d1 = new DynamObstacle(1, false, 50, 70, 2, 95, 65, 500);
    DynamObstacle* d2 = new DynamObstacle(2, false, 85, 5, 1.75, 95, 50, 750);
    DynamObstacle* d3 = new DynamObstacle(3, false, 40, 85, 2.5, 55, 55, 2000);
    DynamObstacle* d4 = new DynamObstacle(4, false, 25, 40, 1.75, 40, 50, 1250);
    // DynamObstacle* d5 = new DynamObstacle(5, false, 50, 35, 2, 75, 35, 250);
    DynamObstacle* d6 = new DynamObstacle(6, false, 65, 60, 2.5, 65, 90, 400);
    DynamObstacle* d7 = new DynamObstacle(7, false, 5, 20, 1.75, 10, 85, 900);
    DynamObstacle* d8 = new DynamObstacle(8, false, 20, 10, 1.75, 5, 40, 500);
    DynamObstacle* d9 = new DynamObstacle(9, false, 25, 60, 2.5, 40, 55, 1500);
    DynamObstacle* d10 = new DynamObstacle(10, false, 5, 60, 2, 25, 40, 600);
    DynamObstacle* d11 = new DynamObstacle(11, false, 20, 50, 1.75, 60, 50, 1500);
    DynamObstacle* d12 = new DynamObstacle(12, false, 20, 60, 1.75, 10, 80, 800);
    DynamObstacle* d13 = new DynamObstacle(13, false, 80, 10, 2, 40, 50, 1200);
    // DynamObstacle* d14 = new DynamObstacle(14, false, 50, 40, 2.5, 80, 80, 500);
    DynamObstacle* d15 = new DynamObstacle(15, false, 20, 40, 2, 20, 60, 1750);
    DynamObstacle* d16 = new DynamObstacle(16, false, 60, 50, 1.75, 40, 35, 1000);
    
    // DynamObstacle* d17 = new DynamObstacle(17, false, 65, 90, 2.5, 90, 55, 500);
    DynamObstacle* d18 = new DynamObstacle(18, false, 45, 75, 2, 45, 35, 1500);
    DynamObstacle* d19 = new DynamObstacle(19, false, 10, 70, 1.75, 25, 70, 1800);

    dynam_obs_vec.push_back(d0);
    // dynam_obs_vec.push_back(d1);
    dynam_obs_vec.push_back(d2);
    dynam_obs_vec.push_back(d3);
    dynam_obs_vec.push_back(d4);
    // dynam_obs_vec.push_back(d5);
    dynam_obs_vec.push_back(d6);
    dynam_obs_vec.push_back(d7);
    dynam_obs_vec.push_back(d8);
    dynam_obs_vec.push_back(d9);
    dynam_obs_vec.push_back(d10);
    dynam_obs_vec.push_back(d11);
    dynam_obs_vec.push_back(d12);
    dynam_obs_vec.push_back(d13);
    // dynam_obs_vec.push_back(d14);
    dynam_obs_vec.push_back(d15);
    dynam_obs_vec.push_back(d16);
    // dynam_obs_vec.push_back(d17);
    dynam_obs_vec.push_back(d18);
    dynam_obs_vec.push_back(d19);
    }

    // Map 3 - maze
    else if (id == 2){
    Obstacle* b0 = new Obstacle(0, true, 40, 25, 80, 10, 0); // sitting at the center of lower wall
    Obstacle* b1 = new Obstacle(1, true, 70, 55, 60, 10, 0); // sitting at the center of lower wall
    Obstacle* b2 = new Obstacle(2, true, 30, 75, 60, 10, 0); // sitting at the center of lower wall

    static_obs_vec.push_back(b0);
    static_obs_vec.push_back(b1);
    static_obs_vec.push_back(b2);

    DynamObstacle* d0 = new DynamObstacle(0, false, 90, 25, 5, 80, 25, 1500);
    DynamObstacle* d1 = new DynamObstacle(1, false, 40, 60, 3, 40, 40, 900);
    DynamObstacle* d2 = new DynamObstacle(2, false, 60, 85, 2.5, 60, 95, 600);
    DynamObstacle* d3 = new DynamObstacle(3, false, 40, 95, 3, 40, 85, 600);
    DynamObstacle* d4 = new DynamObstacle(3, false, 20, 85, 4, 20, 95, 600);

    dynam_obs_vec.push_back(d0);
    dynam_obs_vec.push_back(d1);
    dynam_obs_vec.push_back(d2);
    dynam_obs_vec.push_back(d3);
    dynam_obs_vec.push_back(d4);
    }
}

bool Map::isValidPoint(double x, double y, double curr_step){
    // check against map boundary
    if (x>=x_size || y>=y_size || x<=0 || y<=0){ 
        return false;
    }

    // check against static obstacles
    for (Obstacle* static_obs : static_obs_vec){
        if (!static_obs->isStaticValid(x, y)){
            // obs idx in collision
            return false;
        }
    }

    // check against dynamic obstacles
    for (DynamObstacle* dynam_obs : dynam_obs_vec){
        if (!dynam_obs->isStaticValid(x, y, curr_step)){
            // obs idx in collision
            return false;
        }
    }

    return true;
}


vector<Obstacle*> Map::get_static_obs_vec(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return static_obs_vec;
}

vector<DynamObstacle*> Map::get_dynam_obs_vec(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return dynam_obs_vec;
}

double Map::get_x_size(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return x_size;
}

double Map::get_y_size(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return y_size;
}
