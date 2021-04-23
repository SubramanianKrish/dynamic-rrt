#include "Map.h"
using namespace std;

Map::Map(){
    Obstacle* b0 = new Obstacle(0, true, 50, 10, 10, 20, 0); // sitting at the center of lower wall
    Obstacle* b1 = new Obstacle(1, true, 50, 80, 20, 40, 0); // hanging from the center of upper wall
    Obstacle* b2 = new Obstacle(2, true, 10, 50, 20, 20, 0); // hanging from the center of left wall

    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    static_obs_vec.push_back(b0);
    static_obs_vec.push_back(b1);
    static_obs_vec.push_back(b2);
}

// <TO-DO: Move this outside the map, into the planner maybe(?)>
bool Map::isValidPoint(double x, double y){
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

    return true;
}

vector<Obstacle*> Map::get_static_obs_vec(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return static_obs_vec;
}

double Map::get_x_size(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return x_size;
}

double Map::get_y_size(){
    std::unique_lock<std::mutex> mtxLoc(mapMutex);
    return y_size;
}
