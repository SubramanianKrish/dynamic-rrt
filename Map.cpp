#include "Map.h"
using namespace std;

Map::Map(){
    // Static Obstacles
    Obstacle b0(0, true, 50, 10, 10, 20, 0); // sitting at the center of lower wall
    Obstacle b1(1, true, 50, 80, 20, 40, 0); // hanging from the center of upper wall
    Obstacle b2(2, true, 10, 50, 20, 20, 0); // hanging from the center of left wall

    static_obs_vec.push_back(b0);
    static_obs_vec.push_back(b1);
    static_obs_vec.push_back(b2);

    // Dynamic Obsracles
    DynamObstacle d0(0, false, 10, 10, 5, 70, 70, 60);
    DynamObstacle d1(1, false, 40, 50, 5, 10, 90, 30);

    dynam_obs_vec.push_back(d0);
    dynam_obs_vec.push_back(d1);

}

bool Map::isValidPoint(double x, double y, double curr_step){
    // check against map boundary
    if (x>=x_size || y>=y_size || x<=0 || y<=0){ 
        return false;
    }

    // check against static obstacles
    for (Obstacle static_obs : static_obs_vec){
        if (!static_obs.isStaticValid(x, y)){
            // obs idx in collision
            return false;
        }
    }

    // check against dynamic obstacles
    for (DynamObstacle dynam_obs : dynam_obs_vec){
        if (!dynam_obs.isStaticValid(x, y, curr_step)){
            // obs idx in collision
            return false;
        }
    }

    return true;
}

vector<Obstacle> Map::get_static_obs_vec(){
    return static_obs_vec;
}

vector<DynamObstacle> Map::get_dynam_obs_vec(){
    return dynam_obs_vec;
}

double Map::get_x_size(){
    return x_size;
}

double Map::get_y_size(){
    return y_size;
}