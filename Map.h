#ifndef MAP_H
#define MAP_H

#include <vector>
#include "Obstacle.h"
#include "DynamObstacle.h"
using namespace std;

class Map
{
private:
    double x_size = 100.0;
    double y_size = 100.0;
    vector<Obstacle> static_obs_vec; // static obstacles
    vector<DynamObstacle> dynam_obs_vec; // dynamic obstacles

public:
    Map();
    bool isValidPoint(double x, double y, double curr_step);

    vector<Obstacle> get_static_obs_vec();
    vector<DynamObstacle> get_dynam_obs_vec();

    double get_x_size();
    double get_y_size();
};

#endif