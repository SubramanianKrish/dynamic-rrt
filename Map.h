#ifndef MAP_H
#define MAP_H

#include <vector>
#include "Obstacle.h"
using namespace std;

class Map
{
private:
    double x_size = 100.0;
    double y_size = 100.0;
    vector<Obstacle> static_obs_vec; // static obstacles
    vector<Obstacle> dynamic_obs_vec; // dynamic obstacles 

public:
    Map();
    bool isValidPoint(double x, double y);

    vector<Obstacle> get_static_obs_vec();

    double get_x_size();
    double get_y_size();
};

#endif