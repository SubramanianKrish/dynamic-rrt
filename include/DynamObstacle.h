#ifndef DYNAMOBSTACLE_H
#define DYNAMOBSTACLE_H

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

class DynamObstacle
{
private:
    int obs_idx;
    bool is_static;
    double sx, sy; // obstacle center position, start
    double ex, ey; // obstacle center position, end
    double path_steps;
    pair<double, double> velocity; // next_pos = (x+velocity.first, y+velocity.second)

    bool is_circle; // obstacle type (true:circle, false:rectangle;)
    double radius; // radius for circle only
    double length; // length for rectangle only
    double width; // width for rectangle only
    double theta; // theta for rectangle only

public:
    // constructors
    DynamObstacle();
    DynamObstacle(int obs_idx, bool is_static, double sx, double sy, double radius, double ex, double ey, double path_steps);
    DynamObstacle(int obs_idx, bool is_static, double x, double y, double length, double width, double theta, double ex, double ey, double path_steps);

    // check if this obstacle is static
    bool isStatic();

    // check if this obstacle is circle
    bool isCircle();

    // check if a lane formed by 2 points collision free
    bool isLaneCollisionFree(double x1, double y1, double x2, double y2, double curr_step);

    // check if a point is collision free with this obstacle
    bool isStaticValid(double x1, double y1, double curr_step);

    // check if a point is collision free with an obstacle interpolation
    bool isDynamicValid(double x1, double y1, double curr_step, double interp_step);

    // check if an edge is collision free with an obstacle interpolation
    bool isEdgeDynamicValid(double x1, double y1, double x2, double y2, double curr_step, double interp_step);

    // compute distance of obstacle center to a lane formed by 2 points
    double dist2Lane(double x1, double y1, double x2, double y2, double curr_step);

    // tell if the obstacle is moving from the start or from the end
    pair<bool, double> get_velocity_mode(double curr_step);

    // get current position of the obstacle
    pair<double, double> get_position(double curr_step);

    // get the properties of the obstacle
    vector<double> get_obs_feature(double curr_step);

};

#endif