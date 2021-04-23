#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <algorithm>
#include <vector>

using namespace std;

class Obstacle
{
private:
    int obs_idx;
    bool is_static;
    double x, y; // obstacle center position

    bool is_circle; // obstacle type (true:circle, false:rectangle;)
    double radius; // radius for circle only
    double length; // length for rectangle only
    double width; // width for rectangle only
    double theta; // theta for rectangle only

public:
    // constructors
    Obstacle();
    Obstacle(int obs_idx, bool is_static, double x, double y, double radius);
    Obstacle(int obs_idx, bool is_static, double x, double y, double length, double width, double theta);

    // check if this obstacle is static
    bool isStatic();

    // check if this obstacle is circle
    bool isCircle();

    // check if a lane formed by 2 points collision free
    bool isLaneCollisionFree(double x1, double y1, double x2, double y2);

    // check if a point is collision free with this obstacle
    bool isStaticValid(double x1, double y1);

    // compute distance of obstacle center to a lane formed by 2 points
    double dist2Lane(double x1, double y1, double x2, double y2);

    // Important: get the properties of the obstacle
    vector<double> get_obs_feature();
};

#endif