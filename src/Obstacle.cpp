#include "Obstacle.h"

using namespace std;


// constructors
Obstacle::Obstacle() {};
Obstacle::Obstacle(int obs_idx, bool is_static, double x, double y, double radius) {
    // initialize an obstacle class
    this->obs_idx = obs_idx;
    this->is_static = is_static;
    this->x = x;
    this->y = y;

    this->is_circle = true;
    this->radius = radius;
}
Obstacle::Obstacle(int obs_idx, bool is_static, double x, double y, double length, double width, double theta) {
    // initialize an obstacle class
    this->obs_idx = obs_idx;
    this->is_static = is_static;
    this->x = x;
    this->y = y;

    this->is_circle = false;
    this->length = length;
    this->width = width;
    this->theta = theta; // [0, pi/2)
}

bool Obstacle::isStatic() {
    return this->is_static;
}

bool Obstacle::isCircle() {
    return this->is_circle;
}

bool Obstacle::isLaneCollisionFree(double x1, double y1, double x2, double y2) {
    // check if this obstacle is free from collision with an edge
    // (x1, y1): lane segment start position
    // (x2, y2): lane segment end position
    double dist = dist2Lane(x1, y1, x2, y2);
    if (this->radius >= dist) {
        return false;
    }
    else {
        return true;
    }
}

bool Obstacle::isStaticValid(double x1, double y1) {
    // check if this obstacle is free from collision with a point
    double dist = sqrt((x1 - this->x) * (x1 - this->x) + (y1 - this->y) * (y1 - this->y));
    if (this->is_circle) { // circle obstacle
        if (this->radius >= dist) {
            return false;
        }
        else {
            return true;
        }
    }
    else { // rectangular obstacle
        double A = tan(this->theta);
        double B = -1;
        double C = this->y - this->x * tan(this->theta);
        double dist1 = abs(A * x1 + B * y1 + C) / sqrt(A * A + B * B);
        double dist2 = sqrt(dist * dist - dist1 * dist1);

        if (dist1 > this->width / 2 || dist2 > this->length / 2) {
            return true;
        }
        else {
            return false;
        }
    }
}

double Obstacle::dist2Lane(double x1, double y1, double x2, double y2) {
    // compute coeff for lane segment Ax+By+C=0
    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1 * y2 - x2 * y1;

    // compute distance of obstacle center to lane
    double dist = abs(A * this->x + B * this->y + C) / sqrt(A * A + B * B);

    return dist;
}

vector<double> Obstacle::get_obs_feature() {
    // get the properties of the obstacle
    // for circular obstacles:      {x, y, radius}
    // for rectangular obstacles:   {x, y, length, width, theta}
    vector<double> obs_features;

    // position
    obs_features.push_back(x);
    obs_features.push_back(y);

    if (is_circle) {
        // size
        obs_features.push_back(radius);
    }
    else {
        // size
        obs_features.push_back(length);
        obs_features.push_back(width);
        obs_features.push_back(theta);
    }

    return obs_features;
}