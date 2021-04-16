#include <iostream>
#include <fstream>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <chrono>

using namespace std;

class Obstacle
{
private:
    int obs_idx; // obstacle index
    double x, y; // obstacle center position
    double radius; // obstacle size

public:
    Obstacle(int obs_idx, double x, double y, double radius){
        // initialize an obstacle class
        this->obs_idx = obs_idx;
        this->x = x;
        this->y = y;
        this->radius = radius;
    }

    bool isLaneCollisionFree(double x1, double y1, double x2, double y2){
        // check if this obstacle is free from collision with an edge
        // (x1, y1): lane segment start position
        // (x2, y2): lane segment end position
        double dist = pointDist2Lane(x1, y1, x2, y2);
        if (this->radius >= dist){
            return false;
        }else{
            return true;
        }
    }

    bool isPointCollisionFree(double x1, double y1){
        // check if this obstacle is free from collision with a point
        double dist = sqrt((x1-this->x)*(x1-this->x)+(y1-this->y)*(y1-this->y));
        if (this->radius >= dist){
            return false;
        }else{
            return true;
        }
    }

    double pointDist2Lane(double x1, double y1, double x2, double y2){
        // compute coeff for lane segment Ax+By+C=0
        double A = y1 - y2;
        double B = x2 - x1;
        double C = x1*y2 - x2*y1;

        // compute distance of obstacle center to lane
        double dist = abs(A*this->x + B*this->y + C) / sqrt(A*A + B*B);

        return dist;
    }
};

class Map
{
private:
    double x_size = 100.0;
    double y_size = 100.0;
    unordered_map<int, Obstacle> idx_obs_map; // <obstacle index, obstacle x,y center pose>

public:
    bool isCollisionFree(double x, double y){
        if (x>=x_size || y>=y_size || x<=0 || y<=0){ // map boundary
            return false;
        }

        if ()

        return true;
    }

};