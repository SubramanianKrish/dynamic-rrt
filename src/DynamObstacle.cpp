#include "DynamObstacle.h"

using namespace std;


// constructors
DynamObstacle::DynamObstacle(){};
DynamObstacle::DynamObstacle(int obs_idx, bool is_static, double sx, double sy, double radius, double ex, double ey, double path_steps){
    // initialize an obstacle class
    this->obs_idx = obs_idx;
    this->is_static = is_static;
    this->sx = sx;
    this->sy = sy;

    this->is_circle = true;
    this->radius = radius;

    // dynamics
    this->ex = ex;
    this->ey = ey;
    this->path_steps = path_steps;

    // compute velocity
    this->velocity = make_pair((ex-sx)/path_steps, (ey-sy)/path_steps);
}
DynamObstacle::DynamObstacle(int obs_idx, bool is_static, double x, double y, double length, double width, double theta, double ex, double ey, double path_steps){
    // initialize an obstacle class
    this->obs_idx = obs_idx;
    this->is_static = is_static;
    this->sx = sx;
    this->sy = sy;

    this->is_circle = false;
    this->length = length;
    this->width = width;
    this->theta = theta; // [0, pi/2)

    // dynamics
    this->ex = ex;
    this->ey = ey;
    this->path_steps = path_steps;

    // compute velocity
    this->velocity = make_pair((ex-sx)/path_steps, (ey-sy)/path_steps);
}

bool DynamObstacle::isStatic(){
    return this->is_static;
}

bool DynamObstacle::isCircle(){
    return this->is_circle;
}

bool DynamObstacle::isLaneCollisionFree(double x1, double y1, double x2, double y2, double curr_step){
    // check if this obstacle is free from collision with an edge
    // (x1, y1): lane segment start position
    // (x2, y2): lane segment end position
    double dist = dist2Lane(x1, y1, x2, y2, curr_step);
    if (this->radius >= dist){
        return false;
    }else{
        return true;
    }
}

bool DynamObstacle::isStaticValid(double x1, double y1, double curr_step){
    // check if this obstacle is free from collision with a point

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;

    double dist = sqrt((x1-x_curr)*(x1-x_curr)+(y1-y_curr)*(y1-y_curr));
    if (this->is_circle){ // circle obstacle
        if (this->radius >= dist){
            return false;
        }else{
            return true;
        }
    }else{ // rectangular obstacle
        double A = tan(this->theta);
        double B = -1;
        double C = y_curr - x_curr * tan(this->theta);
        double dist1 = abs(A*x1 + B*y1 + C) / sqrt(A*A + B*B);
        double dist2 = sqrt(dist*dist - dist1*dist1);

        if (dist1>this->width/2 || dist2>this->length/2){
            return true;
        }else{
            return false;
        }
    }
}

bool DynamObstacle::isDynamicValid(double x1, double y1, double curr_step, double interp_step){
    // check if this obstacle extension is free from collision with a point

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;
    // compute the next position
    // 1. compute the unit velocity
    pair<double, double> pos_delta = get_position(curr_step+1);
    double x_delta = pos_delta.first;
    double y_delta = pos_delta.second;
    double vx = x_delta - x_curr;
    double vy = y_delta - y_curr;
    
    // 2. compute the next position with interpolated unit velocity
    double x_next = vx * interp_step + x_curr;
    double y_next = vy * interp_step + y_curr;

    double dist = sqrt((x_next-x_curr)*(x_next-x_curr)+(y_next-y_curr)*(y_next-y_curr));
    double dist_curr = sqrt((x1-x_curr)*(x1-x_curr)+(y1-y_curr)*(y1-y_curr));
    double dist_next = sqrt((x1-x_next)*(x1-x_next)+(y1-y_next)*(y1-y_next));
    if (this->is_circle){ // circle obstacle
        // point enclosed by either circle
        if (this->radius>=dist_curr || this->radius>=dist_next){
            return false;
        }

        // compute two cosine at two endpoints
        double cos_gn = (dist_next*dist_next + dist*dist - dist_curr*dist_curr)/(2*dist_next*dist);
        double cos_gc = (dist_curr*dist_curr + dist*dist - dist_next*dist_next)/(2*dist_curr*dist);

        // if either cos is greater than 90 degree => outside => valid
        if (cos_gc<=0 || cos_gn<=0){
            return true;
        }

        // the point is in between of the two endpoints, check vertical distance
        double sin_gc = sqrt(1.0 - cos_gc*cos_gc);
        double dist_vert = dist_curr * sin_gc;
        if (this->radius>=dist_vert){
            return false;
        }

        return true;
    }else{ 
        cout << "FAILED COLLISION CHECKING: Not Implemented Rectangular Dynamics Obstacle" << endl;
    }
}

bool DynamObstacle::isEdgeDynamicValid(double x1, double y1, double x2, double y2, double curr_step, double interp_step){
    // inputs note: two endpoints of the edge (x1, y1), (x2, y2)
    // separately check 10 points in the middle

    // directional gradient
    double dx = (x2-x1) / 10;
    double dy = (y2-y1) / 10;

    // sequentially check each point
    for (int i=0; i<=10; ++i){
        if (!isDynamicValid(x1+dx*i, y1+dy*i, curr_step, interp_step)){
            return false;
        }
    }
    return true;
}

double DynamObstacle::dist2Lane(double x1, double y1, double x2, double y2, double curr_step){
    // compute coeff for lane segment Ax+By+C=0
    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1*y2 - x2*y1;

    // extract current position
    pair<double, double> pos_curr = get_position(curr_step);
    double x_curr = pos_curr.first;
    double y_curr = pos_curr.second;
    // compute distance of obstacle center to lane
    double dist = abs(A*x_curr + B*y_curr + C) / sqrt(A*A + B*B);

    return dist;
}

// tell if the obstacle is moving from the start or from the end
pair<bool, double> DynamObstacle::get_velocity_mode(double curr_step){
    pair<bool, double> res;

    double div = curr_step / path_steps;
    int step_rounds = floor(div);

    // check if the velocity mode is forward
    bool forward = false;
    if (step_rounds%2==0){
        forward = true;
    }

    // compute the applied time steps for current velocity mode
    double applied_steps = curr_step - path_steps * (double)step_rounds;

    res = make_pair(forward, applied_steps);
    
    return res;
}

// get current position of the obstacle
pair<double, double> DynamObstacle::get_position(double curr_step){
    pair<bool, double> velocity_mode = get_velocity_mode(curr_step);
    bool forward = velocity_mode.first;
    double applied_steps = velocity_mode.second;

    double x_curr, y_curr;
    if (forward){
        x_curr = sx + applied_steps * velocity.first;
        y_curr = sy + applied_steps * velocity.second;
    }else{
        x_curr = ex - applied_steps * velocity.first;
        y_curr = ey - applied_steps * velocity.second;
    }

    pair<double, double> pos_curr = make_pair(x_curr, y_curr);

    return pos_curr;
}

vector<double> DynamObstacle::get_obs_feature(double curr_step){
    // get the properties of the obstacle
    // for circular obstacles:      {x, y, radius}
    // for rectangular obstacles:   {x, y, length, width, theta}
    vector<double> obs_features;

    // position
    pair<double, double> pos_curr = get_position(curr_step);
    obs_features.push_back(pos_curr.first);
    obs_features.push_back(pos_curr.second);

    if (is_circle){
        // size
        obs_features.push_back(radius);
    }else{
        // size
        obs_features.push_back(length);
        obs_features.push_back(width);
        obs_features.push_back(theta);
    }

    return obs_features;
}
