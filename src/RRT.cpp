#include <algorithm>
#include <vector>
#include <random>

// #include "Obstacle.h"
// #include "Map.h"
// #include "Node.h"
#include "RRT.h"

using namespace std;

// Constructors
RRT::RRT(Map* map, Node* Start, Node* Goal, vector<Node*> plan){
    this->map = map;
    this->Start = Start;
    this->Goal = Goal;
    this->plan = plan;
}

// Add Vertex
int RRT::add_vertex(double qnew_x, double qnew_y)
{
    int size = this->samples.size();
    this->samples.push_back(Node());
    this->samples[size].x = qnew_x;
    this->samples[size].y = qnew_y;
    this->samples[size].ID = size;
}

// Add Edge
inline void RRT::add_edge(int child, int parent)
{   
    this->samples[child].parentID = parent;
    return;
}

// Random Sample
inline void RRT::random_sample(double* qrand_x, double* qrand_y);
{
    *qrand_x = dis_x(gen);
    *qrand_y = dis_y(gen);
    return;
}

// Goal Sample
inline void RRT::goal_sample(double* qrand_x, double* qrand_y);
{
    *qrand_x = this->Goal.x;
    *qrand_y = this->Goal.y;
    return;
}

// Nearest Neighbor
int RRT::nearest_neighbor(double qrand_x, double qrand_y)
{
    double dist, min_dist = INT8_MAX;
    int index;
    for(int i = 0; i < this->samples.size(); i++)
    {
        dist = sqrt(pow(qrand_x - this->samples[i].x, 2) + pow(qrand_y - this->samples[i].y, 2));
        if(min_dist > dist)
        {
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

// Valid Edge
bool RRT::valid_edge(double qrand_x, double qrand_y, int qnearID, double* qnew_x, double* qnew_y, double E)
{   
    double qnear_x = this->samples[qnearID].x;
    double qnear_y = this->samples[qnearID].y;
    double dist = sqrt(pow(qrand_x - qnear_x, 2) + pow(qrand_y - qnear_y, 2)); 
    int num_steps = 10;
    double min_step = E / num_steps;
    for(int i = 0; i < num_steps; i++)
    {
        *qnew_x = qnear_x + (min_step * (qrand_x - qnear_x) / dist);
        *qnew_y = qnear_y + (min_step * (qrand_y - qnear_y) / dist);
        if(!this->map.isValidPoint(*qnew_x, *qnew_y))
        {
            return false;
        }
        double dist_qnew = sqrt(pow(qrand_x - *qnew_x, 2) + pow(qrand_y - *qnew_y, 2));
        if(dist_qnew < min_step)
        {
            *qnew_x = qrand_x;
            *qnew_y = qrand_y;
            return true;
        } 
    }
    return true;
}

// Extend
void RRT::extend(double qrand_x, double qrand_y, double E)
{   
    double qnew_x, qnew_y;
    int qnearID = nearest_neighbor(qrand_x, qrand_y);
    bool status = bool valid_edge(qrand_x, qrand_y, qnearID, &qnew_x, &qnew_y, E);
    if(status)
    {
        int childID = add_vertex(qnew_x, qnew_y);
        add_edge(childID, qnearID);
        this->samples[childID].g = sqrt(pow(this->samples[qnearID].x - qnew_x, 2) + pow(this->samples[qnearID].y - qnew_y, 2)) + this->samples[qnearID].g; 
    }
    return;
}

// Reached Goal
inline bool RRT::reached_goal(int index)
{
    if(this->samples[index].x == this->Goal.x && this->samples[index].y == this->Goal.y)
    {
        return true;
    }
    return false;
}

// Backtrack
void RRT::backtrack(vector<Node*> plan)
{
    int child = this->sample.size() - 1;
    plan.push_back(this->sample[child]);
    int parent = this->sample[child].parentID;
    while(!parent == -1)
    {
        child = parent;
        plan.push_back(this->sample[child]);
        parent = this->sample[child].parentID;
    }
}

// Rand90
inline bool RRT::Rand90()
{
    if(dis(gen) <= 90)
    {
        return true;
    }
    return false;
}

// Valid Edge for RRT Connect
// bool valid_edge_connect(double qrand_x, double qrand_y, int qnearID, double* qnew_x, double* qnew_y)
// {
//     double qnear_x = this->samples[qnearID].x;
//     double qnear_y = this->samples[qnearID].y;
//     double dist = sqrt(pow(qrand_x - qnear_x, 2) + pow(qrand_y - qnear_y, 2)); 
//     // int num_steps = 10;
//     double dist_qnew = sqrt(pow(qrand_x - *qnew_x, 2) + pow(qrand_y - *qnew_y, 2));
//     double min_step = 0.1;
//     while(dist_qnew < min_step)
//     {
//         *qnew_x = qnear_x + (min_step * (qrand_x - qnear_x) / dist);
//         *qnew_y = qnear_y + (min_step * (qrand_y - qnear_y) / dist);
//     }
//     qnew_x = qrand_x;
//     qnew_y = qrand_y;
//     return true;

//     // for(int i = 0; i < num_steps; i++)
//     // {
//     //     qnew_x = qnear_x + (min_step * (qrand_x - qnear_x) / dist);
//     //     qnew_y = qnear_y + (min_step * (qrand_y - qnear_y) / dist);
//     //     if(!this->map.isValidPoint(qnew_x, qnew_y))
//     //     {
//     //         return false;
//     //     }
//     //     double dist_qnew = sqrt(pow(qrand_x - qnew_x, 2) + pow(qrand_y - qnew_y, 2));
//     //     if(dist_qnew < min_step)
//     //     {
//     //         qnew_x = qrand_x;
//     //         qnew_y = qrand_y;
//     //         return true;
//     //     } 
//     // }
//     // return true;
// }

// // Extend for RRT Connect
// void extend_connect(double qrand_x, double qrand_y)
// {
//     double qnew_x, qnew_y;
//     int qnearID = nearest_neighbor(qrand_x, qrand_y);
//     bool status = bool valid_edge_connect(qrand_x, qrand_y, qnearID, &qnew_x, &qnew_y);
//     if(status)
//     {
//         int childID = add_vertex(qnew_x, qnew_y);
//         add_edge(childID, qnearID);
//         this->samples[childID].g = sqrt(pow(this->samples[qnearID].x - qnew_x, 2) + pow(this->samples[qnearID].y - qnew_y, 2)) + this->samples[qnearID].g; 
//     }
//     return;
// }