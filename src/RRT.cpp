#include <algorithm>
#include <vector>
#include <random>
#include <iostream>

// #include "Obstacle.h"
// #include "Map.h"
// #include "Node.h"
#include "RRT.h"

using namespace std;

// Constructors
RRT::RRT(Map* map, Node* Start, Node* Goal){
    this->map = map;
    this->Start = Start;
    this->Goal = Goal;

    dis = std::uniform_real_distribution<>(1.0, 100.0);
    dis_x = std::uniform_real_distribution<>(0, map->get_x_size());
    dis_y = std::uniform_real_distribution<>(0, map->get_y_size());

    gen.seed(rd());    
}

// Add Vertex
int RRT::add_vertex(double qnew_x, double qnew_y)
{
    int size = this->samples.size();
    Node* new_node = new Node();
    this->samples.push_back(new_node);
    this->samples[size]->x = qnew_x;
    this->samples[size]->y = qnew_y;
    this->samples[size]->ID = size;

    return size;
}

// Add Edge
void RRT::add_edge(int child, int parent)
{   
    this->samples[child]->parentID = parent;
    this->samples[parent]->neighbors.push_back(child);
    this->samples[parent]->ptrNeighbors.push_back(this->samples[child]);
    
    return;
}

// Random Sample
void RRT::random_sample(double* qrand_x, double* qrand_y)
{
    *qrand_x = dis_x(gen);
    *qrand_y = dis_y(gen);
    // std::cout << "Inside the random sample genereator" << std::endl;
    return;
}

// Goal Sample
void RRT::goal_sample(double* qrand_x, double* qrand_y)
{
    *qrand_x = this->Goal->x;
    *qrand_y = this->Goal->y;
    return;
}

// Nearest Neighbor
int RRT::nearest_neighbor(double qrand_x, double qrand_y)
{
    double dist, min_dist = 1e5;
    int index;
    for(int i = 0; i < this->samples.size(); i++)
    {
        dist = sqrt(pow(qrand_x - this->samples[i]->x, 2) + pow(qrand_y - this->samples[i]->y, 2));
        if(min_dist > dist)
        {
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

// Valid Edge
bool RRT::valid_edge(double qrand_x, double qrand_y, int qnearID, double* qnew_x, double* qnew_y, double E, double cur_time)
{   
    double qnear_x = this->samples[qnearID]->x;
    double qnear_y = this->samples[qnearID]->y;
    double dist = sqrt(pow(qrand_x - qnear_x, 2) + pow(qrand_y - qnear_y, 2)); 
    int num_steps = 10;
    double min_step = E / num_steps;
    for(int i = 0; i < num_steps; i++)
    {
        *qnew_x = qnear_x + (min_step * (i + 1) * (qrand_x - qnear_x) / dist);
        *qnew_y = qnear_y + (min_step * (i + 1) * (qrand_y - qnear_y) / dist);
        if(!this->map->isValidPoint(*qnew_x, *qnew_y, cur_time))
        {
            //cout << "No Valid Edge" << endl;
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
    //cout << "Valid Edge" << endl;
    return true;
}

// Extend
void RRT::extend(double qrand_x, double qrand_y, double E, double cur_time)
{   
    double qnew_x, qnew_y;
    int qnearID = nearest_neighbor(qrand_x, qrand_y);
    bool status = valid_edge(qrand_x, qrand_y, qnearID, &qnew_x, &qnew_y, E, cur_time);
    if(status)
    {
        int childID = add_vertex(qnew_x, qnew_y);
        add_edge(childID, qnearID);
        this->samples[childID]->g = sqrt(pow(this->samples[qnearID]->x - qnew_x, 2) + pow(this->samples[qnearID]->y - qnew_y, 2)) + this->samples[qnearID]->g; 
    }
    return;
}

// Reached Goal
bool RRT::reached_goal(int index)
{
    if(this->samples[index]->x == this->Goal->x && this->samples[index]->y == this->Goal->y)
    {
        return true;
    }
    return false;
}

// Backtrack
void RRT::backtrack(vector<Node*>& plan)
{
    int child = this->samples.size() - 1;
    plan.push_back(this->samples[child]);
    int parent = this->samples[child]->parentID;
    while(parent >= 0)
    {
        child = parent;
        plan.push_back(this->samples[child]);
        parent = this->samples[child]->parentID;
    }

    cout << "Inside backtrack: " << plan.size() << endl;
    return;
}

// Rand90
bool RRT::Rand90()
{
    if(dis(gen) <= 90)
    {
        return true;
    }
    return false;
}

// Tree
vector<Node*> RRT::getTree(){
    return samples;
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
/*
// RRT Star
// Neighbors in Radius
void RRT::neighbors_radius(int qnewID, double radius, vector<int> &neighborID)
{   
    double qnew_x = this->samples[qnewID]->x, qnew_y = this->samples[qnewID]->y;
    for(int i = 0; i < this->samples.size())
    {   
        if(i == qnewID) continue;
        double curr_sample_x = this->samples[i]->x, curr_sample_y = this->samples[i]->y;
        double dist = sqrt(pow(curr_sample_x - qnew_x, 2) + pow(curr_sample_y - qnew_y, 2));
        if(dist < radius)
        {   
            int num_steps = 10;
            double min_step = dist / num_steps;
            double intermediate_x = qnew_x, intermediate_y = qnew_y;
            for(int j = 0; j < num_steps; j++)
            {
                intermediate_x = qnew_x + (min_step * (j + 1) * (curr_sample_x - qnew_x) / dist);
                intermediate_y = qnew_y + (min_step * (j + 1) * (curr_sample_y - qnew_y) / dist);
                if(!this->map->isValidPoint(intermediate_x, intermediate_y)) break;
            }
        }
        neighborID.push_back(i);
    }
    return;
}

// Add Edge with Minimum Cost
void RRT::add_mincost_edge(int qnewID, vector<int> &neighborID)
{
    int min_index;
    double mincost = this->samples[qnewID]->g;
    for(int i = 0; i < neighborID.size(); i++)
    {    
        double cost = sqrt(pow(this->samples[neighborID[i]]->x - this->samples[qnewID]->x, 2) + pow(this->samples[neighborID[i]]->y - this->samples[qnewID]->y, 2)) + this->samples[neighborID[i]]->g;
        if(mincost > cost)
        {
            min_index = neighborID[i];
            mincost = cost;
        }
    }
    add_edge(qnewID, min_index);
    this->samples[qnewID]->g = mincost;
    return;
}

// // Update Edge
// void RRT::update_edge(int qnewID, int qnearID)
// {
//     add_edge(qnewID, qnearID);
//     this->samples[qnearID]->g = sqrt(pow(this->samples[qnewID]->x - this->samples[qnearID]->x, 2) + pow(this->samples[qnewID]->y - this->samples[qnearID]->y, 2)) + this->samples[qnewID]->g;
//     return;
// }

// Rewire
void RRT::rewire(int qnewID, vector<int> &neighborID)
{
    for(int i = 0; i < neighborID.size(); i++)
    {   
        if(this->samples[qnewID]->parentID == neighborID[i]) continue;
        double cost = sqrt(pow(this->samples[qnewID]->x - this->samples[neighborID[i]]->x, 2) + pow(this->samples[qnewID]->y - this->samples[neighborID[i]]->y, 2)) + this->samples[qnewID]->g;
        if(this->samples[neighborID[i]]->g > cost)
        {
            // Necessary to Check Valid Edge?
            add_edge(qnewID, neighborID[i]);
            this->samples[neighborID[i]]->g = cost;
        }
    }
    return;
}

// Radius for RRT*
double RRT::radius(double E, double gamma)
{
    auto v = this->samples.size();
    double delta = 3.1415; // Volume of Unit Hyperball
    double value = sqrt(gamma * log(v) / (v * delta));
    double r = min(value, E);
    return r;
}

// Extend and Rewire
void RRT::extend_rewire(double qrand_x, double qrand_y, double E, double gamma)
{
    double qnew_x, qnew_y;
    int qnearID = nearest_neighbor(qrand_x, qrand_y);
    bool status = valid_edge(qrand_x, qrand_y, qnearID, &qnew_x, &qnew_y, E);
    if(status)
    {   
        // Add qnew to tree
        int childID = add_vertex(qnew_x, qnew_y);
        add_edge(childID, qnearID);
        this->samples[childID]->g = sqrt(pow(this->samples[qnearID]->x - qnew_x, 2) + pow(this->samples[qnearID]->y - qnew_y, 2)) + this->samples[qnearID]->g; 

        // Find neighbors in radius of qnew
        vector<int> neighborID;
        double r = radius(E, gamma);
        neighbors_radius(childID, r, neighborID);
        if(neighborID.size() == 0) return;
        // Rewire
        add_mincost_edge(qnewID, neighborID);
        rewire(childID, neighborID);
    }
    return;
}*/
