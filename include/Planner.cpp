#include <algorithm>
#include <vector>
#include <random>

#include "Planner.h"
// #include "Obstacle.h"
// #include "Map.h"

using namespace std;

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

// Constructors
Planner::Planner(){};
Planner::Planner(Map* map, vector<double>* Start, vector<double>* Goal, int dim){
    this->map = map;
    this->Start = Start;
    this->Goal = Goal;
    this->dim = dim;
}

// Add Vertex
int add_vertex(vector<double> qnew)
{
    int size = this->samples.size();
    this->samples.push_back(Node());
    this->samples[size].position = qnew;
    this->samples[size].ID = size;
}

// Add Edge
void add_edge(int child, int parent)
{   
    this->samples[child].parentID = parent;
    return;
}

// Random Sample
vector<double> random_sample(vector<double> qrand)
{
    std::uniform_real_distribution<> dis_x(0, this->map.get_x_size());
    std::uniform_real_distribution<> dis_y(0, this->map.get_x_size());
    qrand[0] = dis_x(gen);
    qrand[1] = dis_y(gen);
}

// Goal Sample
vector<double> goal_sample(vector<double> qrand)
{
    qrand[0] = this->Goal[0];
    qrand[1] = this->Goal[1];
}

// Nearest Neighbor
int nearest_neighbor(vector<double> qrand)
{
    double dist, min_dist = INT8_MAX;
    int index;
    for(int i = 0; i < this->samples.size(); i++)
    {
        dist = sqrt(pow(qrand[0] - this->samples[i].position[0], 2) + pow(qrand[1] - this->samples[i].position[1], 2));
        if(min_dist > dist)
        {
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

// Valid Edge
bool valid_edge(vector<double> qrand, int qnearID, vector<double> &qnew, double E)
{   
    vector<double> qnear(this->dim) = this->samples[qnearID].position;
    double dist = sqrt(pow(qrand[0] - qnear[0], 2) + pow(qrand[1] - qnear[1], 2)); 
    int num_steps = 10;
    double min_step = E / num_steps;
    for(int i = 0; i < num_steps; i++)
    {
        qnew[0] = qnear[0] + (min_step * (qrand[0] - qnear[0]) / dist);
        qnew[1] = qnear[1] + (min_step * (qrand[1] - qnear[1]) / dist);
        if(!this->map.isValidPoint(qnew[0], qnew[1]))
        {
            return false;
        }
        double dist_qnew = sqrt(pow(qrand[0] - qnew[0], 2) + pow(qrand[1] - qnew[1], 2));
        if(dist_qnew < min_step)
        {
            qnew[0] = qrand[0];
            qnew[1] = qrand[1];
            return true;
        } 
    }
    return true;
}

// Extend
void extend(vector<double> qrand; double E)
{   
    vector<double> qnew(this->dim);
    int qnearID = nearest_neighbor(qrand);
    bool status = valid_edge(qrand, qnearID, qnew, E);
    if(status)
    {
        int childID = add_vertex(qnew);
        add_edge(childID, qnearID);
        this->samples[childID].g = sqrt(pow(this->samples[qnearID].position[0] - qnew[0], 2) + pow(this->samples[qnearID].position[1] - qnew[1], 2)) + this->samples[qnearID].g; 
    }
    return;
}

// Reached Goal
bool reached_goal(int index)
{
    if(this->samples[index].position[0] == this->Goal[0] && this->samples[index].position[1] == this->Goal[1])
    {
        return true;
    }
    return false;
}