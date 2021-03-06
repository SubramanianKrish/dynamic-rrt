# pragma once

#include <algorithm>
#include <vector>
#include <random>

#include "Map.h"
#include "Node.h"

using namespace std;

class RRT
{
public:
    vector<Node*> samples;
    Map* map;
    Node* Start;
    Node* Goal;
    vector<Node*> plan;
    
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen; //Standard mersenne_twister_engine
    
    std::uniform_real_distribution<> dis;
    std::uniform_real_distribution<> dis_x;
    std::uniform_real_distribution<> dis_y;

    RRT(Map* map, Node* Start, Node* Goal);

    // Add Vertex
    int add_vertex(double qnew_x, double qnew_y);

    // Add Edge
    void add_edge(int child, int parent);

    // Random Sample
    void random_sample(double* qrand_x, double* qrand_y);

    // Goal Sample
    void goal_sample(double* qrand_x, double* qrand_y);

    // Nearest Neighbor
    int nearest_neighbor(double qrand_x, double qrand_y);

    // Valid Edge
    bool valid_edge(double qrand_x, double qrand_y, int qnearID, double *qnew_x, double *qnew_y, double E, double cur_time);

    // Extend
    void extend(double qrand_x, double qrand_y, double E, double cur_time);

    // Reached Goal
    bool reached_goal(int index);

    // Bactrack
    void backtrack(vector<Node*>& plan);

    // Rand90
    bool Rand90();

    // Tree <TODO: Add mutex for samples>
    vector<Node*> getTree();

    // Valid Edge for RRT Connect
    // bool valid_edge_connect(double qrand_x, double qrand_y, int qnearID, double &qnew_x, double &qnew_y);

    // // Extend for RRT Connect
    // void extend_connect(double qrand_x, double qrand_y);

    // RRT Star
    // Neighbors in Radius
    void neighbors_radius(int qnewID, double radius, vector<int> &neighborID);

    // Add Edge with Minimum Cost
    void add_mincost_edge(int qnewID, vector<int> &neighborID);

    // // Update Edge
    // void update_edge(int qnewID, int qnearID);

    // Rewire
    void rewire(int qnewID, vector<int> &neighborID);

    // Radius for RRT*
    double radius(double E, double gamma);

    // Extend and Rewire
    void extend_rewire(double qrand_x, double qrand_y, double E, double gamma);
};
