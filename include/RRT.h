#include <algorithm>
#include <vector>
#include <random>

// #include "Obstacle.h"
// #include "Map.h"
#include "Node.h"

using namespace std;

std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis_x(0, this->map.get_x_size());
std::uniform_real_distribution<> dis_y(0, this->map.get_y_size());
std::uniform_real_distribution<> dis(1, 100);

class RRT
{
public:
    vector<Node*> samples;
    Map* map;
    Node* Start;
    Node* Goal;
    vector<Node*> plan;

    RRT(Map* map, Node* Start, Node* Goal, vector<Node*> plan);

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
    bool valid_edge(double qrand_x, double qrand_y, int qnearID, double &qnew_x, double &qnew_y, double E);

    // Extend
    void extend(double qrand_x, double qrand_y, double E);

    // Reached Goal
    bool reached_goal(int index);

    // Bactrack
    void backtrack(vector<Node*> plan);

    // Rand90
    bool Rand90();

    // Valid Edge for RRT Connect
    // bool valid_edge_connect(double qrand_x, double qrand_y, int qnearID, double &qnew_x, double &qnew_y);

    // // Extend for RRT Connect
    // void extend_connect(double qrand_x, double qrand_y);
};