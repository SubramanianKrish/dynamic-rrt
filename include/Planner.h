#include <algorithm>
#include <vector>
#include <random>

// #include "Obstacle.h"
#include "Map.h"

using namespace std;

struct Node{
  vector<double> position; // x, y
  int ID, parentID; 
  vector<int> neighbours; // Index of neighbors 
//   double f;
  double g; // Cost upto this point
};

class Planner
{
public:
    vector<Node> samples;
    Map* map;
    vector<double>* Start;
    vector<double>* Goal;
    int dim;
    // double*** plan;

    // Constructors
    Planner();
    Planner(Map* map, vector<double>* Start, vector<double>* Goal, int dim);

    // Add Vertex
    int add_vertex(vector<double> qnew);

    // Add Edge
    void add_edge(int child, int parent);

    // Random Sample
    vector<double> random_sample(vector<double> qrand);

    // Goal Sample
    vector<double> goal_sample(vector<double> qrand);

    // Nearest Neighbor
    int nearest_neighbor(vector<double> qrand);

    // Valid Edge
    bool valid_edge(vector<double> qrand, int qnearID, vector<double> &qnew, double E);

    // Extend
    void extend(vector<double> qrand; double E);

    // Reached Goal
    bool reached_goal(int index);
}