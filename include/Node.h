# pragma once

#include <algorithm>
#include <vector>

using namespace std;

class Node{
public:
    double x, y; // Position of Node
    int ID, parentID; // Own ID, Parent ID
    vector<int> neighbors; // ID of Neighbors for RRT*
    double g; // gvalue of Node for RRT*

    // Constructors
    Node();
    Node(double x, double y);
};
