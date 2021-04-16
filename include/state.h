/*
    Data structure for a State or Node in the tree
*/

#pragma once

#include <vector>
#include <random>
#include <iostream>

using namespace std;

class State{

typedef std::uniform_real_distribution<double> urd_double;
typedef std::mt19937 random_engine;

public:
    // data variables;
    double x, y;
    double cost_to_come;

    // graph variables
    State* parent;
    vector<State*> neighbors;

    bool blocked;

    // delete default ctor. Need to construct states with x,y value
    State() = delete;
    
    // ctor with given x, y
    State(const int& x, const int& y);

    // copy ctor [maybe useful]
    State(const State& other);

    // ctor for random sample
    State(random_engine& engine, urd_double& dist_x, urd_double& dist_y);

    //dtor for clearing memory
    ~State();

    void print(const string& name);
};
