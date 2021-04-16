#include "state.h"

State::State(const int& x, const int& y): x(x), y(y), parent(NULL), cost_to_come(0), blocked(false) {}

State::State(const State& other): x(other.x), y(other.y), parent(other.parent), 
                                  cost_to_come(other.cost_to_come), blocked(other.blocked)
{
    this->neighbors = other.neighbors; // copy the neighbor list
}

State::State(random_engine& engine, urd_double& dist_x, urd_double& dist_y): parent(NULL), cost_to_come(0)
{
    // generate a random sample
    x = dist_x(engine);
    y = dist_y(engine);
    blocked = false;
}

State::~State(){cout << "destroying state" << endl;}

void State::print(const string& name){
    cout << "------ " << name << ":" << x << " " << y << endl;
}
