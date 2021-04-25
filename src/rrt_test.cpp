#include <iostream>
#include <algorithm>
#include <vector>
#include <random>

// #include "Obstacle.h"
// #include "Map.h"
// #include "Node.h"
#include "RRT.h"

using namespace std;

static void RRT_Initial(Map *map, Node* Start, Node* Goal, vector<Node*> plan)
{
    int number_samples = 10000;
    double E = 1;

    if(!map.isValidPoint(Start.x, Start.y))
    {
        cout<<"Invalid Start Position" << endl;
        return;
    }
    if(!map.isValidPoint(Goal.x, Goal.y))
    {
        cout<<"Invalid Goal Position" << endl;
        return;
    }

    double qrand_x, qrand_y;

    RRT rrt_initial(map, Start, Goal);

    int start_index = rrt_initial.add_vertex(Start.x, Start.y);
    rrt_initial.samples[start_index].parentID = -1;

    bool goal_sample = false;
    bool goal_reached = false;

    for(int i = 0; i < number_samples; i++)
    {
        if(rrt_initial.Rand90())
        {
            do
            {
                rrt_initial.random_sample(&qrand_x, &qrand_y);
            }while(!map.isValidPoint(qrand_x, q_randy))
        }
        else
        {
            goal_sample = true;
            rrt_initial.goal_sample(&qrand_x, &qrand_y);
        }

        rrt_initial.extend(qrand_x, qrand_y, E);

        if(goal_sample)
        {
            goal_sample = false;
            goal_reached = rrt_initial.reached_goal(rrt_initial.samples.size() - 1);
        }

        if(goal_reached)
        {
            cout << "Path Found" << endl;
            return;
        }
    }

    if(goal_reached)
    {
        rrt_initial.backtrack(plan);
        cout << "Number of Nodes " << rrt_initial.samples.size() << endl;
    }
    else
    {
        cout << "No Path Found" << endl;
    } 
}