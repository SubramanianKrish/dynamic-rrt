#include <iostream>
#include <algorithm>
#include <vector>
#include <random>

#include "Obstacle.h"
#include "Map.h"
#include "Node.h"
#include "RRT.h"

using namespace std;

static void RRT_Initial(Map *map, Node* Start, Node* Goal)
{
    int number_samples = 10000;
    double E = 1;

    if(!map->isValidPoint(Start->x, Start->y, 0))
    {
        cout<<"Invalid Start Position" << endl;
        return;
    }
    if(!map->isValidPoint(Goal->x, Goal->y, 0))
    {
        cout<<"Invalid Goal Position" << endl;
        return;
    }

    double qrand_x, qrand_y;
    cout << "checkpoint 1: goal and start are okay" << endl;


    RRT rrt_initial(map, Start, Goal);
    cout << "checkpoint 2: created rrt_initial object" << endl;

    int start_index = rrt_initial.add_vertex(Start->x, Start->y);
    rrt_initial.samples[start_index]->parentID = -1;

    bool goal_sample = false;
    bool goal_reached = false;

    for(int i = 0; i < number_samples; i++)
    {
        cout << "inside sample generator loop" << endl;
        if(rrt_initial.Rand90())
        {
            do
            {
                cout << "trying to create a random sample inside rand90 block" << endl;
                rrt_initial.random_sample(&qrand_x, &qrand_y);
            }while(!map->isValidPoint(qrand_x, qrand_y, 0));
        }
        else
        {
            cout << "trying to sample goal inside else block" << endl;
            goal_sample = true;
            rrt_initial.goal_sample(&qrand_x, &qrand_y);
        }
        cout << "checkpoint 3: Sample Created" << endl;

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
        rrt_initial.backtrack(rrt_initial.plan);
        cout << "Number of Nodes " << rrt_initial.samples.size() << endl;
    }
    else
    {
        cout << "No Path Found" << endl;
    }
}
/*
static void RRT_Star_Initial(Map *map, Node* Start, Node* Goal)
{   
    // Node* Start = new Node(x, y);

    int number_samples = 10000;
    double E = 1;
    double gamma = 6000;

    if(!map->isValidPoint(Start->x, Start->y))
    {
        cout<<"Invalid Start Position" << endl;
        return;
    }
    if(!map->isValidPoint(Goal->x, Goal->y))
    {
        cout<<"Invalid Goal Position" << endl;
        return;
    }

    double qrand_x, qrand_y;
    cout << "checkpoint 1: goal and start are okay" << endl;


    RRT rrt_star_initial(map, Start, Goal);
    cout << "checkpoint 2: created rrt_initial object" << endl;

    int start_index = rrt_star_initial.add_vertex(Start->x, Start->y);
    rr_star_initial.samples[start_index]->g = 0;
    rrt_star_initial.samples[start_index]->parentID = -1;

    bool goal_sample = false;
    bool goal_reached = false;

    for(int i = 0; i < number_samples; i++)
    {
        cout << "inside sample generator loop" << endl;
        if(rrt_star_initial.Rand90())
        {
            do
            {
                cout << "trying to create a random sample inside rand90 block" << endl;
                rrt_star_initial.random_sample(&qrand_x, &qrand_y);
            }while(!map->isValidPoint(qrand_x, qrand_y));
        }
        else
        {
            cout << "trying to sample goal inside else block" << endl;
            goal_sample = true;
            rrt_star_initial.goal_sample(&qrand_x, &qrand_y);
        }
        cout << "checkpoint 3: Sample Created" << endl;

        rrt_star_initial.extend_rewire(qrand_x, qrand_y, E, gamma);

        if(goal_sample)
        {
            goal_sample = false;
            goal_reached = rrt_star_initial.reached_goal(rrt_star_initial.samples.size() - 1);
        }

        if(goal_reached)
        {
            cout << "Path Found" << endl;
            break;
        }
    }

    if(goal_reached)
    {
        rrt_star_initial.backtrack(plan);
        cout << "Number of Nodes " << rrt_star_initial.samples.size() << endl;
    }
    else
    {
        cout << "No Path Found" << endl;
    }
}
*/
int main(){
    cout << "Testing rrt .." << endl;
    Map *my_map = new Map();
    Node* start = new Node(5.0, 15.0);
    Node* goal = new Node(20.0, 20.0);

    vector<Node*> cur_plan;
    RRT_Initial(my_map, start, goal);
}
