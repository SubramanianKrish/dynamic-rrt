#include "Robot.h"

Robot::Robot()
    : x(0), y(0), vel(0), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel)
    : x(x), y(y), vel(vel), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel, double x_goal, double y_goal, Map* ptrMap)
    : x(x), y(y), vel(vel), next_destination(NULL), world(NULL), map(ptrMap) 
    {
        goal = new Node(x_goal, y_goal);
        generatePlan();
        cout << plan.size() << endl;
    }

bool Robot::getGoal(double& gx, double & gy){
    if(goal == NULL){
        cout << "ERR: Goal is empty. Could not get goal" << endl;
        return false;
    } 
    gx = goal->x;
    gy = goal->y;
}

void Robot::getRobotPose(double &rx, double &ry){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    rx = x;
    ry = y;
}

bool Robot::getDestination(double &dx, double &dy){
    std::unique_lock<std::mutex> DestLock(destMtx);
    if(next_destination == NULL){
        // cout << "WARN: Next destination is queried but found to be NULL. Maybe still planning..?" << endl;
        return false;
    }

    dx = next_destination->x;
    dy = next_destination->y;
}

double Robot::getVel() {return vel;}

void Robot::setWorld(World* world){
    this->world = world;
}

void Robot::setRobotPose(const double& x_new, const double& y_new){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    this->x = x_new;
    this->y = y_new;
}

void Robot::setDestination(Node* dest){
    std::unique_lock<std::mutex> DestLock(destMtx);
    this->next_destination = dest;
}


void Robot::generatePlan()
{   
    Node* Start = new Node(x, y);

    int number_samples = 10000;
    double E = 1;

    if(!map->isValidPoint(Start->x, Start->y))
    {
        cout<<"Invalid Start Position" << endl;
        return;
    }
    if(!map->isValidPoint(goal->x, goal->y))
    {
        cout<<"Invalid Goal Position" << endl;
        return;
    }

    double qrand_x, qrand_y;
    // cout << "checkpoint 1: goal and start are okay" << endl;


    rrt = new RRT(map, Start, goal);
    // cout << "checkpoint 2: created rrt_initial object" << endl;

    int start_index = rrt->add_vertex(Start->x, Start->y);
    rrt->samples[start_index]->parentID = -1;

    bool goal_sample = false;
    bool goal_reached = false;

    for(int i = 0; i < number_samples; i++)
    {
        // cout << "inside sample generator loop" << endl;
        if(rrt->Rand90())
        {
            do
            {
                // cout << "trying to create a random sample inside rand90 block" << endl;
                rrt->random_sample(&qrand_x, &qrand_y);
            }while(!map->isValidPoint(qrand_x, qrand_y));
        }
        else
        {
            // cout << "trying to sample goal inside else block" << endl;
            goal_sample = true;
            rrt->goal_sample(&qrand_x, &qrand_y);
        }
        // cout << "checkpoint 3: Sample Created" << endl;

        rrt->extend(qrand_x, qrand_y, E);

        if(goal_sample)
        {
            goal_sample = false;
            goal_reached = rrt->reached_goal(rrt->samples.size() - 1);
        }

        if(goal_reached)
        {
            cout << "Path Found" << endl;
            break;
        }
    }

    if(goal_reached)
    {
        rrt->backtrack(plan);
        cout << "Number of Nodes " << rrt->samples.size() << endl;
    }
    else
    {
        cout << "No Path Found" << endl;
    }
}

vector<Node*> Robot::getCurrentTree(){
    return rrt->samples; // <May have to use accessor if private>
}
