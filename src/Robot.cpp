#include "Robot.h"
#include "World.h"

Robot::Robot()
    : x(0), y(0), vel(0), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel)
    : x(x), y(y), vel(vel), next_destination(NULL), goal(NULL), world(NULL) {}

Robot::Robot(double x, double y, double vel, double x_goal, double y_goal, Map* ptrMap)
    : x(x), y(y), vel(vel), next_destination(NULL), world(NULL), map(ptrMap), E(0.5),
      robot_node(NULL), local_goal(NULL)
    {
        goal = new Node(x_goal, y_goal);
        generatePlan();
        robot_node = plan.back();
        cout << "start has this many nbrs: " << robot_node->ptrNeighbors.size() << endl;
        cout << "generated plan is: " << plan.size() << endl;
    }

bool Robot::getGoal(double& gx, double & gy){
    if(goal == NULL){
        cout << "ERR: Goal is empty. Could not get goal" << endl;
        return false;
    } 
    gx = goal->x;
    gy = goal->y;
    return true;
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
    return true;
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

    int number_samples = 30000;
    

    if(!map->isValidPoint(Start->x, Start->y, 0))
    {
        cout<<"Invalid Start Position" << endl;
        return;
    }
    if(!map->isValidPoint(goal->x, goal->y, 0))
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
            }while(!map->isValidPoint(qrand_x, qrand_y, 0));
        }
        else
        {
            // cout << "trying to sample goal inside else block" << endl;
            goal_sample = true;
            rrt->goal_sample(&qrand_x, &qrand_y);
        }
        // cout << "checkpoint 3: Sample Created" << endl;

        rrt->extend(qrand_x, qrand_y, E, 0);

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

bool Robot::robotAtGoal(){
    std::unique_lock<std::mutex> poseLock(poseMtx);
    return (x == goal->x && y == goal->y);
}

bool Robot::robotAtDestination(){
    if(next_destination == NULL){
        cout << "Something is off. Checking robot at destination before making a destination" << endl;
        return false;
    }
    std::unique_lock<std::mutex> poseLock(poseMtx);
    return (x == next_destination->x && y == next_destination->y);
}

bool Robot::isValidDynamic(double replan_start_x, double replan_start_y, double radius_zone, double qrand_x, double qrand_y, unsigned long int cur_time, double time_hor) {
    //Check whether sample is within radius
    /*
    if (sqrt(pow(replan_start_x - qrand_x, 2) + pow(replan_start_y - qrand_y, 2)) > radius_zone) {
        return false;
        //cout << "Outside Radius " << endl;
    }
    */
    time_hor = time_hor*200;
    if (!map->isValidPoint(qrand_x, qrand_y, cur_time)) {
        return false;
        //cout << "Colliding with Static Obstacle " << endl;
    }
    vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();
    for (auto& dyn_obst : dynamic_obs)
    {
        if (!dyn_obst->isDynamicValid(qrand_x, qrand_y, cur_time, time_hor))
        {   
            //cout << "Colliding with Dynamic Obstacle " << endl;
            return false;
        }
    }
    return true;
}

void Robot::generateReplan(Node* Replan_Start, Node* Replan_Goal, double radius_zone, vector<Node*>& replan_plan, unsigned long int cur_time, double time_hor) {
    int number_samples = 30000;
    local_goal = Replan_Goal;
    // double E = 5;

    cout << "Asked to reach this goal: " << Replan_Goal->x << " " << Replan_Goal->y << endl;

    if (!isValidDynamic(Replan_Start->x, Replan_Start->y, radius_zone, Replan_Start->x, Replan_Start->y, cur_time, 0))
    {
        cout << "Invalid Start Position" << endl;
        return;
    }
    else
    {
        if(!isValidDynamic(Replan_Start->x, Replan_Start->y, radius_zone, Replan_Start->x, Replan_Start->y, cur_time, time_hor)){
            cout << "start valid at current time. But invalid at extended time. moving robot anyway" << endl;
            return;
        }
    }
    if (!isValidDynamic(Replan_Start->x, Replan_Start->y, radius_zone, Replan_Goal->x, Replan_Goal->y, cur_time, time_hor))
    {
        cout << "Invalid Goal Position in generateReplan" << endl;
        // Replan_Goal = goal;
        return;
    }

    double qrand_x, qrand_y;
    // cout << "checkpoint 1: goal and start are okay" << endl;


    RRT* rrt_replan = new RRT(map, Replan_Start, Replan_Goal);
    // cout << "checkpoint 2: created rrt_initial object" << endl;

    int start_index = rrt_replan->add_vertex(Replan_Start->x, Replan_Start->y);
    rrt_replan->samples[start_index]->parentID = -1;

    bool goal_sample = false;
    bool goal_reached = false;

    for (int i = 0; i < number_samples; i++)
    {
        // cout << "inside sample generator loop" << endl;
        if (rrt_replan->Rand90())
        {
            // do
            {
                // cout << "trying to create a random sample inside rand90 block: " << i << endl;
                rrt_replan->random_sample(&qrand_x, &qrand_y);                                  // <DOUBT> Why should sampled point be valid. Shouldn't extend be valid?
            } //while (isValidDynamic(Replan_Start->x, Replan_Start->y, radius_zone, qrand_x, qrand_y, cur_time, time_hor));
        }
        else
        {
            // cout << "trying to sample goal inside else block: " << i << endl;
            goal_sample = true;
            rrt_replan->goal_sample(&qrand_x, &qrand_y);
        }
        // cout << "checkpoint 3: Sample Created" << endl;

        // Modified Extend
        double qnew_x, qnew_y;

        int qnearID = rrt_replan->nearest_neighbor(qrand_x, qrand_y);
        //Check if edge is valid against static obstacles <possible BUG> Shouldn't valid edge be tested against dynamic valid edge or run static against interpolated time
        bool status = rrt_replan->valid_edge(qrand_x, qrand_y, qnearID, &qnew_x, &qnew_y, E, cur_time); 
        
        //Check if edge is valid against dynamic obstacles
        vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();
        for (auto& dyn_obst : dynamic_obs)
        {
            if (!dyn_obst->isEdgeDynamicValid(rrt_replan->samples[qnearID]->x, rrt_replan->samples[qnearID]->y, qnew_x, qnew_y, cur_time, time_hor))
            {
                // cout << "Collision Detected inside replan " << endl;
                status = false;
                --i;
                break;
            }
        }
        
        if (status)
        {
            int childID = rrt_replan->add_vertex(qnew_x, qnew_y);
            rrt_replan->add_edge(childID, qnearID);
            rrt_replan->samples[childID]->g = sqrt(pow(rrt_replan->samples[qnearID]->x - qnew_x, 2) + pow(rrt_replan->samples[qnearID]->y - qnew_y, 2)) + rrt_replan->samples[qnearID]->g;
        }

        if (goal_sample)
        {
            goal_sample = false;
            goal_reached = rrt_replan->reached_goal(rrt_replan->samples.size() - 1);
        }

        if (goal_reached)
        {
            cout << "Path Found" << endl;
            break;
        }
    }

    if (goal_reached)
    {
        rrt_replan->backtrack(replan_plan);
        cout << "Number of Nodes " << rrt_replan->samples.size() << endl;
    }
    else
    {
        cout << "No Path Found inside generateReplan: " << rrt_replan->samples.size() << endl;
        if(rrt_replan->samples.size() == number_samples) cout << "Exhausted samples. Hence no path: " << rrt_replan->samples.size() << endl;
    }

    cout << "Tried to reach this goal: " << Replan_Goal->x << " " << Replan_Goal->y << endl;
    cout << "Robot goal : " << goal->x << " " << goal->y << endl;
    local_tree = rrt_replan->samples;
}

void Robot::replan(){
    // checks state and updates next destination
    if(plan.empty()){
        cout << "ERR: There is no plan to replan with. Exiting replanning .." << endl;
        return;
    }

    int cur_step = plan.size()-1;
    if (next_destination == NULL) // first destination
    {
        next_destination = plan[cur_step];
     //   cout << "1st Destination Created " << endl;
    }

    // Variables
    double time_hor = 1; // currently in real world
    double vel = getVel();
    double dist_des = time_hor * vel;
    vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();
    // cout << "Cur step outside the loop: " << cur_step << endl;
    
    
    while(!robotAtGoal()){
        if(!robotAtDestination()) continue;
        // update robot node
        robot_node = next_destination;
        
        // replan as robot at destination
        // WRITE CODE HERE FOR REPLANNING + ALTERNATIVE NEXT DESTINATION UPDATE
        // Check whether edge is valid (Zhaoyuan)
        // Replan if not valid

        // Check Collision <test further on some specific cases - smaller env/dyn obs moving sides>
        // /*
        unsigned long int cur_time = world->get_system_time();
        // cout << "current time as seen inside replan: " << cur_time << endl;
        
        double dist_trav = 0;
        bool obs_blocking = false;
        int cur_idx = cur_step;
        bool first_edge = true;

        Node* replan_goal = NULL;
        Node* replan_start = plan[cur_step];

        while (dist_trav < dist_des)
        {   
            replan_goal = plan[cur_idx];
            if (plan[cur_idx]->x == goal->x && plan[cur_idx]->y == goal->y)
            {
                //cout << "Current Index is Goal" << endl; <BUG: goal would never be sent as next destination then?>
                break;
            }

            int next_idx = cur_idx - 1;
            dist_trav += sqrt(pow(plan[cur_idx]->x - plan[next_idx]->x, 2) + pow(plan[cur_idx]->y - plan[next_idx]->y, 2));
            
            for (auto& dyn_obst : dynamic_obs)
            {
                // <ENHANCEMENT: check at cur_time + dt is point valid, i.e time horizon zero>
                if (!dyn_obst->isEdgeDynamicValid(plan[cur_idx]->x, plan[cur_idx]->y, plan[next_idx]->x, plan[next_idx]->y, cur_time, 200*time_hor))
                {   
                    if(first_edge){
                        cout << "Collision Detected in valid edge dynamic check blok" << endl;
                        first_edge = false;
                    }
                    obs_blocking = true;
                }
            }
            cur_idx = next_idx;
        }
        // cout << "Desired Distance " << dist_des << " Travelled Distance " << dist_trav << endl;
        // cout << sqrt(pow(plan[cur_idx]->x - plan[cur_step]->x, 2) + pow(plan[cur_idx]->y - plan[cur_step]->y, 2)) << endl;
        
        
        if (obs_blocking)
        {
            // Replan RRT
            vector<Node*> replan_plan;

            // update the local planner goal
            while(!isValidDynamic(replan_start->x, replan_start->y, 0, replan_goal->x, replan_goal->y, cur_time, 200*time_hor)){
                    if(cur_idx >= 0){
                        replan_goal = plan[cur_idx];
                        cur_idx -= 1;
                    }
                    else{
                        cout << "trying to replan outside the path. Cannot replan anymore. Possibly obstacle moving into goal" << endl;
                        return;
                    }
            }
            // Replan Goal Index in original plan = cur_idx + 1
            // Size of original plan that is important will be cur_idx + 2
            
            // cout << "Before replanning " << replan_plan.size() << endl;
            
            // cout << "Start index in orignal tree: " << plan[cur_step]->ID << " Target index in OG tree: " << plan[cur_idx]->ID << endl;            
            generateReplan(replan_start, replan_goal, dist_des, replan_plan, cur_time, 200*time_hor);
            // cout << "After replanning " << replan_plan.size() << endl;
            
            if(replan_plan.size()){

                while (plan.size() > cur_idx + 2) {
                    plan.pop_back();
                }
                for (int i = 1; i < replan_plan.size(); i++) {
                    plan.push_back(replan_plan[i]);
                }
                cur_step = plan.size() - 1;
            }
        }
        
        // */
        --cur_step;
        cout << "Plan size after cur step decrement: " << plan.size() << " cur step in cur plan: " << cur_step << endl;
        if(cur_step >=0) setDestination(plan[cur_step]);
        
    }

    cout << "Reached goal!" << endl;
}

vector<Node*> Robot::getLocalTree(){
    return local_tree;
}

vector<Node*> Robot::getPlan(){
    return plan;
}

Node* Robot::getRobotNode(){ return robot_node; }
Node* Robot::getReplanGoal(){ return local_goal; }
