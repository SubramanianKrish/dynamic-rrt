/*
    Main thread for running the environment, planner and the viewer system

    Authors: Subramanian Krishnan, Zhaoyuan Huo, Vaibhav Shete
*/

#include <iostream>
#include <thread>
#include <string>
#include <memory>
#include <unistd.h>
// #include <Windows.h>

#include "viewer.h"
#include "Map.h"
#include "Robot.h"
#include "Obstacle.h"
#include "World.h"
#include "Node.h"
#include "RRT.h"

using namespace std;

int main(int argc, char* argv[]){
    cout << "Starting program" << endl;

    string env_id_str = argv[1];
    int env_id = stoi(env_id_str);
    Map* test_map = new Map(env_id);
    Robot* asimov = NULL;

    // Make robot
    if(env_id == 0)
    asimov = new Robot(50, 15, 10, 50, 80, test_map); // Map 1 - simple dyn
    else if(env_id == 1)
    asimov = new Robot(10, 90, 10, 90, 50, test_map); // Map 2 - dyn obs only
    else if(env_id == 2)
    asimov = new Robot(10, 5, 10, 10, 90, test_map); // Map 3 - maze


    // Make the world and bind to a bg thread
    World* test_world = new World(test_map, asimov, 5); // Update world at 200 Hz (5ms delay)
    std::thread worldUpdateThread(&World::update, test_world);

    // link robot to world
    asimov->setWorld(test_world);

    // Run the viewer with map and robot
    viewer* env_viewer = new viewer(test_map, test_world, asimov); // Update viewer at 40 HZ (25 ms delay)
    std::thread render_thread(&viewer::run, env_viewer);

    std::uniform_real_distribution<double> x_dist(0,20.0);
    std::uniform_real_distribution<double> y_dist(-5.0,0);
    std::mt19937 random_engine;

    // generate 5 states and close viewer
    
    for(int i=0; i<3; ++i){
        cout << "Do whatever" << endl;
        //sleep(1);
        sleep(1);
    }
    
    /*
     double gx, gy;
     asimov->getGoal(gx, gy);
     Node* goal = new Node(100, 100);
     asimov->setDestination(goal);

    // send close signal to viewer
     while(!env_viewer->isViewerClosed())
       env_viewer->requestViewerClosure();
    */

    asimov->replan();

    // join viewer to main thread
    render_thread.join();
    worldUpdateThread.join();

    return 0;
}
