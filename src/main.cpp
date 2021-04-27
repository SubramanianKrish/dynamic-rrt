/*
    Main thread for running the environment, planner and the viewer system

    Authors: Subramanian Krishnan, Zhaoyuan Huo, Vaibhav Shete
*/

#include <iostream>
#include <thread>
#include <string>
#include <memory>
#include <unistd.h>

#include "viewer.h"
#include "Map.h"
#include "Obstacle.h"
#include "World.h"

using namespace std;

int main(){
    cout << "Starting program" << endl;

    Map* test_map = new Map();

    // Make the world and bind to a bg thread
    World* test_world = new World(test_map, 5); // Update world at 200 Hz (5ms delay)
    std::thread worldUpdateThread(&World::update, test_world);

    // Run the viewer with map and robot
    viewer* env_viewer = new viewer(test_map, test_world); // Update viewer at 40 HZ (25 ms delay)
    std::thread render_thread(&viewer::run, env_viewer);

    std::uniform_real_distribution<double> x_dist(0,20.0);
    std::uniform_real_distribution<double> y_dist(-5.0,0);
    std::mt19937 random_engine;

    // generate 5 states and close viewer
    for(int i=0; i<5; ++i){
        // auto rand_state = make_shared<State>(random_engine, x_dist, y_dist);
        // rand_state->print("current random state");
        cout << "Can do other stuff here" << endl;
        sleep(1);
    }

    // send close signal to viewer
    while(!env_viewer->isViewerClosed())
        env_viewer->requestViewerClosure();
    
    // join viewer to main thread
    render_thread.join();
    worldUpdateThread.join();

    return 0;
}
