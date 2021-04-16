/*
Main thread for running the environment, planner and the viewer system

Authors: Subramanian Krishnan, Zhaoyuan Huo, Vaibhav Shete
*/

#include <iostream>
#include <thread>
#include <string>
#include <unistd.h>

#include "viewer.h"

using namespace std;

int main(){
    cout << "Starting program" << endl;

    string image_path = "../lavalle.jpg";
    cv::Mat planning_image = cv::imread(image_path);

    viewer* env_viewer = new viewer(planning_image);
    std::thread render_thread(&viewer::run, env_viewer);

    // Wait 10 seconds and close viewer
    for(int i=0; i<10; ++i){
        cout << "tic tic " << i << endl;
        sleep(1);
    }

    // send close signal
    while(!env_viewer->isViewerClosed())
        env_viewer->requestViewerClosure();

    // join viewer to main thread
    render_thread.join();

    return 0;
}
