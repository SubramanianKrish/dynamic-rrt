/*

    Class definition for the viewer. Would display the tree, obstacles, start and goal

*/

#pragma once

#include <iostream>
#include <mutex>

#include <opencv2/opencv.hpp>

#include "Map.h"
#include "Obstacle.h"
#include "World.h"

class viewer
{
private:
    cv::Mat background;
    int refresh_rate;               // viewer refresh rate in ms
    Map* map;
    World* world;
    Robot* robot;

    double map_width, map_height;
    const float scale_factor = 10.0;
    
    bool closeViewer;               // can be fired from main to stop viewer
    std::mutex closeViewerMutex;    // safeguards thread shared variable closeViewer

public:
    // ctor for viewer
    viewer(Map* ptrMap, World* ptrWorld, Robot* ptrRobot);

    // main function to display env - Bind to another thread
    void run();

    // utility drawing functions
    void DrawObstacle(vector<double>& features, cv::Mat& scene);
    void DrawRobot(cv::Mat& scene);
    void DrawGoal(cv::Mat& scene);

    // utils to get, set shared vars
    bool requestViewerClosure();
    bool isViewerClosed();
};
