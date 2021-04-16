/*

Class definition for the viewer. Would display the tree, obstacles, start and goal

*/

#pragma once

#include <iostream>
#include <mutex>

#include <opencv2/opencv.hpp>

class viewer
{
private:
    cv::Mat image;
    int refresh_rate;       // viewer refresh rate in ms
    
    bool closeViewer;       // can be fired from main to stop viewer
    std::mutex closeViewerMutex;    // safeguards thread shared variable closeViewer

public:
    // ctor for viewer
    viewer(const cv::Mat& im);

    // main function to display env - Bind to another thread
    void run();

    // utils to get, set shared vars
    bool requestViewerClosure();
    bool isViewerClosed();
};
