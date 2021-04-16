#include "viewer.h"

using namespace std;

viewer::viewer(const cv::Mat& im): image(im), closeViewer(false), refresh_rate(10) {}

void viewer::run()
{   
    if(image.empty()){
        cout << "ERROR: Image is empty. Run cannot be executed inside viewer. Exiting..";
        return;
    }

    while(!isViewerClosed()){
        cv::imshow("Environment display", image);
        cv::waitKey(refresh_rate);
    }

    return;
}

bool viewer::requestViewerClosure(){
    std::unique_lock<std::mutex> closeViewerLock(closeViewerMutex);
    closeViewer = true;
    return true;
}

bool viewer::isViewerClosed(){
    std::unique_lock<std::mutex> closeViewerLock(closeViewerMutex);
    return closeViewer;
}
