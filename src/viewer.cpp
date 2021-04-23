#include "viewer.h"

using namespace std;

viewer::viewer(Map* ptrMap): map(ptrMap), closeViewer(false), refresh_rate(25) // refresh at 40Hz
{}

void viewer::run()
{   
    cout << "Starting viewer .. " << endl;

    // Set up white background with map size
    int width = map->get_x_size(), height = map->get_y_size();
    background = cv::Mat((int)height*scale_factor, (int)width*scale_factor, CV_8UC3, cv::Scalar(255, 255, 255)); // white bg
    
    // Draw static obstacles onto background
    vector<Obstacle*> static_obs = map->get_static_obs_vec();
    for(Obstacle* cur_obst: static_obs){
        if(cur_obst->isStatic()){
            vector<double> features = cur_obst->get_obs_feature();

            // draw circular static obstacles
            if(cur_obst->isCircle()){
                if(features.size() != 3) cout << "ERROR: Something is wrong. Circle obst has "<< features.size() << " features" << endl;
                circle(background, cv::Point2f((features[0]*scale_factor), (height - features[1])*scale_factor), (int)features[2]*scale_factor, cv::Scalar(0,0,0),CV_FILLED);
            }

            // draw rectangular obstacles using fill convex poly
            else{
                if(features.size() != 5) cout << "ERROR: Something is wrong. Rect obst has "<< features.size() << " features" << endl;
                cv::Scalar color = cv::Scalar(0,0,0);
                cv::Point2f rec_center = cv::Point2f(features[0]*scale_factor, (height - features[1])*scale_factor);
                cv::Size2f rec_size = cv::Point2f(features[2]*scale_factor, features[3]*scale_factor); // check len,wid
                cv::RotatedRect cur_rect = cv::RotatedRect(rec_center, rec_size, -features[4]);
                
                cv::Point2f vertices2f[4];
                cur_rect.points(vertices2f);

                vector<cv::Point> vertices;
                for(int i=0; i<4; ++i) vertices.push_back(vertices2f[i]);
                cv::fillConvexPoly(background, vertices, cv::Scalar(0,0,0));
            }
        }
    }

    while(!isViewerClosed()){


        cv::imshow("Environment display", background);
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
