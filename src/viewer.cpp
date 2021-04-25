#include "viewer.h"

using namespace std;

viewer::viewer(Map* ptrMap): map(ptrMap), closeViewer(false), refresh_rate(25) // refresh at 40Hz
{
    map_width = map->get_x_size();
    map_height = map->get_y_size();
}

void viewer::DrawObstacle(Obstacle* obst, cv::Mat& scene){
    vector<double> features = obst->get_obs_feature();
    if(features.size() == 3)
        circle(scene, cv::Point2f((features[0]*scale_factor), (map_height - features[1])*scale_factor), (int)features[2]*scale_factor, cv::Scalar(0,0,0),CV_FILLED);
    
    else if (features.size() == 5)
    {
        cv::Point2f rec_center = cv::Point2f(features[0]*scale_factor, (map_height - features[1])*scale_factor);
        cv::Size2f rec_size = cv::Point2f(features[2]*scale_factor, features[3]*scale_factor); // check len,wid
        cv::RotatedRect cur_rect = cv::RotatedRect(rec_center, rec_size, -features[4]);
        
        cv::Point2f vertices2f[4];
        cur_rect.points(vertices2f);

        vector<cv::Point> vertices;
        for(int i=0; i<4; ++i) vertices.push_back(vertices2f[i]);
        cv::fillConvexPoly(background, vertices, cv::Scalar(0,0,0));
    }

    else
        cout << "ERROR: Something is wrong. Obst has "<< features.size() << " features" << endl;
}

void viewer::run()
{   
    cout << "Starting viewer .. " << endl;

    // Set up white background with map size
    background = cv::Mat((int)map_height*scale_factor, (int)map_width*scale_factor, CV_8UC3, cv::Scalar(255, 255, 255)); // white bg
    
    vector<Obstacle*> static_obs = map->get_static_obs_vec();
    // Draw static obstacles onto background
    for(Obstacle* cur_obst: static_obs)
        if(cur_obst->isStatic())
            // draw circular static obstacles
            DrawObstacle(cur_obst, background);

    vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();

    while(!isViewerClosed()){
        cv::Mat cur_scene = background.clone();
        for(auto& cur_dyn_obst: dynamic_obs)
            // Draw the dynamic obstacle
            continue;

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
