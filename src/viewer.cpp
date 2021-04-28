#include "viewer.h"

using namespace std;

viewer::viewer(Map* ptrMap, World* ptrWorld, Robot* ptrRobot)
    : map(ptrMap), closeViewer(false), refresh_rate(25), robot(ptrRobot),
      world(ptrWorld) // refresh at 40Hz
{
    map_width = map->get_x_size();
    map_height = map->get_y_size();
}

void viewer::DrawObstacle(vector<double>& features, cv::Mat& scene){
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

void viewer::DrawRobot(cv::Mat& scene){
    if(robot == NULL) {
        cout << "Robot does not exist. Cannot draw robot." << endl;
        return;
    }
    double x, y;
    robot->getRobotPose(x, y);
    circle(scene, cv::Point2f((x*scale_factor), (map_height - y)*scale_factor), (int)(0.8*scale_factor), cv::Scalar(255,0,0), CV_FILLED);
}

void viewer::DrawGoal(cv::Mat& scene){
    double gx, gy;
    robot->getGoal(gx, gy);
    circle(scene, cv::Point2f((gx*scale_factor), (map_height - gy)*scale_factor), (int)(0.8*scale_factor), cv::Scalar(0,255,0), CV_FILLED);
}

void viewer::DrawTree(const vector<Node*>& tree, cv::Mat& scene){
    // traverse the tree
    queue<Node*> open;
    open.push(tree[0]);
    
    while(!open.empty()){
        Node* cur_node = open.front();
        open.pop();

        for(int n_id: cur_node->neighbors){
            // draw the line
            cv::line(scene, cv::Point2f(cur_node->x*scale_factor, (map_height - cur_node->y)*scale_factor), 
                     cv::Point2f(tree[n_id]->x*scale_factor, (map_height - tree[n_id]->y)*scale_factor),
                     cv::Scalar(100,100,100), 1, 8, 0);

            // add to queue
            open.push(tree[n_id]);
        }
    }
}

void viewer::run()
{   
    cout << "Starting viewer .. " << endl;

    // Set up white background with map size
    background = cv::Mat((int)map_height*scale_factor, (int)map_width*scale_factor, CV_8UC3, cv::Scalar(255, 255, 255)); // white bg
    
    vector<Obstacle*> static_obs = map->get_static_obs_vec();
    // Draw static obstacles onto background
    for(Obstacle* cur_obst: static_obs){
        if(cur_obst->isStatic()){
            vector<double> features = cur_obst->get_obs_feature();
            // draw circular static obstacles
            DrawObstacle(features, background);
        }
    }

    // Draw the goal
    DrawGoal(background);

    vector<DynamObstacle*> dynamic_obs = map->get_dynam_obs_vec();

    while(!isViewerClosed()){
        cv::Mat cur_scene = background.clone();
        
        // Get dynamic obstacles and draw them on canvas
        unsigned long int cur_time = world->get_system_time();
        for(auto& cur_dyn_obst: dynamic_obs){
            // Draw the dynamic obstacle
            vector<double> features = cur_dyn_obst->get_obs_feature(cur_time);
            DrawObstacle(features, cur_scene);
        }

        DrawTree(robot->getCurrentTree(), cur_scene);

        DrawRobot(cur_scene);   // draw the robot

        cv::imshow("Environment display", cur_scene);
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
