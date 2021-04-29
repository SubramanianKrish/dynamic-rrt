#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include "Obstacle.h"
#include "Map.h"

using namespace std;

int main() {
    Map test_map;
    // get static obstacle vector in the map
    vector<Obstacle> stat_obs = test_map.get_static_obs_vec();
    // get map size
    double x_size = test_map.get_x_size();
    double y_size = test_map.get_y_size();

    cout << "Successfully built a test map with " <<  stat_obs.size() << " obstacles" << endl;
    cout << "The map has a size of (" << x_size << "x" << y_size << ")" << endl;

    // extract the first obstacle
    Obstacle obs1 = stat_obs[0];
    vector<double> feat1 = obs1.get_obs_feature(); 
    // check if obstacle is static
    bool is_static = obs1.isStatic();
    if (is_static){
        cout << "The first obstacle is static"<< endl;
    }else{
        cout << "The first obstacle is not static"<< endl;
    }
    // get position of the obstacle
    double x1 = feat1[0];
    double y1 = feat1[1];
    if (feat1.size()==3){ // circular obstacle ==> 3 features
        double radius = feat1[2];
        cout << "the circular obstacle at position (" << x1 << ", " << y1 << ") has radius " << radius << endl;
    }else{ // rectangular obstacle ==> 5 features
        double length = feat1[2];
        double width = feat1[3];
        double theta = feat1[4]; // for testing static obstacles, theta should be 0
        cout << "the rectangular obstacle at position (" << x1 << ", " << y1 << ") has length and width (" << length << ", " << width << ")" << endl;
    }

    // test dynamics obstacle
    vector<DynamObstacle> dynam_obs_vec = test_map.get_dynam_obs_vec();
    DynamObstacle d0 = dynam_obs_vec[0];
    // inspect position at different timesteps
    double curr_step = 61; // TODO: change this value to inspect dynamics obstacle position output
    vector<double> dynam_feat0 = d0.get_obs_feature(curr_step);
    double dynam_x0 = dynam_feat0[0];
    double dynam_y0 = dynam_feat0[1];
    if (dynam_feat0.size()==3){ // circular obstacle ==> 3 features
        double dynam_radius = dynam_feat0[2];
        cout << "@ Timestep "<< curr_step << ": the dynamics circular obstacle at position (" << dynam_x0 << ", " << dynam_y0 << ") has radius " << dynam_radius << endl;
    }else{ // rectangular obstacle ==> 5 features
        double dynam_length = dynam_feat0[2];
        double dynam_width = dynam_feat0[3];
        double dynam_theta = dynam_feat0[4]; // for testing static obstacles, theta should be 0
        cout << "the rectangular obstacle at position (" << dynam_x0 << ", " << dynam_x0 << ") has length and width (" << dynam_length << ", " << dynam_width << ")" << endl;
    }

    return 0;
}