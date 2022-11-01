#include "PurePursuit.h"

// --- PurePursuit --- START

PurePursuit::PurePursuit(){
    // Just for debug
    for(int i = 0; i < 100; i++){
        this->path.push_back(Vector3(0. + (double)i / 100., 0., 0.));
    }
    for(int i = 0; i < 100; i++){
        this->path.push_back(Vector3(1., 0. + (double)i / 100., 0.));
    }

    this->pointcontroller = new PointController;
}

void PurePursuit::set_path(std::vector<Vector3> path){
    this->path = path;
}

geometry_msgs::Twist PurePursuit::get_vgoal(Vector3 location, Vector3 velocity, double time_diff){

    geometry_msgs::Twist cmd_vel;
    Vector3 nextPoint = findNextPoint(location);
    this->pointcontroller->set_vgoal(nextPoint);
    this->pointcontroller->check_get_goal(location);
    cmd_vel = this->pointcontroller->get_vgoal(location, velocity, time_diff);

    return cmd_vel;
}

Vector3 PurePursuit::findNextPoint(Vector3 location){
    for(int i = this->nowGoalIdx; i < this->path.size(); i++){
        if(!this->isInCircle(location, this->path[i])){
            nowGoalIdx = i;
            return this->path[nowGoalIdx];
        }
    }
    nowGoalIdx = this->path.size() - 1;
    return this->path[nowGoalIdx];
}

bool PurePursuit::isInCircle(Vector3 now, Vector3 next){
    if(this->get_length(now, next) > this->lookAhead_dis)
        return false;
    return true;
}

double PurePursuit::get_length(Vector3 now, Vector3 next){
    return sqrt(pow(now.x - next.x, 2) + pow(now.y - next.y, 2));
}

// --- PurePursuit --- END