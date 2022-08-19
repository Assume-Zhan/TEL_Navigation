#include "PointController.h"

// --- Point Controller start---

void PointController::set_vgoal(double x, double y, double z){
    this->goal_x = x;
    this->goal_y = y;
    this->goal_theta = z;
    return;
}

geometry_msgs::Twist PointController::get_vgoal(geometry_msgs::Twist::ConstPtr msg, double time_diff){
    geometry_msgs::Twist cmd_vel;

    // Transform goal distance from map frame to odemetry frame
    double pha = msg->angular.z;
    double sinpha = std::sin(pha);
    double cospha = std::cos(pha);

    double delta_x = (this->goal_x - msg->linear.x);
    double delta_y = (this->goal_y - msg->linear.y);
    double delta_theta = (this->goal_theta - msg->angular.z) * 0.6;

    double final_x = cospha * delta_x - sinpha * delta_y;
    double final_y = sinpha * delta_x + cospha * delta_y;

    // Get previous velocity by location_node
    this->prev_vx = msg->angular.x;
    this->prev_vy = msg->angular.y;
    this->prev_omega = msg->linear.z;
    double nowV = this->prev_vx * this->prev_vx + this->prev_vy * this->prev_vy;
    nowV = sqrt(nowV);
    double distance = final_x * final_x + final_y * final_y;
    distance = sqrt(distance);

    // x : y
    double x_all = 0;
    double y_all = 0;
    if(final_x != 0 || final_y != 0)
        x_all = final_x / (abs(final_x) + abs(final_y)),
        y_all = final_y / (abs(final_x) + abs(final_y));

    // Calculate max speed delta
    double V = 0;
    if(distance < this->V_max)
        this->prevCarV = V = distance;
    else
        this->prevCarV = V = std::min(this->prevCarV + this->CarAccel * time_diff, this->V_max);


    cmd_vel.linear.x = V * x_all;
    cmd_vel.linear.y = V * y_all;

    // Speed for orientation
    if(abs(delta_theta) < 1.)
        cmd_vel.angular.z = delta_theta;
    else if(delta_theta > this->prev_omega)
        cmd_vel.angular.z = this->prev_omega + this->az * time_diff;
    else
        cmd_vel.angular.z = this->prev_omega - this->az * time_diff;

    return cmd_vel;
}

double PointController::get_distance(geometry_msgs::Twist::ConstPtr msg){
    double x_error = this->goal_x - msg->linear.x;
    double y_error = this->goal_y - msg->linear.y;

    return sqrt(x_error * x_error + y_error * y_error);
}

double PointController::get_orientationErr(geometry_msgs::Twist::ConstPtr msg){
    double theta_error = this->goal_theta - msg->angular.z;

    return abs(theta_error);
}

void PointController::Reset_prev_v(){
    this->prev_vx = this->prev_vy = this->prev_omega = 0;
    return;
}

bool PointController::check_get_goal(geometry_msgs::Twist::ConstPtr msg){
    if(this->get_distance(msg) > this->xyDeviation)
        return false;
    if(this->get_orientationErr(msg) > this->tDeviation)
        return false;
    return true;
}

// --- PointController end ---