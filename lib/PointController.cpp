#include "PointController.h"

// --- Point Controller start---

void PointController::set_vgoal(double x, double y, double z){
    this->goal_x = x;
    this->goal_y = y;

    // Bind the theta goal
    while(z > PI)
        z = z - 2 * PI;
    while(z < -PI)
        z = z + 2 * PI;

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

    double final_x = cospha * delta_x - sinpha * delta_y;
    double final_y = sinpha * delta_x + cospha * delta_y;

    // Get previous velocity by location_node
    this->prev_vx = msg->angular.x;
    this->prev_vy = msg->angular.y;
    double distance = final_x * final_x + final_y * final_y;
    distance = std::sqrt(distance);
    double nowV = this->prev_vx * this->prev_vx + this->prev_vy * this->prev_vy;
    nowV = std::sqrt(nowV);


    // x : y
    double x_all = 0;
    double y_all = 0;
    if(final_x != 0 || final_y != 0)
        x_all = final_x / std::sqrt(abs(final_x) * abs(final_x) + abs(final_y) * abs(final_y)),
        y_all = final_y / std::sqrt(abs(final_x) * abs(final_x) + abs(final_y) * abs(final_y));

    // Calculate max speed delta
    double V = 0;
    if(distance < this->dis_decrease(nowV))
        V = std::min(nowV - this->CarAccel * time_diff, this->V_max);
    else
        V = std::min(nowV + this->CarAccel * time_diff, this->V_max);

    cmd_vel.linear.x = V * x_all;
    cmd_vel.linear.y = V * y_all;


    // Speed for orientation
    double thetaDis = (this->goal_theta - msg->angular.z);
    this->prev_omega = msg->linear.z;
    if(thetaDis > PI)
        thetaDis = thetaDis - 2 * PI;
    else if(thetaDis < -PI)
        thetaDis = thetaDis + 2 * PI;

    double V_omega = 0;

    if(abs(thetaDis) < this->tDeviation)
        V_omega = 0;
    else if(abs(thetaDis) < this->theta_decrease(this->prev_omega)){
        // Start to decrease our velocity
        if(thetaDis > 0)
            V_omega = std::max(-this->maxOmega, std::min(this->maxOmega, this->prev_omega - time_diff * this->az));
        else
            V_omega = std::max(-this->maxOmega, std::min(this->maxOmega, this->prev_omega + time_diff * this->az));

    }
    else{
        if(thetaDis > 0)
            V_omega = std::max(-this->maxOmega, std::min(this->maxOmega, this->prev_omega + time_diff * this->az));
        else
            V_omega = std::max(-this->maxOmega, std::min(this->maxOmega, this->prev_omega - time_diff * this->az));
    }

    cmd_vel.angular.z = V_omega;

    return cmd_vel;
}

double PointController::dis_decrease(double nowSpeed){
    // V = V0 + at
    // 0 = nowSpeed + this->V_a * t
    double t = abs(nowSpeed / this->CarAccel);
    return (nowSpeed * t) / 2;
}

double PointController::theta_decrease(double nowOmega){
    double t = abs(nowOmega / this->az);
    return abs((nowOmega * t) / 2);
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