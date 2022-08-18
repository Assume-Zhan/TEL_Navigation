#include <cmath>
#include <iostream>

#include "navMec_node.h"

// --- Implement callback functions ---
bool serverCB(nav_mec::navMec_srv::Request& req, nav_mec::navMec_srv::Response& res){
    return true;
}

void subCB(const geometry_msgs::Twist::ConstPtr& msg){
    // Subscribe location info from location node

    geometry_msgs::Twist cmd_vel;

#ifndef DEBUGGER_MODE
    if(!trigger){
        // Haven't trigger the navigation
        // Don't move the chasis ---> give 0 speed ---> return
        navMec_pub.publish(cmd_vel);
        // pointControl.set_vgoal(1, 0, 0);
        return;
    }
#endif /* DEBUGGER_MODE */

    // If trigger --> start to nagigation
    // Problem : How to get path ?
    static double time_before = 0;
    static double time_after = 0;
    if(!time_before){
        time_before = ros::Time::now().toSec();
        navMec_pub.publish(cmd_vel);
        return;
    }

    time_after = ros::Time::now().toSec();
    double time_diff = time_after - time_before;
    time_before = ros::Time::now().toSec();

    cmd_vel = pointControl.get_vgoal(msg, time_diff);
    navMec_pub.publish(cmd_vel);


#ifndef DEBUGGER_MODE
    // If we got the goal --> send success info
    if(/* error < DEVIATION */1){
        trigger = false;
        nav_mec::navMec_srv res_srv;
        res_srv.request.start = true;
        navMec_cli.call(res_srv);
        // Ignore the response

        time_before = time_after = 0;
        pointControl.Reset_prev_v();
    }
#endif /* DEBUGGER_MODE */
}

// --- Point Controller ---

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

    double final_x = cospha * delta_x - sinpha * delta_y; final_x *= 0.45;
    double final_y = sinpha * delta_x + cospha * delta_y; final_y *= 0.45;

    // Get previous velocity by location_node
    this->prev_vx = msg->angular.x;
    this->prev_vy = msg->angular.y;
    this->prev_omega = msg->linear.z;

    // x : y
    double x_all = abs(delta_x) / (abs(delta_x) + abs(delta_y));
    double y_all = abs(delta_y) / (abs(delta_x) + abs(delta_y));

    // Calculate max speed delta
    double min_delta_vx;
    if(final_x > this->prev_vx)
        min_delta_vx = std::min(this->ax * time_diff, abs(final_x - this->prev_vx));
    else
        min_delta_vx = 0 - std::min(this->ax * time_diff, abs(final_x - this->prev_vx));

    cmd_vel.linear.x = std::max(-this->maxSpeed, std::min(this->maxSpeed, this->prev_vx + min_delta_vx));

    double min_delta_vy;
    if(final_y > prev_vy)
        min_delta_vy = std::min(this->ay * time_diff, abs(final_y - this->prev_vy));
    else
        min_delta_vy = 0 - std::min(this->ay * time_diff, abs(final_y - this->prev_vy));

    cmd_vel.linear.y = std::max(-this->maxSpeed, std::min(this->maxSpeed, this->prev_vy + min_delta_vy));

    double min_delta_omega;
    if(delta_theta > prev_omega)
        min_delta_omega = std::min(this->az * time_diff, abs(delta_theta - prev_omega));
    else
        min_delta_omega = 0 - std::min(this->az * time_diff, abs(delta_theta - prev_omega));

    cmd_vel.angular.z = std::max(-this->maxOmega, std::min(this->maxOmega, this->prev_omega + min_delta_omega));

    std::cout << final_x << ' ' << this->prev_vx << ' ' << this->ax * time_diff << '\n';

    return cmd_vel;
}

void PointController::Reset_prev_v(){
    this->prev_vx = this->prev_vy = this->prev_omega = 0;
    return;
}