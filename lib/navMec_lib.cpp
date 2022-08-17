#include <cmath>

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

    // Transform goal position from map frame to odemetry frame
    double pha = msg->angular.z;
    double sinpha = std::sin(pha);
    double cospha = std::cos(pha);
    double goalX = cospha * this->goal_x - sinpha * this->goal_y;
    double goalY = sinpha * this->goal_x + cospha * this->goal_y;

    double delta_x = goalX - msg->linear.x;
    double delta_y = goalY - msg->linear.y;
    double delta_theta = this->goal_theta - msg->angular.z;

    // v_next < a * time_diff + v_prev
    double maxVx = std::min(this->maxSpeed, this->ax * time_diff + this->prev_vx);
    double maxVy = std::min(this->maxSpeed, this->ay * time_diff + this->prev_vy);
    double maxVw = std::min(this->maxOmega, this->az * time_diff + this->prev_omega);

    // Bind delta_v
    // Record v
    this->prev_vx = cmd_vel.linear.x = std::max(-maxVx, std::min(maxVx, delta_x));
    this->prev_vy = cmd_vel.linear.y = std::max(-maxVy, std::min(maxVy, delta_y));
    this->prev_omega = cmd_vel.angular.z = std::max(-maxVw, std::min(maxVw, delta_theta));

    return cmd_vel;
}

void PointController::Reset_prev_v(){
    this->prev_vx = this->prev_vy = this->prev_omega = 0;
    return;
}