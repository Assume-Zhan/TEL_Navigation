#ifndef NAV_MEC_NODE_H_
#define NAV_MEC_NODE_H_

#define DEBUGGER_MODE

#include <ros/ros.h>
#include <nav_mec/navMec_srv.h>
#include "geometry_msgs/Twist.h"

// Define ""server, client, publisher, subscriber""
ros::ServiceServer navMec_ser;
ros::ServiceClient navMec_cli;
ros::Publisher navMec_pub;
ros::Subscriber navMec_sub;

// Callback function for subscriber and server
bool serverCB(nav_mec::navMec_srv::Request&, nav_mec::navMec_srv::Response&);
void subCB(const geometry_msgs::Twist::ConstPtr&);

// --- class PointController ---
class PointController{
public:
    // Constructors
    PointController(){}
    PointController(double x, double y, double z):
        goal_x(x), goal_y(y), goal_theta(z){
    }

    // Get goal
    void set_vgoal(double, double, double);

    // Get cmd_vel
    geometry_msgs::Twist get_vgoal(geometry_msgs::Twist::ConstPtr, double);

    // Reset prev_v
    void Reset_prev_v();

private:
    double goal_x = 0;
    double goal_y = 0;
    double goal_theta = 0;

    double prev_vx = 0;
    double prev_vy = 0;
    double prev_omega = 0;

    const double ax = 0.06;
    const double ay = 0.06;
    const double az = 0.3;

    const double maxSpeed = 0.25;
    const double maxOmega = 1.2;
};

// --- vars ---
bool trigger = false;
PointController pointControl(0, 1, 0);

#endif /* NAV_MEC_NODE_H_ */