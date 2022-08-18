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

    // Get distance from now position to goal position
    double get_distance(geometry_msgs::Twist::ConstPtr);
    double get_orientationErr(geometry_msgs::Twist::ConstPtr);

    // Check
    bool check_get_goal(geometry_msgs::Twist::ConstPtr);

private:
    double goal_x = 0;
    double goal_y = 0;
    double goal_theta = 0;

    double prev_vx = 0;
    double prev_vy = 0;
    double prev_omega = 0;

    const double ax = 0.1;
    const double ay = 0.1;
    const double az = 0.4;

    const double maxSpeed = 0.65;
    const double maxOmega = 1.2;

    // Use V control
    double CarV = 0;
    double prevCarV = 0;
    const double CarAccel = 0.2;
    const double V_max = 0.4;

    // DEVIATION
    const double xyDeviation = 0.03;
    const double tDeviation = 0.5;
};

// --- vars ---
bool trigger = false;
PointController pointControl(2, 2, 4);

// --- Need to be removed ---
class Vector3{
public:
    Vector3(){}

    void setxyz(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->theta = z;
    }

    double x, y, theta;
};

int count = 0;
int maxCount = 3;
Vector3 vectors[3];
void constructVectors();

#endif /* NAV_MEC_NODE_H_ */