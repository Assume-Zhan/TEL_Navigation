#ifndef POINTCONTROLLER_H_
#define POINTCONTROLLER_H_

#include "geometry_msgs/Twist.h"

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

#endif /* POINTCONTROLLER_H_ */