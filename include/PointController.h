#ifndef POINTCONTROLLER_H_
#define POINTCONTROLLER_H_

#include "geometry_msgs/Twist.h"
#include <string>
#define PI 3.14159265358979323846

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

    // Check
    bool check_get_goal(geometry_msgs::Twist::ConstPtr);
    bool getGoal = false;

    // Renew
    void RenewInfo();

private:
    // State Definition
    typedef enum{
        ACCEL = 1,
        STOP = 0,
        SLOWDOWN = -1
    } STATE;

    // Get distance from now position to goal position
    double get_linearErr(geometry_msgs::Twist::ConstPtr);
    double get_orientationErr(geometry_msgs::Twist::ConstPtr);
    bool GotLinearErr = false;
    bool GotAngularErr = false;

    // State information
    STATE CarState_linear = STOP;
    STATE CarState_angular = STOP;
    void get_CarState();

    // Goal information
    double goal_x = 0;
    double goal_y = 0;
    double goal_theta = 0;
    bool changeGoal = false;

    // Error odemetry frame
    double CarError_linearX = 0;
    double CarError_linearY = 0;
    double CarError_linear = 0;
    double CarError_angular = 0;

    // Calculate break point
    // true : linear
    // false : angular
    double breakPoint(bool);

    // Omega direction
    int CarDir_angular = 1;
    void get_orientationDir(double);

    // Velocity info and theta info
    double CarVel_linear = 0;
    double CarVel_angular = 0;
    double sinpha = 0;
    double cospha = 0;

    // About linear vector information
    double Gsin = 0;
    double Gcos = 0;
    void Gpha();

    // P controller
    const double P = 0.8;

    // Velocity restriction
    const double maxSpeed = 0.5;
    const double maxOmega = 1.2;

    // Use V control
    const double CarAccel = 0.1; /* Linear acceloration */
    const double CarAlpha = 0.2; /* Angular acceloration */

    // DEVIATION
    const double xyDeviation = 0.01;
    const double tDeviation = 0.1;
};

#endif /* POINTCONTROLLER_H_ */