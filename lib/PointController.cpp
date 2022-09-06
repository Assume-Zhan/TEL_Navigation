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

    /* Linear velocity calculation */
    // Get error information
    if(!this->GotLinearErr)
        this->get_linearErr(msg);

    // Get goal-theta information
    this->Gpha();

    // Velocity control (Calculate proper velocity for mecanum)
    // Calculate linear velocity
    double prev_vel = sqrt(pow(msg->angular.x, 2) + pow(msg->angular.y, 2));
    this->CarVel_linear = prev_vel + (this->CarAccel * time_diff);
    if(this->CarError_linear < this->breakPoint(true)){
        this->CarVel_linear = this->CarError_linear * this->P;
    }
    if(this->CarVel_linear > this->maxSpeed)
        this->CarVel_linear = this->maxSpeed;

    // Calculate Vx and Vy
    cmd_vel.linear.x = this->CarVel_linear * this->Gcos;
    cmd_vel.linear.y = this->CarVel_linear * this->Gsin;


    /* Angular velocity calculation ( theta domain : [-pi, pi] ) */
    // Get Theta Error
    if(!this->GotAngularErr)
        this->get_orientationErr(msg);
    this->CarVel_angular = abs(msg->linear.z) + (this->CarAlpha * time_diff);
    if(this->CarError_angular < this->breakPoint(false)){
        this->CarVel_angular = this->CarError_angular * this->P;
    }
    if(this->CarVel_angular > this->maxOmega)
        this->CarVel_angular = this->maxOmega;

    cmd_vel.angular.z = this->CarDir_angular * this->CarVel_angular;

    return cmd_vel;
}

double PointController::breakPoint(bool type){
    /* type true : linear
       type false : angular */
    if(type){
        /* Linear mode */
        return ((pow(this->CarVel_linear, 2)) / (2 * this->CarAccel));
    }
    else{
        /* Angular mode */
        return (pow(this->CarVel_angular, 2) / (2 * this->CarAlpha));
    }
}

bool PointController::check_get_goal(geometry_msgs::Twist::ConstPtr msg){
    if(this->get_linearErr(msg) > this->xyDeviation)
        return this->getGoal = false;
    if(abs(this->get_orientationErr(msg)) > this->tDeviation)
        return this->getGoal = false;
    return this->getGoal = true;
}

double PointController::get_linearErr(geometry_msgs::Twist::ConstPtr msg){
    /* x-y distance error
    Transfrom error from map frame to odemetry frame */

    // Transform goal distance from map frame to odemetry frame
    // Calculate linear error
    this->sinpha = std::sin(msg->angular.z);
    this->cospha = std::cos(msg->angular.z);
    this->CarError_linearX = this->cospha * (this->goal_x - msg->linear.x) - this->sinpha * (this->goal_y - msg->linear.y);
    this->CarError_linearY = this->sinpha * (this->goal_x - msg->linear.x) + this->cospha * (this->goal_y - msg->linear.y);

    this->GotLinearErr = true;

    return this->CarError_linear = sqrt(pow(this->CarError_linearX, 2) + pow(this->CarError_linearY, 2));
}

double PointController::get_orientationErr(geometry_msgs::Twist::ConstPtr msg){
    double theta_error = this->goal_theta - msg->angular.z;

    // Calculate proper direction
    this->get_orientationDir(theta_error);

    this->GotAngularErr = true;

    return this->CarError_angular = abs(theta_error);
}

void PointController::get_orientationDir(double thetaErr){
    if(thetaErr == 0)
        return;

    double absThetaErr = abs(thetaErr);
    if(absThetaErr < PI)
        this->CarDir_angular = (thetaErr / absThetaErr);
    else
        this->CarDir_angular = -(thetaErr / absThetaErr);
}

void PointController::Gpha(){
    /* Calculate theta about goal vector
    Need after calculate linear error */
    if(this->CarError_linear == 0){
        this->Gcos = this->Gsin = 0;
        return;
    }

    this->Gcos = this->CarError_linearX / this->CarError_linear;
    this->Gsin = this->CarError_linearY / this->CarError_linear;
}

void PointController::RenewInfo(){
    this->GotLinearErr = this->GotAngularErr = false;
}

// --- PointController end ---