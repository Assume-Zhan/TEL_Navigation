#include "PointController.h"

// --- Point Controller start---

void PointController::set_vgoal(Vector3 goal){

    // Bind the theta goal
    while(goal.theta > PI)
        goal.theta = goal.theta - 2 * PI;
    while(goal.theta < -PI)
        goal.theta = goal.theta + 2 * PI;

    this->GoalPosition = goal;

    return;
}

void PointController::check_get_goal(Vector3 location_vector){
    // Calculating error vector and goal sin, cos
    this->ErrorVector.x = this->GoalPosition.x - location_vector.x;
    this->ErrorVector.y = this->GoalPosition.y - location_vector.y;
    this->ErrorVector.theta = this->GoalPosition.theta - location_vector.theta;

    this->get_current_state(location_vector);

    if(abs(this->ErrorVector) > this->CarErrorLinear)
        this->getGoal = false;
    else if(abs(this->ErrorVector) > this->CarErrorAngular)
        this->getGoal = false;
    else this->getGoal = true;
}

geometry_msgs::Twist PointController::get_vgoal(Vector3 location_vector, Vector3 velocity_vector, double time_diff){

    // Calculating error vector and goal sin, cos
    this->ErrorVector.x = this->GoalPosition.x - location_vector.x;
    this->ErrorVector.y = this->GoalPosition.y - location_vector.y;
    this->ErrorVector.theta = this->GoalPosition.theta - location_vector.theta;

    double Gcos, Gsin;
    if(abs(this->ErrorVector) == 0){
        Gcos = Gsin = 0;
    }
    Gcos = this->ErrorVector.x / abs(this->ErrorVector);
    Gsin = this->ErrorVector.y / abs(this->ErrorVector);

    geometry_msgs::Twist cmd_vel;

    this->get_current_state(location_vector);

    double prev_vel = sqrt(pow(velocity_vector.x, 2) + pow(velocity_vector.y, 2));
    if(this->CarState_linear == ACCEL)
        this->CarLinear_vel = prev_vel + (this->CarAccel * time_diff);
    else if(this->CarState_linear == SLOWDOWN)
        this->CarLinear_vel = prev_vel - (this->CarAccel * time_diff);
    else if(this->CarState_linear == PCONTROL)
        this->CarLinear_vel = abs(this->ErrorVector) * this->P_gain;
    else /* STOP state */
        this->CarLinear_vel = 0;

    if(this->CarLinear_vel > this->CarSpeed_MAX)
        this->CarLinear_vel = this->CarSpeed_MAX;

    cmd_vel.linear.x = this->CarLinear_vel * Gcos;
    cmd_vel.linear.y = this->CarLinear_vel * Gsin;

    // TODO : Add angular velocity here
    cmd_vel.angular.z = this->ErrorVector.theta;

    ROS_INFO("State : %d\n", this->CarState_linear);

    return cmd_vel;
}


void PointController::get_current_state(Vector3 location){

    double linear_error = abs(this->ErrorVector);

    // We need to calculate the breakpoint and pcontrol point
    this->get_breakPoint(location);
    this->get_pcontrol_point(location);

    // First check the to STOP state
    if(linear_error < this->CarErrorLinear)
        this->CarState_linear = STOP;
    else{
        switch(this->CarState_linear){
            case STOP:
                // From the STOP state to the ACCEL state when the error is bigger than the breakpoint
                if(linear_error > this->breakpoint_linear)
                    this->CarState_linear = ACCEL;
                break;
            case ACCEL:
                // From the ACCEL state to SLOWDOWN state when the error is smaller than the breakpoint
                if(linear_error < this->breakpoint_linear)
                    this->CarState_linear = SLOWDOWN;
                break;
            case SLOWDOWN:
                // From the SLOWDOWN state to the PCONTROL state
                if(linear_error < this->p_control_point)
                    this->CarState_linear = PCONTROL;
                break;
            case PCONTROL:
                // TODO : we need more thing to take care of pure pursuit
                break;
        }
    }
}

void PointController::get_breakPoint(Vector3 location){

    this->breakpoint_linear = pow(this->CarLinear_vel, 2) / (2 * this->CarAccel);

    // TODO : Add angular breakpoint calculation here
}

void PointController::get_pcontrol_point(Vector3 location){

    this->p_control_point = abs(this->CarLinear_vel) * this->P_gain;

    // TODO : Add angular pcontrol breakpoint calculation here
}

Vector3 PointController::get_error_vector(Vector3 location){

    // Operator overloading 
    // Minus to get error vector
    Vector3 error_vector = this->GoalPosition - location;

    // Change the error vector from world frame to macanum frame
    double sintheta = std::sin(-location.theta);
    double costheta = std::cos(-location.theta);

    return Vector3(error_vector.x * costheta - error_vector.y * sintheta,
                   error_vector.x * sintheta + error_vector.y * costheta,
                   error_vector.theta);
}

// --- PointController end ---