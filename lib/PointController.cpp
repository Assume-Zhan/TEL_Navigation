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
    this->ErrorVector = this->get_error_vector(location_vector);

    if(this->ErrorVector.theta > PI) this->ErrorVector.theta -= 2 * PI;
    else if(this->ErrorVector.theta < -PI) this->ErrorVector.theta += 2 * PI;

    this->get_current_state(location_vector);

    if(abs(this->ErrorVector) >= this->CarErrorLinear)
        this->getGoal = false;
    else if(abs(this->ErrorVector.theta) >= this->CarErrorAngular)
        this->getGoal = false;
    else this->getGoal = true;
}

geometry_msgs::Twist PointController::get_vgoal(Vector3 location_vector, Vector3 velocity_vector, double time_diff){

#ifdef USING_OFFSET
    ///////
    this->offset.x = 0.03 * location_vector.x;
    this->offset.y = 0;
    ///////
#endif /* USING_OFFSET */

    // Calculating error vector and goal sin, cos
    this->ErrorVector = this->get_error_vector(location_vector);

    if(this->ErrorVector.theta > PI) this->ErrorVector.theta -= 2 * PI;
    else if(this->ErrorVector.theta < -PI) this->ErrorVector.theta += 2 * PI;

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

    // Get absolute angular velocity
    double prev_omega = abs(velocity_vector.theta);
    if(this->CarState_angular == ACCEL)
        this->CarAngular_vel = prev_omega + (this->CarAlpha * time_diff);
    else if(this->CarState_angular == SLOWDOWN)
        this->CarAngular_vel = prev_omega - (this->CarAlpha * time_diff);
    else if(this->CarState_angular == PCONTROL)
        this->CarAngular_vel = abs(this->ErrorVector.theta) * this->P_gain;
    else /* STOP state */
        this->CarAngular_vel = 0;

    if(this->CarAngular_vel > this->CarOmega_MAX)
        this->CarAngular_vel = this->CarOmega_MAX;

    // TODO : Get direction of angular
    if(abs(this->ErrorVector.theta) < PI){
        if(this->ErrorVector.theta > 0)
            this->orientation_dir = CW;
        else
            this->orientation_dir = CCW;
    }
    else{
        if(this->ErrorVector.theta < 0)
            this->orientation_dir = CW;
        else
            this->orientation_dir = CCW;
    }

    cmd_vel.angular.z = this->CarAngular_vel * this->orientation_dir;

    // ROS_INFO("State : %d, ang_bp : %lf, ang_err : %lf\n", this->CarState_angular,
    //                                                       this->breakpoint_angular,
    //                                                       abs(this->ErrorVector.theta));
    // ROS_INFO("State : %d, prev : %lf, Car ang : %lf\n", this->CarState_angular, prev_omega, this->CarAngular_vel);
    ROS_INFO("State : %d\n", this->CarState_linear);

    return cmd_vel;
}


void PointController::get_current_state(Vector3 location){

    double linear_error = abs(this->ErrorVector);
    double angular_error = abs(this->ErrorVector.theta);

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
                if(linear_error < this->breakpoint_linear * this->BP_LINEAR_CONST)
                    this->CarState_linear = SLOWDOWN;
                break;
            case SLOWDOWN:
                // From the SLOWDOWN state to the PCONTROL state
                if(abs(this->p_control_point - abs(this->ErrorVector) * this->P_gain) < this->CarAccel * this->PCONTROL_CONST)
                    this->CarState_linear = PCONTROL;
                break;
            case PCONTROL:
                // TODO : we need more thing to take care of pure pursuit
                break;
        }
    }

    // TODO : Angular state
    if(angular_error < this->CarErrorAngular)
        this->CarState_angular = STOP;
    else{
        switch(this->CarState_angular){
            case STOP:
                // From the STOP state to the ACCEL state when the error is bigger than the breakpoint
                if(angular_error > this->breakpoint_angular)
                    this->CarState_angular = ACCEL;
                break;
            case ACCEL:
                // From the ACCEL state to SLOWDOWN state when the error is smaller than the breakpoint
                if(angular_error < this->breakpoint_angular * this->BP_ANGULAR_CONST)
                    this->CarState_angular = SLOWDOWN;
                break;
            case SLOWDOWN:
                // From the SLOWDOWN state to the PCONTROL state
                if(abs(this->p_angular - abs(this->ErrorVector.theta) * this->P_gain) < this->CarAlpha * this->PCONTROL_CONST)
                    this->CarState_angular = PCONTROL;
                break;
            case PCONTROL:
                // TODO : we need more thing to take care of pure pursuit
                break;
        }
    }
}

void PointController::get_breakPoint(Vector3 location){

    this->breakpoint_linear = pow(this->CarLinear_vel, 2) / (2 * this->CarAccel);

    this->breakpoint_angular = pow(this->CarAngular_vel, 2) / (2 * this->CarAlpha);
}

void PointController::get_pcontrol_point(Vector3 location){

    this->p_control_point = abs(this->CarLinear_vel) * this->P_gain;

    this->p_angular = abs(this->CarAngular_vel) * this->P_gain;
}

Vector3 PointController::get_error_vector(Vector3 location){

    // Operator overloading 
    // Minus to get error vector
    Vector3 error_vector = this->GoalPosition - location - this->offset;

    // Change the error vector from world frame to macanum frame
    double sintheta = std::sin(-location.theta);
    double costheta = std::cos(-location.theta);

    return Vector3(error_vector.x * costheta - error_vector.y * sintheta,
                   error_vector.x * sintheta + error_vector.y * costheta,
                   error_vector.theta);
}

// --- PointController end ---