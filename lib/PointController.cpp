#include "PointController.h"

// --- Point Controller start---

void PointController::set_vgoal(std::queue<Vector3> goalBuffer){

    // Bind the theta goal


    this->GoalBuffer = goalBuffer;

    while(this->GoalBuffer.front().theta > PI)
        this->GoalBuffer.front().theta = this->GoalBuffer.front().theta - 2 * PI;
    while(this->GoalBuffer.front().theta < -PI)
        this->GoalBuffer.front().theta = this->GoalBuffer.front().theta + 2 * PI;

    this->GoalPosition = this->GoalBuffer.front();
    this->GoalChanged = true;

    return;
}


Vector3 PointController::get_V_goal(){
    return this->GoalPosition;
}

void PointController::set_const(bool carconst, BasicConst basicconst){

    this->P_gain = basicconst.P_gain;
    this->CarSpeed_MAX = basicconst.CarSpeed_MAX;
    this->CarOmega_MAX = basicconst.CarOmega_MAX;
    this->CarAccel = basicconst.CarAccel;
    this->CarAlpha = basicconst.CarAlpha;
    this->CarErrorLinear = basicconst.CarErrorLinear;
    this->CarErrorAngular = basicconst.CarErrorAngular;

    this->CarAccel_basicMode = basicconst.CarAccel_basicMode;
    this->CarAccel_turboMode = basicconst.CarAccel_turboMode;
    this->CarSpeed_MAX_basicMode = basicconst.CarSpeed_MAX_basicMode;
    this->CarSpeed_MAX_turboMode = basicconst.CarSpeed_MAX_turboMode;

    if(this->using_offset){
        this->offset_const_xa = basicconst.offset_const_xa;
        this->offset_const_xb = basicconst.offset_const_xb;
        this->offset_const_ya = basicconst.offset_const_ya;
        this->offset_const_yb = basicconst.offset_const_yb;
        this->offset_const_za = basicconst.offset_const_za;
        this->offset_const_zb = basicconst.offset_const_zb;
    }
    else{

        this->offset_const_xa = 0;
        this->offset_const_xb = 0;
        this->offset_const_ya = 0;
        this->offset_const_yb = 0;
        this->offset_const_za = 0;
        this->offset_const_zb = 0;
    }

    if(carconst){
        this->BP_LINEAR_CONST = basicconst.BP_LINEAR_CONST;
        this->BP_ANGULAR_CONST = basicconst.BP_ANGULAR_CONST;
        this->PCONTROL_CONST = basicconst.PCONTROL_CONST;
    }
    else{
        this->BP_LINEAR_CONST = 1;
        this->BP_ANGULAR_CONST = 1;
        this->PCONTROL_CONST = 1;
    }

}

void PointController::modeSettings(std::queue<char> mode){
    this->ModeBuffer = mode;
}

char PointController::getMode(){
    if(this->ModeBuffer.front() == 't'){
        this->CarAccel = this->CarAccel_turboMode;
        this->CarSpeed_MAX = this->CarAccel_turboMode;
    }
    else{
        this->CarAccel = this->CarAccel_basicMode;
        this->CarSpeed_MAX = this->CarAccel_basicMode;
    }

    return this->ModeBuffer.front();
}

bool PointController::calibMode_clearBuffer(){
    this->GoalBuffer.pop();
    this->ModeBuffer.pop();

    if(this->ModeBuffer.empty() || this->GoalBuffer.empty())
        return true;
    else{
        while(this->GoalBuffer.front().theta > PI)
            this->GoalBuffer.front().theta = this->GoalBuffer.front().theta - 2 * PI;
        while(this->GoalBuffer.front().theta < -PI)
            this->GoalBuffer.front().theta = this->GoalBuffer.front().theta + 2 * PI;

        this->GoalPosition = this->GoalBuffer.front();
        this->GoalChanged = true;
        this->getGoal = false;

        return false;
    }
}

void PointController::check_get_goal(Vector3 location_vector){

    // Calculating error vector and goal sin, cos
    this->get_error_vector(location_vector);

    // FSM part
    this->get_current_state(location_vector);

    if(abs(this->ErrorVector) >= this->CarErrorLinear)
        this->getGoal = false;
    else if(abs(this->ErrorVector.theta) >= this->CarErrorAngular)
        this->getGoal = false;
    else{
        // ROS_DEBUG_STREAM("CURRENT_GOAL : " << this->GoalPosition.x << ", " << this->GoalPosition.y << ", " << this->GoalPosition.theta);
        this->GoalBuffer.pop();
        this->ModeBuffer.pop();
        if(this->GoalBuffer.empty()){
            ROS_DEBUG_STREAM("CURRENT_GOAL : " << this->GoalPosition.x << ", " << this->GoalPosition.y << ", " << this->GoalPosition.theta);
            this->getGoal = true;
        }
        else{
            while(this->GoalBuffer.front().theta > PI)
                this->GoalBuffer.front().theta = this->GoalBuffer.front().theta - 2 * PI;
            while(this->GoalBuffer.front().theta < -PI)
                this->GoalBuffer.front().theta = this->GoalBuffer.front().theta + 2 * PI;

            this->GoalPosition = this->GoalBuffer.front();
            this->GoalChanged = true;
            this->getGoal = false;
        }
    }
}

geometry_msgs::Twist PointController::get_vgoal(Vector3 location_vector, Vector3 velocity_vector, double time_diff){

    geometry_msgs::Twist cmd_vel;
    ROS_INFO_STREAM("CURRENT loc : " << location_vector.x << ", " << location_vector.y << ", " << location_vector.theta);
    ROS_INFO_STREAM("CURRENT STATE : " << this->CarState_linear);

    // Calculate the offset
    this->offset.x = this->offset_const_xa * location_vector.x + this->offset_const_xb;
    this->offset.y = this->offset_const_ya * location_vector.y + this->offset_const_yb;
    this->offset.theta = this->offset_const_za * location_vector.theta + this->offset_const_zb;

    // Calculating error vector and goal sin, cos
    this->get_error_vector(location_vector);

    // FSM part
    this->get_current_state(location_vector);

    double Gcos, Gsin;
    if(abs(this->ErrorVector) == 0){
        Gcos = Gsin = 0;
    }
    Gcos = this->ErrorVector.x / abs(this->ErrorVector);
    Gsin = this->ErrorVector.y / abs(this->ErrorVector);

    double prev_vel = abs(velocity_vector);
    switch(this->CarState_linear){
        case ACCEL:
            this->CarLinear_vel = prev_vel + (this->CarAccel * time_diff);
            break;
        case SLOWDOWN:
            this->CarLinear_vel = prev_vel - (this->CarAccel * time_diff);
            break;
        case PCONTROL:
            this->CarLinear_vel = abs(this->ErrorVector) * this->P_gain;
            if(this->CarLinear_vel > prev_vel)
                this->CarLinear_vel = prev_vel;
            break;
        case STOP:
            this->CarLinear_vel = 0;
            break;
        default:
            this->CarLinear_vel = 0;
            break;
    }

    if(this->CarLinear_vel > this->CarSpeed_MAX)
        this->CarLinear_vel = this->CarSpeed_MAX;

    // Get absolute angular velocity
    double prev_omega = abs(velocity_vector.theta);
    switch(this->CarState_angular){
        case ACCEL:
            this->CarAngular_vel = prev_omega + (this->CarAlpha * time_diff);
            break;
        case SLOWDOWN:
            this->CarAngular_vel = prev_omega - (this->CarAlpha * time_diff);
            break;
        case PCONTROL:
            this->CarAngular_vel = abs(this->ErrorVector.theta) * this->P_gain;
            if(this->CarAngular_vel > prev_omega)
                this->CarAngular_vel = prev_omega;
            break;
        case STOP:
            this->CarAngular_vel = 0;
            break;
        default:
            this->CarAngular_vel = 0;
            break;
    }

    if(this->CarAngular_vel > this->CarOmega_MAX)
        this->CarAngular_vel = this->CarOmega_MAX;

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

    cmd_vel.linear.x = this->CarLinear_vel * Gcos;
    cmd_vel.linear.y = this->CarLinear_vel * Gsin;
    cmd_vel.angular.z = this->CarAngular_vel * this->orientation_dir;

    // ROS_INFO("State : %d, ang_bp : %lf, ang_err : %lf\n", this->CarState_angular,
    //                                                       this->breakpoint_angular,
    //                                                       abs(this->ErrorVector.theta));
    // ROS_INFO("State : %d, prev : %lf, Car ang : %lf\n", this->CarState_angular, prev_omega, this->CarAngular_vel);
    // ROS_INFO("State : %d\n", this->CarState_linear);

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
                if(linear_error < this->breakpoint_linear * this->BP_LINEAR_CONST && this->CarLinear_vel >= 0.15)
                    this->CarState_linear = SLOWDOWN;
                break;
            case SLOWDOWN:
                // From the SLOWDOWN state to the PCONTROL state
                if(abs(this->p_control_point - linear_error * this->P_gain) < this->CarAccel * this->PCONTROL_CONST)
                    this->CarState_linear = PCONTROL;

                if(this->GoalChanged) this->CarState_linear = STOP;
                break;
            case PCONTROL:
                // TODO : we need more thing to take care of pure pursuit
                if(this->GoalChanged) this->CarState_linear = STOP;
                break;
        }
    }

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
                if(this->GoalChanged) this->CarState_angular = STOP;
                break;
            case PCONTROL:
                // TODO : we need more thing to take care of pure pursuit
                if(this->GoalChanged) this->CarState_angular = STOP;
                break;
        }
    }

    this->GoalChanged = false;
}

void PointController::get_breakPoint(Vector3 location){

    this->breakpoint_linear = pow(this->CarLinear_vel, 2) / (2 * this->CarAccel);

    this->breakpoint_angular = pow(this->CarAngular_vel, 2) / (2 * this->CarAlpha);
}

void PointController::get_pcontrol_point(Vector3 location){

    this->p_control_point = abs(this->CarLinear_vel) * this->P_gain;

    this->p_angular = abs(this->CarAngular_vel) * this->P_gain;
}

void PointController::get_error_vector(Vector3 location){

    // Operator overloading 
    // Minus to get error vector
    Vector3 error_vector = this->GoalPosition - location - this->offset;

    // Change the error vector from world frame to macanum frame
    double sintheta = std::sin(-location.theta);
    double costheta = std::cos(-location.theta);

    // Bound the region of error vector
    if(error_vector.theta > PI) error_vector.theta -= 2 * PI;
    else if(error_vector.theta < -PI) error_vector.theta += 2 * PI;

    this->ErrorVector = Vector3(error_vector.x * costheta - error_vector.y * sintheta,
                                error_vector.x * sintheta + error_vector.y * costheta,
                                error_vector.theta);
}

// --- PointController end ---