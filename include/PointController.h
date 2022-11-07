#ifndef POINTCONTROLLER_H_
#define POINTCONTROLLER_H_

#include <string>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#define USING_OFFSET
#define CAR_CONSTANT
#define PI 3.14159265358979323846

/**
 * @brief
 * Utility -> Vector3
 * For store location, velocity information
 */
typedef struct Vector3{
    double x = 0;
    double y = 0;
    double theta = 0;

    Vector3(){}
    Vector3(double x, double y, double z): x(x), y(y), theta(z){
    }

    Vector3 operator-(Vector3 a) const{
        return Vector3(this->x - a.x, this->y - a.y, this->theta - a.theta);
    }

} Vector3;

// Easy calculation for normalize the error vector
double abs(Vector3 v){ return sqrt(v.x * v.x + v.y * v.y); }

/**
 * @brief
 * PointController
 * Calculating a proper velocity vector for
 * known location and velocity
 */
class PointController{
public:
 // Constructors
    PointController(){}
    PointController(Vector3 goal): GoalPosition(goal){
    }

    // Get goal
    void set_vgoal(Vector3);

    // Check get the goal
    void check_get_goal(Vector3);
    bool getGoal;

    /**
     * @brief Get proper velocity vector by given position and goal,
     * @n Datapath part
     *
     * @param location_vector
     * @param velocity_vector
     * @param time_diff
     * @return geometry_msgs::Twist
     */
    geometry_msgs::Twist get_vgoal(Vector3 location_vector, Vector3 velocity_vector, double time_diff);

private:
 /**
  * @brief
  * Motion state Definition.
  * State information.
  */
    typedef enum{
        STOP = 0,
        ACCEL = 1,
        SLOWDOWN = 2,
        PCONTROL = 3
    } STATE;
    STATE CarState_linear = STOP;
    STATE CarState_angular = STOP;

    /**
     * @brief
     * Circular direction definition.
     * Circular direction information
     */
    typedef enum{
        CW = 1,
        CCW = -1
    } CIRCULAR_DIR;
    CIRCULAR_DIR orientation_dir = CW;

    // Private functions

    /**
     * @brief Get the current state, FSM part
     *
     * @param location
     */
    void get_current_state(Vector3 location);

    void get_breakPoint(Vector3 location);

    void get_pcontrol_point(Vector3 location);

    Vector3 get_error_vector(Vector3 location);

    // Basic variables
    Vector3 GoalPosition;
    Vector3 ErrorVector;
    Vector3 offset;
    double CarLinear_vel;
    double CarAngular_vel;
    double breakpoint_linear = 0;
    double breakpoint_angular = 0;
    double p_control_point = 0;
    double p_angular = 0;

    // Basic constants
    const double P_gain = 0.6;           /* P gain for p controller */
    const double CarSpeed_MAX = 0.7;    /* Max car linear speed */
    const double CarOmega_MAX = 1.2;     /* Max car angular speed */
    const double CarAccel = 0.15;         /* Car linear acceleration */
    const double CarAlpha = 0.5;         /* Car angular acceleration */
    const double CarErrorLinear = 0.01;  /* Min car linear error */
    const double CarErrorAngular = 0.04; /* Min car angular error */

#ifdef CAR_CONSTANT
    const double BP_LINEAR_CONST = 2.;
    const double BP_ANGULAR_CONST = 1.5;
    const double PCONTROL_CONST = 0.75;
#else
    const double BP_LINEAR_CONST = 1.;
    const double BP_ANGULAR_CONST = 1.;
    const double PCONTROL_CONST = 1.;
#endif /* CAR_CONSTANT */
};

#endif /* POINTCONTROLLER_H_ */