#ifndef POINTCONTROLLER_H_
#define POINTCONTROLLER_H_

#include <string>
#include <queue>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

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
 * Basic constant
 * For setting the basic constant value in pointcontroller
 */
typedef struct BasicConst{
    double P_gain = 0.6;           /* P gain for p controller */
    double CarSpeed_MAX = 0.7;     /* Max car linear speed */
    double CarOmega_MAX = 1.2;     /* Max car angular speed */
    double CarAccel = 0.15;        /* Car linear acceleration */
    double CarAlpha = 0.5;         /* Car angular acceleration */
    double CarErrorLinear = 0.01;  /* Min car linear error */
    double CarErrorAngular = 0.04; /* Min car angular error */

    double BP_LINEAR_CONST = 2.;
    double BP_ANGULAR_CONST = 1.5;
    double PCONTROL_CONST = 0.75;

    double offset_const_xa = 0.03;
    double offset_const_xb = 0;
    double offset_const_ya = 0;
    double offset_const_yb = 0;
    double offset_const_za = 0;
    double offset_const_zb = 0;

    double CarAccel_basicMode = 0.4;
    double CarAccel_turboMode = 2.0;

    double CarSpeed_MAX_basicMode = 0.7;
    double CarSpeed_MAX_turboMode = 2.0;
} BasicConst;

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

    // DEBUGMODE : Some debug value 
    bool using_offset = true;
    void set_const(bool, BasicConst);

    // Get goal
    void set_vgoal(std::queue<Vector3>);

    // Check get the goal
    void check_get_goal(Vector3);
    Vector3 get_V_goal();
    bool getGoal;

    // Set different mode
    void modeSettings(std::queue<char> mode);
    char getMode();
    bool calibMode_clearBuffer();

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

    void get_error_vector(Vector3 location);

    // Basic variables
    bool GoalChanged = false;
    std::queue<Vector3> GoalBuffer;
    std::queue<char> ModeBuffer;
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
    double P_gain = 0.6;           /* P gain for p controller */
    double CarSpeed_MAX = 0.7;     /* Max car linear speed */
    double CarOmega_MAX = 1.2;     /* Max car angular speed */
    double CarAccel = 0.15;        /* Car linear acceleration */
    double CarAlpha = 0.5;         /* Car angular acceleration */
    double CarErrorLinear = 0.01;  /* Min car linear error */
    double CarErrorAngular = 0.04; /* Min car angular error */

    double BP_LINEAR_CONST = 2.;
    double BP_ANGULAR_CONST = 1.5;
    double PCONTROL_CONST = 0.75;

    double offset_const_xa = 0.03;
    double offset_const_xb = 0;
    double offset_const_ya = 0;
    double offset_const_yb = 0;
    double offset_const_za = 0;
    double offset_const_zb = 0;

    // Mode settings
    double CarAccel_basicMode = 0.4;
    double CarAccel_turboMode = 2.0;

    double CarSpeed_MAX_basicMode = 0.7;
    double CarSpeed_MAX_turboMode = 2.0;
};

#endif /* POINTCONTROLLER_H_ */
