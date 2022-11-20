#include <ros/ros.h>
#include "navMec_ros.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "navMec_node");
    ros::NodeHandle navMec_nh;

    // Get parameter : debug mode
    // Get private parameters
    ros::param::get("~debugmode", debug_mode);
    ros::param::get("~usingoffset", pointControl.using_offset);

    ros::param::get("~offset_x_a", basicconst.offset_const_xa);
    ros::param::get("~offset_x_b", basicconst.offset_const_xb);
    ros::param::get("~offset_y_a", basicconst.offset_const_ya);
    ros::param::get("~offset_y_b", basicconst.offset_const_yb);
    ros::param::get("~offset_z_a", basicconst.offset_const_za);
    ros::param::get("~offset_z_b", basicconst.offset_const_zb);

    ros::param::get("~pgain", basicconst.P_gain);
    ros::param::get("~carSpeedMax", basicconst.CarSpeed_MAX);
    ros::param::get("~carOmegaMax", basicconst.CarOmega_MAX);
    ros::param::get("~carAccel", basicconst.CarAccel);
    ros::param::get("~carAlpha", basicconst.CarAlpha);
    ros::param::get("~carErrorLinear", basicconst.CarErrorLinear);
    ros::param::get("~carErrorAngular", basicconst.CarErrorAngular);

    ros::param::get("~carConst", carconst);

    ros::param::get("~bpLinear", basicconst.BP_LINEAR_CONST);
    ros::param::get("~bpAngular", basicconst.BP_ANGULAR_CONST);
    ros::param::get("~pcontrolconst", basicconst.PCONTROL_CONST);

    ros::param::get("~carAccel_basicMode", basicconst.CarAccel_basicMode);
    ros::param::get("~carAccel_turboMode", basicconst.CarAccel_turboMode);
    ros::param::get("~carSpeedMax_basicMode", basicconst.CarSpeed_MAX_basicMode);
    ros::param::get("~carSpeedMax_turboMode", basicconst.CarSpeed_MAX_turboMode);

    ros::param::get("~calibMode_linear_y", calibMode_linear_y);
    ros::param::get("~calibMode_timeout", timeout);
    timeoutReload = timeout;

    // DEBUGMODE : construct vector
    if(debug_mode) constructVectors();
    pointControl.set_const(carconst, basicconst);

    // Here we act like a """server/client + pub/sub"""
    // Server will first receive data from smach, and trigger the pure pursuit.
    // Then subscriber will subscribe the location info from location_node.
    // In subscriber callback, we will use pure pursuit (if triggered) to \
    // calculate the cmd_vel and publish to STM node.
    // After the error between now and goal location, \
    // client will tell smach that we have done all the thing.

    // Init our server/client, pub/sub
    // DEBUGMODE : Setting server and client
    if(!debug_mode){
        navMec_ser = navMec_nh.advertiseService("navMec_trigger", serverCB);
        navMec_cli = navMec_nh.serviceClient<nav_mec::navMec_fsrv>("navMec_resp");
    }
    navMec_pub = navMec_nh.advertise<geometry_msgs::Twist>("/push_vel", 1);
    navMec_sub = navMec_nh.subscribe("/location_cha", 1, subCB);

    // Go to callback functions
    ros::spin();
}
