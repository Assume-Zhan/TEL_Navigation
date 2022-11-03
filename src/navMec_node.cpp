#include <ros/ros.h>
#include "navMec_ros.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "navMec_node");
    ros::NodeHandle navMec_nh;

    // Get parameter : debug mode
    // Get private parameters
    ros::param::get("~debugmode", debug_mode);

    if(debug_mode) constructVectors();

    // Here we act like a """server/client + pub/sub"""
    // Server will first receive data from smach, and trigger the pure pursuit.
    // Then subscriber will subscribe the location info from location_node.
    // In subscriber callback, we will use pure pursuit (if triggered) to \
    // calculate the cmd_vel and publish to STM node.
    // After the error between now and goal location, \
    // client will tell smach that we have done all the thing.

    // Init our server/client, pub/sub
    if(!debug_mode){
        navMec_ser = navMec_nh.advertiseService("navMec_trigger", serverCB);
        navMec_cli = navMec_nh.serviceClient<nav_mec::navMec_fsrv>("navMec_resp");
    }
    navMec_pub = navMec_nh.advertise<geometry_msgs::Twist>("/push_vel", 1);
    navMec_sub = navMec_nh.subscribe("/location_cha", 1, subCB);

    // Go to callback functions
    ros::spin();
}