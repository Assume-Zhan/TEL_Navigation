#ifndef NAV_MEC_NODE_H_
#define NAV_MEC_NODE_H_

#define DEBUGGER_MODE

#include <ros/ros.h>
#include <nav_mec/navMec_srv.h>
#include <nav_mec/navMec_fsrv.h>
#include "geometry_msgs/Twist.h"
#include "PointController.h"
#include "PurePursuit.h"

// Define ""server, client, publisher, subscriber""
ros::ServiceServer navMec_ser;
ros::ServiceClient navMec_cli;
ros::Publisher navMec_pub;
ros::Subscriber navMec_sub;

// Callback function for subscriber and server
bool serverCB(nav_mec::navMec_srv::Request&, nav_mec::navMec_srv::Response&);
void subCB(const geometry_msgs::Twist::ConstPtr&);
void subCBPP(const geometry_msgs::Twist::ConstPtr&);

// --- vars ---
bool trigger = false;
bool firstTrigger = true;
PointController pointControl(Vector3(0., 0, 0));
PurePursuit purepursuit;



// --- Need to be removed --- S
class vector3{
public:
    vector3(){}

    void setxyz(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->theta = z;
    }

    double x, y, theta;
};

int count = 0;
int maxCount = 3;
vector3 vectors[13];
void constructVectors();
// --- Need to be removed --- E

#endif /* NAV_MEC_NODE_H_ */