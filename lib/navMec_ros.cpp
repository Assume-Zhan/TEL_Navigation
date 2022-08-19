#include <cmath>
#include <iostream>

#include "navMec_ros.h"

// --- Implement callback functions ---
/* Server Callback function */
bool serverCB(nav_mec::navMec_srv::Request& req, nav_mec::navMec_srv::Response& res){
    return true;
}

/* Subscriber Callback function */
void subCB(const geometry_msgs::Twist::ConstPtr& msg){
    // Subscribe location info from location node

    geometry_msgs::Twist cmd_vel;

#ifndef DEBUGGER_MODE
    if(!trigger){
        // Haven't trigger the navigation
        // Don't move the chasis ---> give 0 speed ---> return
        navMec_pub.publish(cmd_vel);
        // pointControl.set_vgoal(1, 0, 0);
        return;
    }
#endif /* DEBUGGER_MODE */

    // If trigger --> start to nagigation
    // Problem : How to get path ?
    static double time_before = 0;
    static double time_after = 0;
    if(!time_before){
        time_before = ros::Time::now().toSec();
        navMec_pub.publish(cmd_vel);
        return;
    }

    time_after = ros::Time::now().toSec();
    double time_diff = time_after - time_before;
    time_before = ros::Time::now().toSec();

    if(!pointControl.check_get_goal(msg)){
        cmd_vel = pointControl.get_vgoal(msg, time_diff);
        navMec_pub.publish(cmd_vel);
    }
    else if(count < maxCount){
        pointControl.set_vgoal(vectors[count].x, vectors[count].y, vectors[count].theta);
        count++;
    }
    else navMec_pub.publish(cmd_vel);


#ifndef DEBUGGER_MODE
    // If we got the goal --> send success info
    if(/* error < DEVIATION */1){
        trigger = false;
        nav_mec::navMec_srv res_srv;
        res_srv.request.start = true;
        navMec_cli.call(res_srv);
        // Ignore the response

        time_before = time_after = 0;
        pointControl.Reset_prev_v();
    }
#endif /* DEBUGGER_MODE */
}


// --- Need to be removed --- S
void constructVectors(){
    vectors[0].setxyz(0, 0, 0);
    vectors[1].setxyz(2., 3., 0);
    vectors[2].setxyz(-1., 0, 3.);
}// --- Need to be removed --- E