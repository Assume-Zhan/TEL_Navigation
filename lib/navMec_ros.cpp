#include <cmath>
#include <iostream>

#include "navMec_ros.h"

// --- Implement callback functions ---
/* Server Callback function */
bool serverCB(nav_mec::navMec_srv::Request& req, nav_mec::navMec_srv::Response& res){
    if(req.start == true)
        trigger = true,
        res.get_request = true;
    else
        trigger = true,
        res.get_request = false;

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
        return;
    }
    else if(firstTrigger){
        while(pointControl.check_get_goal(msg)){
            if(count < maxCount){
                pointControl.set_vgoal(vectors[count].x, vectors[count].y, vectors[count].theta);
                count++;
            }
            else break;
        }
        firstTrigger = false;
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
    else navMec_pub.publish(cmd_vel);


#ifndef DEBUGGER_MODE
    // If we got the goal --> send success info
    if(/* error < DEVIATION */ pointControl.check_get_goal(msg)){
        trigger = false;
        nav_mec::navMec_fsrv res_srv;
        res_srv.request.finished = true;
        navMec_cli.call(res_srv);
        // Ignore the response

        time_before = time_after = 0;
        pointControl.Reset_prev_v();

        // Set next goal
        if(count < maxCount){
            pointControl.set_vgoal(vectors[count].x, vectors[count].y, vectors[count].theta);
            count++;
        }
    }
#endif /* DEBUGGER_MODE */
}


// --- Need to be removed --- S
void constructVectors(){
    vectors[0].setxyz(0.5, 0.2, 0);
    vectors[1].setxyz(0.2, 0.8, 0);
    vectors[2].setxyz(0.6, 0.8, 0.98);
    vectors[3].setxyz(0.6, 1.6, 2.01);
    vectors[4].setxyz(0.2, 1.6, 3.14);
    vectors[5].setxyz(0.4, 2.7, 1.57);
    vectors[6].setxyz(0.4, 3.1, 0);
    vectors[7].setxyz(0.7, 3.7, 0);
    vectors[8].setxyz(0.7, 4.6, 0);
    vectors[9].setxyz(0.325, 4.9, 0);
    vectors[10].setxyz(0.325, 5.75, 0);
    vectors[11].setxyz(0.65, 5.95, 0);
    vectors[12].setxyz(0.65, 6.75, 0);
}// --- Need to be removed --- E