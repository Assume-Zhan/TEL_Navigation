#include "navMec_ros.h"

#include <cmath>
#include <iostream>

// --- Implement callback functions ---
/* Server Callback function */
bool serverCB(nav_mec::navMec_srv::Request& req, nav_mec::navMec_srv::Response& res){
    if(req.next.size() >= 0){
        trigger = true;
        res.get_request = true;

        /* Set next point */
        std::queue<char> modes;
        for(auto eachMode : req.mode){
            modes.push(eachMode);
        }
        pointControl.modeSettings(modes);

        std::queue<Vector3> goals;
        for(auto goal : req.next){
            goals.push(Vector3(goal.x, goal.y, goal.z));
        }

        pointControl.set_vgoal(goals);
    }
    else{
        trigger = false;
        res.get_request = false;
    }

    return true;
}

/* Subscriber Callback function */
void subCB(const localization::Locate::ConstPtr msg){
    // Subscribe location info from location node

    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    if(!debug_mode /* DEBUGMODE */ && !trigger){
        // Haven't trigger the navigation
        // Don't move the chases ---> give 0 speed ---> return
        navMec_pub.publish(cmd_vel);
        return;
    }

    static double time_before = 0.;
    static double time_after = 0.;
    if(!time_before){
        time_before = ros::Time::now().toSec();
        navMec_pub.publish(cmd_vel);
        return;
    }

    /* Get time different (Here just for calculate proper delta velocity) */
    time_after = ros::Time::now().toSec();
    double time_diff = time_after - time_before;
    time_before = time_after;

    /* Change geometry msgs to Vector3 */
    Vector3 location(msg->PositionX, msg->PositionY, msg->PositionOmega);
    Vector3 velocity(msg->VelocityX, msg->VelocityY, msg->VelocityOmega);

    switch(pointControl.getMode()){
        case 't':
        case 'b': {
            /* Check get goal and renew Error information */
                pointControl.check_get_goal(location);

                /* For calculate proper goal or move to next goal */
                if(!pointControl.getGoal){
                    cmd_vel = pointControl.get_vgoal(location, velocity, time_diff);
                    navMec_pub.publish(cmd_vel);
                }
                else if(debug_mode /* DEBUGMODE */ && count < maxCount){
                    std::cout << "Got goal : " << count << '\n';
                    std::queue<Vector3> next_goal;
                    next_goal.push(Vector3(vectors[count].x, vectors[count].y, vectors[count].theta));
                    pointControl.set_vgoal(next_goal);
                    count++;
                    navMec_pub.publish(cmd_vel);
                }
                else
                    navMec_pub.publish(cmd_vel);

                /* If we got the goal --> send success info */
                if(!debug_mode /* DEBUGMODE */ && pointControl.getGoal /* if we get the goal */){
                    trigger = false;
                    nav_mec::navMec_fsrv res_srv;
                    res_srv.request.finished = true;
                    while(!navMec_cli.call(res_srv))
                        ;
                    // Ignore the response
                    time_before = time_after = 0.;
                }
            } break;
        case 'c':
        case 'i': {
                timeoutReload -= time_diff;

                cmd_vel.linear.x = 0.;
                cmd_vel.linear.y = pointControl.getMode() == 'c' ? -calibMode_linear_y : calibMode_linear_y;
                cmd_vel.angular.z = 0.;

                navMec_pub.publish(cmd_vel);

                /* If we got the goal --> send success info */
                if(!debug_mode /* DEBUGMODE */ && timeoutReload < 0 /* if we get the goal */){
                    timeoutReload = timeout;

                    if(pointControl.calibMode_clearBuffer()){
                        trigger = false;
                        nav_mec::navMec_fsrv res_srv;
                        res_srv.request.finished = true;
                        while(!navMec_cli.call(res_srv))
                            ;
                        time_before = time_after = 0.;
                    }
                }
            } break;
        default:
            break;
    }

    ROS_INFO_STREAM("Current velocity : (" << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.angular.z << ")");

    // ROS_DEBUG_STREAM("Current mode : " << pointControl.getMode());
}

void subCBPP(const localization::Locate::ConstPtr msg){
    geometry_msgs::Twist cmd_vel;
    static double time_before = 0.;
    static double time_after = 0.;
    if(!time_before){
        time_before = ros::Time::now().toSec();
        cmd_vel.linear.x = 0.;
        cmd_vel.linear.y = 0.;
        cmd_vel.angular.z = 0.;
        navMec_pub.publish(cmd_vel);
        return;
    }

    /* Get time different (Here just for calculate proper delta velocity) */
    time_after = ros::Time::now().toSec();
    double time_diff = time_after - time_before;
    time_before = ros::Time::now().toSec();

    /* Change geometry msgs to Vector3 */
    Vector3 location(msg->PositionX, msg->PositionY, msg->PositionOmega);
    Vector3 velocity(msg->VelocityX, msg->VelocityY, msg->VelocityOmega);

    cmd_vel = purepursuit.get_vgoal(location, velocity, time_diff);

    navMec_pub.publish(cmd_vel);
}

// --- Need to be removed --- S
void constructVectors(){
    vectors[0].setxyz(0., -0.5, 0.);
    vectors[1].setxyz(0.97, -0.2, 0);
    vectors[2].setxyz(0.97, -0.65, 0);
    vectors[3].setxyz(2.47, -0.65, 0);
    vectors[4].setxyz(2.47, -0.2, 0);
    vectors[5].setxyz(3.35, -0.2, 0);
    vectors[6].setxyz(3.35, -0.7, 0);
    vectors[7].setxyz(4.6, -0.7, 0);
    vectors[8].setxyz(4.6, -0.275, 0);
    vectors[9].setxyz(5.6, -0.275, 0);
    vectors[10].setxyz(5.6, -0.65, 0);
    vectors[11].setxyz(6.6, -0.65, 0);
}
// --- Need to be removed --- E

// vectors[0].setxyz(0.5, 0.2, 0);
// vectors[1].setxyz(0.2, 0.8, 0);
// vectors[2].setxyz(0.6, 0.8, 0.98);
// vectors[3].setxyz(0.6, 1.6, 2.01);
// vectors[4].setxyz(0.2, 1.6, 3.14);
// vectors[5].setxyz(0.4, 2.7, 1.57);
// vectors[6].setxyz(0.4, 3.1, 0);
// vectors[7].setxyz(0.7, 3.7, 0);
// vectors[8].setxyz(0.7, 4.6, 0);
// vectors[9].setxyz(0.325, 4.9, 0);
// vectors[10].setxyz(0.325, 5.75, 0);
// vectors[11].setxyz(0.65, 5.95, 0);
// vectors[12].setxyz(0.65, 6.75, 0);

// vectors[0].setxyz(3.35, -0.5, 0.);
// vectors[1].setxyz(3.35, -0.7, 0);
// vectors[2].setxyz(4.6, -0.7, 0);
// vectors[3].setxyz(4.6, -0.275, 0);
// vectors[4].setxyz(5.6, -0.275, 0);
// vectors[5].setxyz(5.6, -0.65, 0);
// vectors[6].setxyz(6.6, -0.65, 0);