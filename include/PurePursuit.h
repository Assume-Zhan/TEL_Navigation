#ifndef PUREPURSUIT_H_
#define PUREPURSUIT_H_

#include <vector>

#include "geometry_msgs/Twist.h"
#include "PointController.h"

/**
 * @brief
 * Pure Pursuit controller
 * Since point controller will make the
 * car stop very often,
 * we introduce pure pursuit controller
 * for making the path more smooth.
 */
class PurePursuit{
public:

    /* Constructor */
    PurePursuit();

    /* Set goal path */
    void set_path(std::vector<Vector3>);

    /* Get current v goal */
    geometry_msgs::Twist get_vgoal(Vector3, Vector3, double);

    /* Check get path */
    bool check_get_path(Vector3);
    bool getPath = false;

private:

    /* We need a poincontroller to control point move */
    PointController* pointcontroller;

    // Path information
    std::vector<Vector3> path;

    // Goal information
    int nowGoalIdx = 0;

    // Find next point
    Vector3 findNextPoint(Vector3);

    // Is in circle
    bool isInCircle(Vector3, Vector3);

    // Get length
    double get_length(Vector3, Vector3);

    // Look ahead distance
    const double lookAhead_dis = 0.2;
};

#endif /* PUREPURSUIT_H_ */