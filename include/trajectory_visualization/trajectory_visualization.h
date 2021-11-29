#ifndef icg_trajectory_visualization_H
#define icg_trajectory_visualization_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
// ROS, msgs, etc
#include "ros/ros.h"
// trajectory msg
#include "mav_planning_msgs/PolynomialTrajectory4D.h"
// rviz markers
#include <visualization_msgs/Marker.h>
// Boost
#include <boost/ref.hpp>


// I used source code from here: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines

class TrajectoryVisualization
{
public:
    TrajectoryVisualization(ros::NodeHandle &n, ros::NodeHandle &pn);

    typedef enum {
        RED   = 0,
        GREEN,
        BLUE ,
        ORANGE,
        WHITE,
        DEEP_BLUE
    } MarkersColor;

private:
    // ROS subscribers
    ros::Subscriber trajectory_sub;
    void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg);

    // ROS publishers
    ros::Publisher rviz_markers_white_publisher;
};

#endif // icg_trajectory_visualization_H
