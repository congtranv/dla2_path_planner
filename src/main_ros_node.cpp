/*
 *  DLA2 Path Planner ROS Node - main_ros_node.cpp
 *
 *  Description: Minimal ROS Node example implementing functionality from OMPL 
 *               example demos/OptimalPlanning.cpp (checked on y2019m12d19).
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <dla2_path_planner/dla2_path_planner_ros.h>

int main(int argc, char** argv)
{
    ROS_INFO("Checking input arguments!");
    ROS_INFO_STREAM("argc: " << argc);
    for (int i=0; i<argc; i++) {
        ROS_INFO_STREAM("argv[" << i << "]: " << argv[i]);
    }
    ROS_INFO("\n");

    /* initialize ros */
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO("Starting DLA2PathPlanner...");
    DLA2PathPlanner dla2_path_planner( nh, pnh, argc, argv);
    ROS_INFO("DLA2PathPlanner started...");

    ros::spin();

    return 0;
}
