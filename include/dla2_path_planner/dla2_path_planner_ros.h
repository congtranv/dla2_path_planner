/*
 * DLA2 Path Planner ROS - dla2_path_planner_ros.h
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 *
 */

#ifndef DLA2_PATH_PLANNER_H_
#define DLA2_PATH_PLANNER_H_

#include "ros/ros.h"
#include <dla2_path_planner/helper_functions.h>
// #include <geometry_msgs/Point.h>
#include <mav_planning_msgs/Point2D.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
// 2D Version
//#include <mav_planning_msgs/Point2D.h>
// 3D Version
#include <geometry_msgs/Point.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>


// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

class DLA2PathPlanner
{
public:
    DLA2PathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv);
    ~DLA2PathPlanner();

private:
    ros::NodeHandle &pnode_;
    ros::NodeHandle &node_;

    // parsed arguments
    double runTime;
    optimalPlanner plannerType;
    planningObjective objectiveType;
    std::string outputFile;

    void plan();

    // ROS Topics
    ros::Subscriber current_position_sub;
    ros::Subscriber goal_position_sub;
    ros::Publisher trajectory_pub;
    void currentPositionCallback(const mav_planning_msgs::Point2D::ConstPtr& p_msg);
    void goalPositionCallback(const mav_planning_msgs::Point2D::ConstPtr& p_msg);
    void convertOMPLPathToMsg();
    mav_planning_msgs::Point2D current_position, goal_position;
    bool traj_planning_successful;
    std::shared_ptr<ompl::geometric::PathGeometric> p_last_traj_ompl;
    mav_planning_msgs::PolynomialTrajectory4D last_traj_msg;
};

#endif // DLA2_PATH_PLANNER_H_
