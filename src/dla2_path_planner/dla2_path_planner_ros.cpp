/*
 * DLA2 Path Planner ROS - dla2_path_planner_ros.cpp
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 *
 */

#include <dla2_path_planner/dla2_path_planner_ros.h>

// congtranv
// #include<dynamicEDT3D/dynamicEDTOctomap.h>
// #include<dynamicEDT3D/dynamicEDT3D.h>
// #include<iostream>

DLA2PathPlanner::DLA2PathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv) :
    pnode_(pn),
    node_(n),
    traj_planning_successful(false)
{   
    // ROS topics
    // current_position_sub = pnode_.subscribe("current_position", 10, &DLA2PathPlanner::currentPositionCallback, this);
    // goal_position_sub = pnode_.subscribe("goal_position", 10, &DLA2PathPlanner::goalPositionCallback, this);

    // congtranv
    current_3d_position_sub = pnode_.subscribe("current_3d_position", 10, &DLA2PathPlanner::current3DPositionCallback, this);
    goal_3d_position_sub = pnode_.subscribe("goal_3d_position", 10, &DLA2PathPlanner::goal3DPositionCallback, this);

    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);

    // current_position.x = 0.; current_position.y = 0.;
    // goal_position.x = 1.; goal_position.y = 1.;

    // congtranv
    current_3d_position.x = 0.0; current_3d_position.y = 0.0; current_3d_position.z = 0.0;
    goal_3d_position.x = 1.0; goal_3d_position.y = 1.0; goal_3d_position.z = 1.0;

    // Parse the arguments, returns true if successful, false otherwise
    if (argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile))
    {
        // Return with success
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse success!");
    } else {
        // Return with error - Modified argParse to make this equivalent to giving no arguments.
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse error!");
    }
}

DLA2PathPlanner::~DLA2PathPlanner() {

}

// void DLA2PathPlanner::currentPositionCallback(const mav_planning_msgs::Point2D::ConstPtr& p_msg) {
//     current_position = *p_msg;
//     ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y);
// }

// void DLA2PathPlanner::goalPositionCallback(const mav_planning_msgs::Point2D::ConstPtr& p_msg) {
//     goal_position = *p_msg;
//     ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y);

//     plan();

//     if (traj_planning_successful) {
//         convertOMPLPathToMsg();
//         mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg = 
//             mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_msg ) );
//         trajectory_pub.publish(last_traj_msg);
//     }
// }

// congtranv
void DLA2PathPlanner::current3DPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    current_3d_position = *msg;
    ROS_INFO_STREAM("New current position, x: " << current_3d_position.x << "; y: " << current_3d_position.y << "; z: " << current_3d_position.z);
}
// congtranv
void DLA2PathPlanner::goal3DPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    goal_3d_position = *msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_3d_position.x << "; y: " << goal_3d_position.y << "; z: " << goal_3d_position.z);

    plan();

    if (traj_planning_successful) {
        convertOMPLPathToMsg();
        mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg = 
            mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_msg ) );
        trajectory_pub.publish(last_traj_msg);
    }
}

void DLA2PathPlanner::convertOMPLPathToMsg() {
    mav_planning_msgs::PolynomialTrajectory4D &msg = last_traj_msg;
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = p_last_traj_ompl->getStates();
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        // double z_s = 0.;

        //congtranv
        const double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];

        double yaw_s = 0.;
        // ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s);

        // congtranv
        ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }
}

void DLA2PathPlanner::plan()
{
    // congtranv
    //---------------------------------------------//
    octomap::OcTree *tree = NULL;
    tree = new octomap::OcTree(0.05);
    tree->readBinary("/home/congtranv/ros/tu_graz_cdl/tug_ws/src/dla2_path_planner/maps/power_plant.bt");
    // tree->readBinary("/home/congtranv/ros/tu_graz_cdl/tug_ws/src/dla2_path_planner/maps/fr_078_tidyup.bt");
    std::cout<<"[ ] read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

    double x,y,z;
    tree->getMetricMin(x,y,z);
    octomap::point3d min(x,y,z);
    std::cout<<"[ ] Metric min: "<<x<<","<<y<<","<<z<<std::endl;
    tree->getMetricMax(x,y,z);
    octomap::point3d max(x,y,z);
    std::cout<<"[ ] Metric max: "<<x<<","<<y<<","<<z<<std::endl;
    
    bool unknownAsOccupied = true;
    // unknownAsOccupied = false;
    // float maxDist = 1.0;
    float maxDist = 0.5;
    //- the first argument ist the max distance at which distance computations are clamped
    //- the second argument is the octomap
    //- arguments 3 and 4 can be used to restrict the distance map to a subarea
    //- argument 5 defines whether unknown space is treated as occupied or free
    //The constructor copies data but does not yet compute the distance map
    // DynamicEDTOctomap distmap(maxDist, tree, min, max, unknownAsOccupied);
    distmap = new DynamicEDTOctomap(maxDist, tree, min, max, unknownAsOccupied);
    //This computes the distance map
    // distmap.update();
    distmap->update();
    // -------------------------------------------//

    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    // auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    
    // Set the bounds of space to be in [0,1].
    // space->setBounds(0.0, 1.0);
    
    // congtranv
    // space->addDimension(0.0, 1.0);

    // congtranv
    auto space(std::make_shared<ob::RealVectorStateSpace>());
    space->addDimension(min.x(), max.x()); 
    space->addDimension(min.y(), max.y());
    space->addDimension(min.z(), max.z());

    std::cout << "[ ] space dimension: " << space->getDimension() << "\n";

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    // start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
    // start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;

    // congtranv
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_3d_position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_3d_position.y;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_3d_position.z; 

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;

    // congtranv
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_3d_position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_3d_position.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_3d_position.z;

    // congtranv
    start_time = ros::Time::now();

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);
    calculation_time = ros::Time::now().toSec() - start_time.toSec();

    if (solved)
    {
        // congtranv
        // calculation_time = ros::Time::now().toSec() - start_time.toSec();
        std::printf("[ ] Point reached: %s \n", solved ? "true":"false");
        std::printf("[ ] Calculation time: %f \n", calculation_time);

        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;        

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (!outputFile.empty())
        {
            std::ofstream outFile(outputFile.c_str());
            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
        }

        p_last_traj_ompl =  std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
        traj_planning_successful = true;
    } else {
        std::cout << "No solution found." << std::endl;
        traj_planning_successful = false;
    }
}
