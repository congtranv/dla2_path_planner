#include <iostream>
#include "ros/ros.h"
#include "trajectory_visualization/trajectory_visualization.h"

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::cout << "[ROSNODE] Starting TrajectoryVisualization" << std::endl;

    TrajectoryVisualization trajectory_visualization_node(n,pn);

    try
    {
        //Read messages
        ros::spin();
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}

