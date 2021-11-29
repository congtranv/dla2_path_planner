Drones' Lecture Assigment 2 - Path Planner
------------------------------------------

Compilation instructions
------------------------

1. Install every dependency listed on the CMakeLists.txt and/or in the package.xml file. In particular the ompl library:

sudo apt install ros-melodic-ompl*

inside the src workspace folder: git clone https://github.com/ethz-asl/mav_comm.git

2. Download this package inside the src folder of your ROS workspace.

3. Compile it using either the command 'catkin_make' or 'catkin build'.

Testing instructions
--------------------

* To start the ROS node run either of:

rosrun dla2_path_planner dla2_path_planner_ros_node

rosrun dla2_path_planner dla2_path_planner_ros_node --runtime 0.5 --planner RRTStar -o WeightedLengthAndClearanceCombo -f planner_trajectory.txt --info 2

* To change the initial/current position run:

2D (master branch):
rostopic pub /path_planner/current_position mav_planning_msgs/Point2D "x: 0.1
y: 0.1" --once

3D (when using 3D version):
rostopic pub /path_planner/current_position geometry_msgs/Point "x: 0.1                                  
y: 0.1
z: 0.1" --once

* To change the initial/goal position and plan a trajectory run:

2D (master branch):
rostopic pub /path_planner/goal_position mav_planning_msgs/Point2D "x: 0.9
y: 0.9" --once

3D (when using 3D version):
rostopic pub /path_planner/goal_position geometry_msgs/Point "x: 0.9
y: 0.9
z: 0.9" --once

* To receive the planned trajectory on the terminal run:

rostopic echo /path_planner/planned_trajectory

* For the simple visualization that we prepared for you, you need to run:

rosrun dla2_path_planner dla2_path_planner_trajectory_visualization

rviz

inside rviz add a "display" of typer "Marker" subscribed to the topic "/trajectory_visualization/trajectory_markers". Afterwards every new trajectory published by the planner should be visualized in rviz.

* To visualize octomap

Add power_plant.bt file to the maps folder: dla2_path_planner/maps/power_plant.bt

Launch the octomap_server:

roslaunch dla2_path_planner octomap_mapping_a2.launch

inside rviz add a "display" of type "OccupancyGrid" subscribed to the topic "/octomap_binarys". Afterwards the map for power_plant.bt should be visualized in rviz.

* Integrating dynamic_edt_3d

sudo apt install ros-melodic-octovis* ros-melodic-octomap* ros-melodic-dynamic-edt-3d*

Code example from the library chosen by the tutor: dynamicEDT3D/src/examples/exampleEDTOctomap.cpp

https://github.com/OctoMap/octomap/blob/devel/dynamicEDT3D/src/examples/exampleEDTOctomap.cpp

The CMakeLists.txt file needs to load the octomap and the dynamicEDT3D libraries. This part is left to be done by the student, please use the lines related to adding the ompl library on the CMakeLists.txt as example.

