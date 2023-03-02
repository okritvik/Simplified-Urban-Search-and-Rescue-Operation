[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.png)](https://opensource.org/licenses/MIT)

# Simplified Urban Search & Rescue

The ﬁnal project is inspired by the challenge of autonomous robotics for Urban Search and Rescue (US&R). In US&R after a disaster occurs, a robot is used to explore an unknown environment, such as a partially collapsed building, and locates trapped or unconscious human victims of the disaster. The robot builds a map of the collapsed building as it explores, and places markers in the map of where the victims are located. This map is then given to trained First Responders who use it to go into the building and rescue the victims.

In order to simplify the above challenge, the map of the unknown disaster environment is already assumed to exist and the human victims are replaced by ArUco markers which encode the pose of that location. There are 2 robots (TurtleBot3 Waffle) - Explorer and Follower. The explorer robot first navigates around the environment and looks for the ArUco markers (a.k.a victims). It saves the location information of these markers and later shares them with the follower robot, whose task is to 'rescue' the victims based on this shared information. Only the explorer robot is capable of detecting the 'victims'.   

Final Project for the course _ENPM809Y: Introductory Robot Programming (Fall 2021)_.

<p align="center">
    <img src="https://user-images.githubusercontent.com/40534801/222019574-52eb334a-5ece-4683-8c35-0f379bb57ab2.png" width="500" height="300">
</p>

## Team Members

* Kumara Ritvik Oruganti (UID: 117368963) okritvik@terpmail.umd.edu
* Adarsh Malapaka (UID: 118119625) amalapak@terpmail.umd.edu
* Venkata Sai Ram Polina (UID: 118436579) sairamp@umd.edu

## Demo

### Simulation

The below video corresponds to the demo of the search and rescue task with the ArUco locations as provided by the course instructor.

<p align="center">
<a href="https://www.youtube.com/watch?v=bqUnzQA4Owg" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/222015993-7c4b03eb-c1a6-4317-b077-c26ba82f5edc.png" alt="Watch the Simulation" width="450" height="300" border="10" />
</a>
</p>
<p align = "center">
Link: https://www.youtube.com/watch?v=bqUnzQA4Owg
</p>

### Testing

The below video corresponds to the demo of the search and rescue task with the ArUco locations as custom defined by the team in order to validate our implementation with different test cases. 

<p align="center">
<a href="https://www.youtube.com/watch?v=N_4cfe7u9H8" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/222016364-382d4904-cf1e-44b5-b66f-649dab8a4091.jpg" alt="Watch the Simulation" width="450" height="300" border="10" />
</a>
</p>
<p align = "center">
Link: https://www.youtube.com/watch?v=N_4cfe7u9H8
</p>

### ArUco Marker Locations

The locations of the ArUco markers for the simulation and test cases are shown below.

| Location | ArUco ID | X<sub>sim</sub> (m) | Y<sub>sim</sub> (m) | X<sub>test</sub> (m) | Y<sub>test</sub> (m) | 
| :---: | :---: | :---: | :---: | :---: | :---: |
| 1 | 0 | -1.752882 | 3.246192 | -1.192846 | 2.821559 |
| 2 | 1 | -2.510293 | 1.125585 | -2.510293 | 1.125585 |
| 3 | 3 | -0.289296 | -1.282680 | -0.289296 | -1.282680 |
| 4 | 2 | -7.710214 | -1.716889 | 7.710214 | -1.716889 |


## Report

<p align="center">
    <a href="https://github.com/adarshmalapaka/urban-search-and-rescue/blob/12b5987e43a0270cb9afd2f1e4a206a34402f030/final_project_G9-Report.pdf" target="_blank">
        <img src="https://user-images.githubusercontent.com/40534801/222019082-5b1d3023-87e2-490d-b3c9-fd8b5ea30d7c.png" width="300" height="300">
    </a>
</p>


### Course Instructor Feedback
> * This is a very good report, which can be considered to be published in a conference. All the important aspects of the project have been highlighted. - Very good attention to detail and lots of efforts put into the report. - Your "Testing Phase" section is very informative and very well detailed. - Lots of efforts were put in Figure 15. - Some other groups lost points for including figures in their reports and not referencing the figures in the text. I'm glad you did not take this route. - One advice, avoid using "we" or "our" in your report as this is frown upon. Rewrite your sentences using the past tense if you can. For instance: Change "Due to this problem, we stopped with only changing one ArUco position" to "Due to this problem, only changing one ArUco position was performed..." - Finally one team who could get "ArUco" right.



## Developer Documentation

### Dependencies 
* [ROS Melodic (roscpp)](http://wiki.ros.org/melodic) or [ROS Noetic (roscpp)](http://wiki.ros.org/noetic)
* [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
* [aruco_detect](http://wiki.ros.org/aruco_detect)
* [fiducial_msgs](http://wiki.ros.org/fiducial_msgs)
* [tf2](http://wiki.ros.org/tf2)
* [move_base](http://wiki.ros.org/move_base)

Run the ```install.bash``` script inside the ```script``` directory in the package to install all necessary packages.

### Running the Codes

1. Open a terminal and clone the repository in the ```src``` directory of your ROS catkin workspace into a folder/package called ```urban_search_rescue```.
  ```
  source /opt/ros/<your-ros-distro>/setup.bash
  cd ~/catkin_ws/src
  git clone https://github.com/adarshmalapaka/urban-search-and-rescue.git urban_search_rescue
  cd ..
  catkin_make 
  ```
2. To start the Gazebo World, RViz, spawn robots and set parameters on the Parameter Server:
    * In a terminal:
        ```
        source /opt/ros/<your-ros-distro>/setup.bash
        source ~/catkin_ws/devel/setup.bash
        roslaunch urban_search_rescue multiple_robots.launch
        ```
3. To run the search and rescue ROS node:
    * In another terminal
        ```
        source /opt/ros/<your-ros-distro>/setup.bash
        source ~/catkin_ws/devel/setup.bash
        rosrun urban_search_rescue urban_search_rescue_node 
        ```

## Package Structure
```
urban_search_rescue
├─ CMakeLists.txt
├─ LICENSE
├─ README.md
├─ doc
│  ├─ Doxyfile
│  └─ html
├─ final_project_G9-Report.pdf
├─ final_project_fall2021.pdf
├─ include
│  └─ usar.h
├─ launch
│  ├─ explorer_amcl.launch
│  ├─ explorer_move_base.launch
│  ├─ follower_amcl.launch
│  ├─ follower_move_base.launch
│  ├─ mapping
│  │  ├─ bringup.launch
│  │  ├─ start_amcl.launch
│  │  ├─ start_mapping.launch
│  │  └─ start_navigation.launch
│  ├─ multiple_robots.launch
│  ├─ navigation.launch
│  ├─ single_robot.launch
│  └─ start_world.launch
├─ maps
│  ├─ final_world.pgm
│  ├─ final_world.yaml
│  ├─ final_world2.pgm
│  └─ final_world2.yaml
├─ models
│  ├─ aruco_marker_0
│  ├─ aruco_marker_1
│  ├─ aruco_marker_2
│  └─ aruco_marker_3
├─ package.xml
├─ param
│  ├─ aruco_lookup.yaml
│  ├─ base_local_planner_params.yaml
│  ├─ costmap_common_params_explorer.yaml
│  ├─ costmap_common_params_follower.yaml
│  ├─ dwa_local_planner_params.yaml
│  ├─ global_costmap_params_explorer.yaml
│  ├─ global_costmap_params_follower.yaml
│  ├─ local_costmap_params_explorer.yaml
│  ├─ local_costmap_params_follower.yaml
│  ├─ move_base_params.yaml
│  └─ real_aruco_poses.yaml
├─ rviz
│  ├─ bringup.rviz
│  ├─ mapping.rviz
│  ├─ navigation.rviz
│  └─ turtlebot3_navigation.rviz
├─ script
│  └─ install.bash
├─ src
│  ├─ main.cpp
│  └─ usar.cpp
└─ world
   ├─ aruco_marker.world
   └─ final_world.world

```
