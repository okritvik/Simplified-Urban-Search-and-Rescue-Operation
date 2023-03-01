# Simplified Urban Search & Rescue

The ﬁnal project is inspired by the the challenge of autonomous robotics for Urban Search and Rescue (US&R). In US&R after a disaster occurs, a robot is used to explore an unknown environment, such as a partially collapsed building, and locates trapped or unconscious human victims of the disaster. The robot builds a map of the collapsed building as it explores, and places markers in the map of where the victims are located. This map is then given to trained First Responders who use it to go into the building and rescue the victims.

In order to simplify the above challenge, the map of the unknown disaster environment is already assumed to exist and the human victims are replaced by ArUco markers which encode the pose of that location. There are 2 robots (TurtleBot3 Waffle) - Explorer and Follower. The explorer robot first navigates around the environment and looks for the ArUco markers (a.k.a victims). It saves the location information of these markers and later shares them with the follower robot, whose task is to 'rescue' the victims based on this shared information. Only the explorer robot is capable of detecting the 'victims'.   

Final Project for the course _ENPM809Y: Introductory Robot Programming (Fall 2021)_.


## Team Members

* Kumara Ritvik Oruganti (UID: 117368963)
* Adarsh Malapaka (UID: 118119625)
* Venkata Sai Ram Polina (UID: 118436579)

## Demo

<p align="center">
<a href="https://youtu.be/U7UHgr7ei_M" target="_blank">
 <img src="https://user-images.githubusercontent.com/40534801/215581561-e66e154e-622e-44a2-87d6-55ed90809d01.png" alt="Watch the Simulation" width="450" height="300" border="10" />
</a>
</p>
<p align = "center">
Link: https://youtu.be/U7UHgr7ei_M
</p>


## Report

<p align="center">
  <img src="https://user-images.githubusercontent.com/40534801/220487934-8dcc68c7-337e-4780-ad9b-77483228bbeb.png" width="300" height="300">
</p>


## Developer Documentation

### Dependencies 
<!-- * [ROS Noetic (roscpp)](http://wiki.ros.org/noetic)
* [MAVROS](https://github.com/mavlink/mavros)
* [os](https://docs.python.org/3/library/os.html)
* [pandas](https://pandas.pydata.org/)
* [bagpy](https://github.com/jmscslgroup/bagpy)
* [matplotlib](https://matplotlib.org/) -->

Run the ```install.bash``` script inside the ```script``` directory in the package to install all necessary packages

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
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        rosrun urban_search_rescue urban_search_rescue_node 
        ```

## Package Structure
```
├─ CMakeLists.txt
├─ README.md
├─ bag
│  ├─ enae788m_hw1_VOXL1
│  │  ├─ mavros-local_position-pose.csv
│  │  └─ mavros-setpoint_raw-local.csv
│  └─ enae788m_hw1_VOXL1.bag                # Saved rosbag for /mavros/local_position/pose & /mavros/setpoint_raw/local
├─ launch
│  ├─ hw1.launch                            # Launch file to run enae788m_hw1.cpp and rosbag record 
│  ├─ record_data.launch
│  └─ total_launch.launch
├─ package.xml
└─ src
   ├─ enae788m_hw1.cpp                      # Code to perform waypoint tracking as given in HW1 
   ├─ enae788m_plot.py                      # Code to visualize recorded ROS topics using rosbag
   └─ offboard_example.cpp


```
urban_search_rescue
├─ .vscode
│  ├─ c_cpp_properties.json
│  └─ settings.json
├─ CMakeLists.txt
├─ LICENSE
├─ README.md
├─ doc
│  ├─ Doxyfile
│  ├─ html
│  │  ├─ UMD_Logo.jpeg
│  │  ├─ _c_make_lists_8txt.html
│  │  ├─ _r_e_a_d_m_e_8md.html
│  │  ├─ annotated.html
│  │  ├─ bc_s.png
│  │  ├─ bdwn.png
│  │  ├─ class_urban___search___and___rescue-members.html
│  │  ├─ class_urban___search___and___rescue.html
│  │  ├─ classes.html
│  │  ├─ closed.png
│  │  ├─ dir_000001_000000.html
│  │  ├─ dir_68267d1309a1af8e8297ef4c3efbcdba.html
│  │  ├─ dir_68267d1309a1af8e8297ef4c3efbcdba_dep.map
│  │  ├─ dir_68267d1309a1af8e8297ef4c3efbcdba_dep.md5
│  │  ├─ dir_68267d1309a1af8e8297ef4c3efbcdba_dep.png
│  │  ├─ dir_d44c64559bbebec7f509842c48db8b23.html
│  │  ├─ doc.png
│  │  ├─ doxygen.css
│  │  ├─ doxygen.png
│  │  ├─ dynsections.js
│  │  ├─ files.html
│  │  ├─ folderclosed.png
│  │  ├─ folderopen.png
│  │  ├─ functions.html
│  │  ├─ functions_func.html
│  │  ├─ functions_type.html
│  │  ├─ functions_vars.html
│  │  ├─ globals.html
│  │  ├─ globals_func.html
│  │  ├─ graph_legend.html
│  │  ├─ graph_legend.md5
│  │  ├─ graph_legend.png
│  │  ├─ index.html
│  │  ├─ instructions_8txt.html
│  │  ├─ jquery.js
│  │  ├─ main_8cpp.html
│  │  ├─ main_8cpp__incl.map
│  │  ├─ main_8cpp__incl.md5
│  │  ├─ main_8cpp__incl.png
│  │  ├─ main_8cpp_source.html
│  │  ├─ md__r_e_a_d_m_e.html
│  │  ├─ menu.js
│  │  ├─ menudata.js
│  │  ├─ nav_f.png
│  │  ├─ nav_g.png
│  │  ├─ nav_h.png
│  │  ├─ open.png
│  │  ├─ pages.html
│  │  ├─ search
│  │  │  ├─ all_0.html
│  │  │  ├─ all_0.js
│  │  │  ├─ all_1.html
│  │  │  ├─ all_1.js
│  │  │  ├─ all_2.html
│  │  │  ├─ all_2.js
│  │  │  ├─ all_3.html
│  │  │  ├─ all_3.js
│  │  │  ├─ all_4.html
│  │  │  ├─ all_4.js
│  │  │  ├─ all_5.html
│  │  │  ├─ all_5.js
│  │  │  ├─ all_6.html
│  │  │  ├─ all_6.js
│  │  │  ├─ classes_0.html
│  │  │  ├─ classes_0.js
│  │  │  ├─ close.png
│  │  │  ├─ files_0.html
│  │  │  ├─ files_0.js
│  │  │  ├─ files_1.html
│  │  │  ├─ files_1.js
│  │  │  ├─ files_2.html
│  │  │  ├─ files_2.js
│  │  │  ├─ files_3.html
│  │  │  ├─ files_3.js
│  │  │  ├─ functions_0.html
│  │  │  ├─ functions_0.js
│  │  │  ├─ functions_1.html
│  │  │  ├─ functions_1.js
│  │  │  ├─ functions_2.html
│  │  │  ├─ functions_2.js
│  │  │  ├─ functions_3.html
│  │  │  ├─ functions_3.js
│  │  │  ├─ functions_4.html
│  │  │  ├─ functions_4.js
│  │  │  ├─ mag_sel.png
│  │  │  ├─ nomatches.html
│  │  │  ├─ pages_0.html
│  │  │  ├─ pages_0.js
│  │  │  ├─ search.css
│  │  │  ├─ search.js
│  │  │  ├─ search_l.png
│  │  │  ├─ search_m.png
│  │  │  ├─ search_r.png
│  │  │  ├─ searchdata.js
│  │  │  ├─ typedefs_0.html
│  │  │  ├─ typedefs_0.js
│  │  │  ├─ variables_0.html
│  │  │  └─ variables_0.js
│  │  ├─ splitbar.png
│  │  ├─ sync_off.png
│  │  ├─ sync_on.png
│  │  ├─ tab_a.png
│  │  ├─ tab_b.png
│  │  ├─ tab_h.png
│  │  ├─ tab_s.png
│  │  ├─ tabs.css
│  │  ├─ usar_8cpp.html
│  │  ├─ usar_8cpp__incl.map
│  │  ├─ usar_8cpp__incl.md5
│  │  ├─ usar_8cpp__incl.png
│  │  ├─ usar_8cpp_source.html
│  │  ├─ usar_8h.html
│  │  ├─ usar_8h__dep__incl.map
│  │  ├─ usar_8h__dep__incl.md5
│  │  ├─ usar_8h__dep__incl.png
│  │  ├─ usar_8h__incl.map
│  │  ├─ usar_8h__incl.md5
│  │  ├─ usar_8h__incl.png
│  │  └─ usar_8h_source.html
│  ├─ instructions.txt
│  └─ latex
│     ├─ Makefile
│     ├─ UMD_Logo.jpeg
│     ├─ _c_make_lists_8txt.tex
│     ├─ _r_e_a_d_m_e_8md.tex
│     ├─ annotated.tex
│     ├─ class_urban___search___and___rescue.tex
│     ├─ dir_68267d1309a1af8e8297ef4c3efbcdba.tex
│     ├─ dir_68267d1309a1af8e8297ef4c3efbcdba_dep.md5
│     ├─ dir_68267d1309a1af8e8297ef4c3efbcdba_dep.pdf
│     ├─ dir_d44c64559bbebec7f509842c48db8b23.tex
│     ├─ doxygen.sty
│     ├─ files.tex
│     ├─ instructions_8txt.tex
│     ├─ longtable_doxygen.sty
│     ├─ main_8cpp.tex
│     ├─ main_8cpp__incl.md5
│     ├─ main_8cpp__incl.pdf
│     ├─ md__r_e_a_d_m_e.tex
│     ├─ refman.tex
│     ├─ tabu_doxygen.sty
│     ├─ usar_8cpp.tex
│     ├─ usar_8cpp__incl.md5
│     ├─ usar_8cpp__incl.pdf
│     ├─ usar_8h.tex
│     ├─ usar_8h__dep__incl.md5
│     ├─ usar_8h__dep__incl.pdf
│     ├─ usar_8h__incl.md5
│     └─ usar_8h__incl.pdf
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
│  │  ├─ materials
│  │  │  ├─ scripts
│  │  │  │  └─ marker.material
│  │  │  └─ textures
│  │  │     └─ marker_0.png
│  │  ├─ model.config
│  │  └─ model.sdf
│  ├─ aruco_marker_1
│  │  ├─ materials
│  │  │  ├─ scripts
│  │  │  │  └─ marker.material
│  │  │  └─ textures
│  │  │     └─ marker_1.png
│  │  ├─ model.config
│  │  └─ model.sdf
│  ├─ aruco_marker_2
│  │  ├─ materials
│  │  │  ├─ scripts
│  │  │  │  └─ marker.material
│  │  │  └─ textures
│  │  │     └─ marker_2.png
│  │  ├─ model.config
│  │  └─ model.sdf
│  └─ aruco_marker_3
│     ├─ materials
│     │  ├─ scripts
│     │  │  └─ marker.material
│     │  └─ textures
│     │     └─ marker_3.png
│     ├─ model.config
│     └─ model.sdf
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