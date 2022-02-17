
## SRVT-ROS Installation & Usage Guide ##

### Required setups to run SRVT Environment:
------------------------------------------------------------------

- To install ROS Noetic:

```bash
http://wiki.ros.org/noetic/Installation/Ubuntu
```

- For installations of ROS dependencies the following command should be run:

```bash
sudo apt-get install ros-noetic-moveit && sudo apt-get install ros-noetic-controller-manager && sudo apt-get install ros-noetic-joint-trajectory-controller && sudo apt-get install ros-noetic-rqt-joint-trajectory-controller && sudo apt-get install ros-noetic-effort-controllers
```
- Then the rosdep commands should be run.

```bash
sudo rosdep init && rosdep update
```
- Finally, the "srvt_ros" package must be dumped into the ROS workspace and compiled.

```bash
cd ~/catkin_ws && catkin_make && catkin_make install
```

### Editing Any Robot Model Files

- Extract the "model" file of the robot to be used in the ".gazebo/models" folder and into the "srvt_ros" folder.

   ##### NOTE: If the gazebo has not been run before, the gazebo must be run by typing the "gazebo" command from the terminal in order to create the .gazebo file.
   ##### NOTE2: If the "models" folder does not exist, it must be created.
 
- Put the "meshes" folder of the robot model into the "srvt_description" folder.

- edit the ~/.bashrc file contents for the models to work.

### Editing the ~/.bashrc File Content

- The following commands need to be added to the ~/.bashrc file.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source /usr/share/gazebo-11/setup.sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/srvt_ros/model/:$GAZEBO_MODEL_PATH
```

### Usage Commands of SRVT
 
- To run SRVT, the following commands must be run in order and on separate terminals.

To run the SRVT Gazebo:

```bash
roslaunch srvt_moveit start_system.launch
```

To run the SRVT Image Server node:

```bash
rosrun srvt_moveit image_service_node.py
```

To run SRVT Moveit:

```bash
roslaunch srvt_moveit start_moveit.launch
```

To run SRVT Task Service:

```bash
roslaunch srvt_moveit start_rokos_task_service.launch
```

To run SRVT Smach node:

```bash	
roslaunch srvt_moveit start_rokos_smach.launch
```

To view the robot's branches from RVIZ (in srvt_rviz folder, have some example Rviz file for this purpose):

```bash	
cd ~/catkin_ws/src/srvt_ros/srvt_rviz
rosrun rviz rviz -d left_rokos_rviz.rviz (ya da right_rokos_rviz.rviz)
```
