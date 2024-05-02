# autonomous_mobile_manipulation

Basic setup assumes *Ubuntu 18.04* with [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) and [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) installed.

### Some dependencies
```
sudo apt install \
collada-urdf-tools \
swig \
libglew-dev \
libmetis-dev \
libsuitesparse-dev \
libnlopt-dev \
libfcl-dev \
liboctomap-dev \
ros-tf2-msgs \
ros-melodic-collada-urdf \
ros-melodic-eigenpy \
ros-melodic-rosparam-shortcuts \
ros-melodic-eigen-stl-containers \
ros-melodic-geometric-shapes \
ros-melodic-object-recognition-msgs \
ros-melodic-octomap \
ros-melodic-octomap-msgs \
ros-melodic-octomap-ros \
ros-melodic-octomap-server \
ros-melodic-random-numbers \
ros-melodic-srdfdom \
ros-melodic-ros-control \
ros-melodic-ros-controllers \
ros-melodic-position-controllers \
ros-melodic-velocity-controllers \
ros-melodic-effort-controllers \
ros-melodic-joint-state-controller \
ros-melodic-joint-trajectory-controller \
ros-melodic-rqt-joint-trajectory-controller \
ros-melodic-rqt-joint-trajectory-plot \
ros-melodic-rqt-controller-manager \
ros-melodic-force-torque-sensor-controller \
ros-melodic-industrial-robot-status-controller \
ros-melodic-industrial-robot-status-interface \
ros-melodic-trac-ik-kinematics-plugin \
ros-melodic-gazebo-ros \
ros-melodic-gazebo-ros-control \
ros-melodic-hector-gazebo \
ros-melodic-hector-imu-tools \
ros-melodic-geographic-msgs \
ros-melodic-graph-msgs \
ros-melodic-warehouse-ros \
ros-melodic-ddynamic-reconfigure \
ros-melodic-pointgrey-camera-description \
ros-melodic-interactive-marker-twist-server \
ros-melodic-twist-mux \
ros-melodic-joystick-drivers \
ros-melodic-teleop-twist-joy \
ros-melodic-teleop-tools \
ros-melodic-move-base
```

### Pull resources
```
# Setup and pull
mkdir -p $HOME/autonomous_mobile_manipulation_ws/src
cd $HOME/autonomous_mobile_manipulation_ws/src
git clone https://github.com/robowork/autonomous_mobile_manipulation
cd autonomous_mobile_manipulation
git submodule update --init --recursive

# Copy gazebo models
cp -r gazebo_resources/models/* $HOME/.gazebo/models/
```

### Clone required libraries and install them in devel space
```
# GTSAM
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/borglab/gtsam && cd gtsam && git checkout 4.0.2 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# g2o
cd $HOME/autonomous_mobile_manipulation_ws &&  git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && git checkout 20200410_git && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=OFF -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# Libnabo
cd $HOME/autonomous_mobile_manipulation_ws && git clone git@github.com:ethz-asl/libnabo.git && cd libnabo && git checkout 1.0.7 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# Libpointmatcher
cd $HOME/autonomous_mobile_manipulation_ws && git clone git@github.com:ethz-asl/libpointmatcher.git && cd libpointmatcher && git checkout 1.3.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install

# RTABMAP
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/introlab/rtabmap && cd rtabmap && git checkout 0.20.3 && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCSPARSE_INCLUDE_DIR=$HOME/autonomous_mobile_manipulation_ws/g2o/EXTERNAL/csparse -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install 

# FCL
cd $HOME/autonomous_mobile_manipulation_ws && git clone https://github.com/flexible-collision-library/fcl && cd fcl && git checkout 0.6.1 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/autonomous_mobile_manipulation_ws/devel ..
make -j 12 && make install 
```

### Build
```
cd $HOME/autonomous_mobile_manipulation_ws

# Apply small required patches to external dependencies
patch -p0 < src/autonomous_mobile_manipulation/deps/patches/TebLocalPlanner_FindG2O_cmake.patch

catkin config -DCMAKE_BUILD_TYPE=Release -DOMPL_REGISTRATION=OFF -DG2O_INCLUDE_DIR=$HOME/autonomous_mobile_manipulation_ws/devel/include

# Builds have to be informed that above built libraries are installed in devel space and should look in there first:

# Set environment variable each command:
CMAKE_PREFIX_PATH=$HOME/autonomous_mobile_manipulation_ws/devel:$CMAKE_PREFIX_PATH catkin build

# Or export for entire shell session:
export CMAKE_PREFIX_PATH=$HOME/autonomous_mobile_manipulation_ws/devel:$CMAKE_PREFIX_PATH
catkin build 
```

### Note:
In case you run ```catkin clean```, this will clear out the devel space where the above libraries are installed. You have to reinstall them with ```make install``` (no need to rebuild): 
```
cd $HOME/autonomous_mobile_manipulation_ws/gtsam/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/g2o/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/libnabo/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/libpointmatcher/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/rtabmap/build && make install && \
cd $HOME/autonomous_mobile_manipulation_ws/fcl/build && make install
```

### Simulation using gazebo

```
# Terminal 1 - Launch Simulation 
roslaunch robowork_gazebo bvr_SIM_playpen.launch

# Terminal 2 - Launch MoveIt! move_group planning
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_moveit_config robowork_moveit_planning_execution.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM

# Terminal 3 - Visualization (If not already running for the real robot)
roslaunch robowork_moveit_config moveit_rviz.launch
```

Example Case A) Launch a sample MoveIt! planning pipeline - This is a general test to give goal commands to the mobile robot and the manipulator end-effector
```
# Terminal A1 - Launch robowork_planning sample node (vTest)
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_planning move_group_interface_vTest.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM

# Terminal A2 - Publish a goal pose in the 'map' frame for the mobile robot
rostopic pub --once -s /rviz_2d_nav_goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'map'}, pose: {position: {x: 0.75, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}"

# The move_base node will immediately start planning and moving to execute the goal 

# Terminal A3 (option i) - Publish a goal position in the 'map' frame for the end-effector
rostopic pub --once -s /endeffector_goal_position geometry_msgs/PointStamped "{header: {seq: 0, stamp: now, frame_id: 'map'}, point: {x: 1.75, y: 0, z: 0.75}}"

# Terminal A3 (option ii) - Publish a goal pose (position & orientation) in the 'map' frame for the end-effector
rostopic pub --once -s /endeffector_goal_pose geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'map'}, pose: {position: {x: 1.75, y: 0, z: 0.75}, orientation: {x: 0, y: 0.383, z: 0, w: 0.924}}}"

# On Terminal A1 you should see a message from the move_group_interface_vTest node:
# "move_group_interface: New PointStamped goal in frame [map] received!"

# The move_group_interface_vTest node will initiate planning execution once you press 'Next' on Rviz to trigger it.
# Alternatively you can achieve the same thing by publishing a ROS topic on Terminal A3:
rostopic pub --once -s /rviz_visual_tools_gui sensor_msgs/Joy "{header: {seq: 0, stamp: now, frame_id: 'map'}, buttons: [0,1,0,0,0,0]}"
```

Example Case B) Launch a sample MoveIt! planning pipeline - This is based on the ```master``` branch environment that contains an AprilTag marker  
```
# Terminal B - Launch robowork_planning sample node (vAprilTag)
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_planning move_group_interface_vAprilTag.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM

# Press 'Next' on Rviz to trigger planning to reach AprilTag
```

Example Case C) Launch Compliance and Jog the end-effector
```
# Terminal C1 - Launch ur5e_compliance
ROS_NAMESPACE="bvr_SIM" roslaunch robowork_ur_launch ur5e_compliance.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM

# Terminal C2 - Enable compliance
rosservice call /compliance_controller/toggle_compliance "{}"

# Move the interactive_marker_3d_twist on Rviz to command jogging references
```
