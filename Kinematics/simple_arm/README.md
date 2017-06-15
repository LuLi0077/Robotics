# simple_arm

A mini-project to better understand pub-sub architecture in ROS.


### How to launch the simulation?

Make sure all the required ROS packages installed, and `~/catkin_ws` is in the ROS workspace:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

Build the workspace (make sure at the workspace root directory):

```sh
$ catkin_make
```

Once the `simple_arm` package has been built, launch the simulation environment using:

```sh
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch simple_arm robot_spawn.launch
```

Robot can be controlled by publishing joint values to joint controller topics:

```
/simple_arm/joint_1_position_controller/command
/simple_arm/joint_2_position_controller/command
```

In a separate terminal type this command to control joint_1:

```sh
$ rostopic pub -1 /simple_arm/joint_1_position_controller/command std_msgs/Float64 "data: 1.5"
```

To control joint_2:

```sh
$ rostopic pub -1 /simple_arm/joint_2_position_controller/command std_msgs/Float64 "data: 1.5"
```

**Note: change the value of "data" argument to change joint_position values.**


### How to view image stream from the camera?

Camera image stream is published to following topic:

```
/rgb_camera/image_raw
```

This stream can be viewed by folliwng command in separate terminal:

```sh
$ rosrun image_view image_view image:=/rgb_camera/image_raw
```

If encounter an error and gazebo crashes while vieweing camera stream, upgrade gazebo7 to gazebo7.6:

```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```
