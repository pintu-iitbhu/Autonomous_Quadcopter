# Autonomus_Quadcopter

![interiit_!](https://user-images.githubusercontent.com/39412350/70888928-6cc60c80-2007-11ea-9d29-2551dd660f18.png)

![interiit2](https://user-images.githubusercontent.com/39412350/70888941-7485b100-2007-11ea-88a8-08d0904c5e26.png)

![drone_path](https://user-images.githubusercontent.com/39412350/70888996-94b57000-2007-11ea-80b2-6c247ab3ddf8.png)





# ROS INSTALLATION 

ROS Kinetic ONLY supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8) for debian packages

Setup your sources.list
```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys
```sh
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Installation
```sh
sudo apt-get update
```
```sh
sudo apt-get install ros-kinetic-desktop-full
```

Initialize rosdep
```sh
sudo rosdep init
rosdep update
```

Environment setup
```sh
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Check the version of gazebo installed
It should be gazebo7.
```sh
gazebo -v
```
Run 
```sh
gazebo
```


# MAVROS

The mavros ROS package enables MAVLink extendable communication between computers running ROS, MAVLink enabled autopilots, and MAVLink enabled GCS.

### Installation

Binary Installation (Debian / Ubuntu)
```sh
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```
```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```
Source Installation
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```
```sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```
```sh
catkin init
```
```sh
sudo apt-get install wstool
```
```sh
wstool init src
wstool init ~/catkin_ws/src
```
Install MAVLink
```sh
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
```

Install MAVROS from source 
```sh
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```
Create workspace & deps
```sh
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

Install GeographicLib datasets:
```sh
sudo apt-get ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
Build source
```sh
catkin build
```
Make sure that you use setup.bash or setup.zsh from workspace.
```sh
source devel/setup.bash
```
### You again need to check the version of gazebo.
if wouldn't get gazebo7, then remove the version that you have on you PC. Reinstall gazebo7.
Here is an example of removing gazebo9
```sh
sudo apet-get remove gazebo9
```
Installation of gazebo7
```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
```sh
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
```sh
$ sudo apt-get update
$ sudo apt-get install gazebo7
# For developers that work on top of Gazebo, one extra package
$ sudo apt-get install libgazebo7-dev
```
