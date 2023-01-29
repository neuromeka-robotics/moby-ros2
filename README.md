# MOBY ROS2 FOXY

## Installation
### Supported OS (2023.01.08)
* Ubuntu 20.04
* ROS2 Foxy

### BIOS Setting (NUC)
* Connect monitor, keyboard and mouse to NUC and reboot, press F2 to enter BIOS Setup 
* **Boot** > **Secure boot** > **Secure boot** > **Disabled**
* **Power** > **Secondary Power Settings** > **After Power Failure** > **Power On**
* **Advanced** > **Onboard Devices** > **Bluetooth** > *Uncheck*
* Press **F10** to Save and Exit

### Basic Setup
* Install basic tools
```bash
sudo apt-get update \
&& sudo apt-get install -y git openssh-server \
&& sudo apt-get install -y python3-pip \
&& sudo pip3 install --upgrade pip \
&& sudo pip3 install setuptools==41.0.0 \
&& sudo apt install protobuf-compiler \
&& python3 -m pip install protobuf==3.19.4 grpcio==1.34.1 grpcio_tools==1.34.1
```

* Disable kernel update (kernel updates sometimes break some package functions)
```bash
sudo apt-mark hold linux-image-generic linux-headers-generic
```

### Install ROS2
#### Setup sources
```bash
sudo apt install software-properties-common \
&& sudo add-apt-repository universe \
&& sudo apt update && sudo apt install curl \
&& sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
&& sudo apt update
```

#### Install ROS2 and dependent packages
```bash
sudo apt install -y ros-foxy-desktop python3-argcomplete \
&& sudo apt install -y ros-foxy-perception-pcl \
&& sudo apt install -y ros-foxy-cartographer* \
&& sudo apt install -y ros-foxy-xacro  \
&& sudo apt install -y ros-foxy-ros2-control  \
&& sudo apt install -y ros-foxy-ros2-controllers  \
&& sudo apt install -y ros-foxy-controller-manager  \
&& sudo apt install -y ros-foxy-joint-state-controller  \
&& sudo apt install -y ros-foxy-joint-state-broadcaster \
&& sudo apt install -y ros-foxy-joint-state-publisher-gui \
&& sudo apt install -y ros-foxy-navigation2 \
&& sudo apt install -y ros-foxy-nav2* \
&& sudo apt install -y ros-foxy-geographic-msgs \
&& sudo apt install -y ros-foxy-robot-localization \
&& sudo apt install -y ros-foxy-joy-linux \
&& sudo apt install -y ros-foxy-libg2o \
&& sudo apt install -y ros-foxy-slam-toolbox \
&& sudo apt install -y python3-colcon-common-extensions \
&& sudo apt-get install python3-rosdep -y \
&& sudo rosdep init
```

#### source workspace
```bash
source /opt/ros/foxy/setup.bash
echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc
```

### Install Sensor Dependencies
#### Realsense
* librealsense
  * **NOTE** Version fixed because sensors are not recognized with recent version (2.53)
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
|| sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
&& sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
&& sudo apt-get -y install librealsense2-dkms=1.3.18-0ubuntu1 \
    librealsense2=2.50.0-0~realsense0.6128 \
    librealsense2-gl=2.50.0-0~realsense0.6128 \
    librealsense2-net=2.50.0-0~realsense0.6128 \
    librealsense2-udev-rules=2.50.0-0~realsense0.6128 \
    librealsense2-utils=2.50.0-0~realsense0.6128 \
    librealsense2-dev=2.50.0-0~realsense0.6128 \
    librealsense2-dbg=2.50.0-0~realsense0.6128 \
&& sudo pip3 install pyrealsense2==2.50.0.3812
```
 
### Build Moby Source
* Create workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

* Copy the project's ```src``` contents to ```~/ros2_ws/src```

* Get dependencies
```bash
cd ~/ros2_ws/src
git clone https://github.com/relffok/ira_laser_tools.git -b ros2-devel
git clone https://github.com/rst-tu-dortmund/teb_local_planner -b foxy-devel
git clone --depth 1 --branch 0.1.2 https://github.com/rst-tu-dortmund/costmap_converter
git clone --depth 1 --branch 3.2.3 https://github.com/IntelRealSense/realsense-ros
git clone --depth 1 --branch 2.8.11 https://github.com/SICKAG/sick_scan_xd
cd ~/ros2_ws \
&& rosdep update \
&& rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```


* Build ***sick_scan*** first in a clean state
```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select sick_scan --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" " -DSCANSEGMENT_XD=0" " -DCMAKE_BUILD_TYPE=Release" --event-handlers console_direct+
```
 
* Build other packages
```bash
cd ~/ros2_ws
colcon build --packages-skip sick_scan --cmake-args -DCMAKE_BUILD_TYPE=Release
```

* Source setup file
```bash
. install/setup.bash
echo 'source $HOME/ros2_ws/install/setup.bash' >> ~/.bashrc
```

## Usage

### NOTE

- Moby behavior: define Moby behavior
- Moby description: define Moby model
- Moby bringup: connect to Moby GRPC server
- Moby mapping: mapping using cartographer or slam toolbox
- Moby navigation: navigation with Moby
- Moby marker detection: detect Aruco marker

### Moby Setting
- Change the Moby config in ```moby-ros2/moby_bringup/param/moby_config.yaml```
* Moby Type
  - Robot type [moby_rp, moby_agri]
* Step IP
  - Ip address of STEP PC
* Sick IP
  - Connect Windows Computer (with Sopas ET installed) to the router.
  - Open Sopas ET and scan devices
  - Change IP address of the TIM lidars
    - front: 192.168.214.10
    - rear: 192.168.214.11
* Realsense
  - Open ```realsense-viewer``` from NUC
  - Check serial numbers for each camera
  - Change serial numbers in config file

### To Start Control Moby
- There are 2 Moby models: moby_rp(default) and moby_agri
```bash
ros2 launch moby_bringup moby_bringup.launch.py moby_type:=<type of moby robot>
```

### To Start Mapping

- Connect to the controller
  - Press **X + Home** button to connect controller to Moby (red led ON)
  - When the controller connected
    - To move: Press **L2 + Left joystick** for moving (non-holonomic)
    - To move: Press **L2 + Right joystick** for moving (holonomic)
    - To change speed: Press **R, R2** to change speed. Maximum 0.8 m/s (linear), 0.8 rad/s (angular)

- Using Cartographer
```bash
ros2 launch moby_bringup moby_bringup.launch.py
ros2 launch moby_mapping cartographer.launch.py
```
- Using Slam Toolbox
```bash
ros2 launch moby_bringup moby_bringup.launch.py
ros2 launch moby_mapping slam_toolbox.launch.py
```

- To see the Map
  - On Rviz press Add (Near bottom left) => Choose Map
- To save the Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/<map_name>
```

- Easy map application - save as <map_name> = default_map and copy to working directory as below
```bash
mkdir ~/map_bak \
&& mv ~/default_map.* ~/map_bak \
&& ros2 run nav2_map_server map_saver_cli -f ~/default_map \
&& cp ~/default_map.* ~/ros2_ws/install/moby_navigation/share/moby_navigation/map/ \
&& cp ~/default_map.* ~/ros2_ws/src/moby-ros2/moby_navigation/map/
```

### To Start Navigation

- Change the map before navigation
  - Copy map file to **moby_navigation/map** (2 files .pgm and .yaml)
  - Modify yaml file: Change the path link to pgm file to: **image: <map_name.pgm>**
  - In folder **moby_navigation/launch** modify **navigation2.launch.py** change map file in **map_dir** variable (line 30).
```bash
ros2 launch moby_bringup moby_bringup.launch.py
ros2 launch moby_navigation navigation2.launch.py
```
- Can tuning navigation parameter in **moby_navigation/param** folder.

### To Detect Marker
- Bring up start front, right and left camera
- To record marker: Start mapping, start moby_marker.launch.py and moby_save_marker.py =>  slowly move robot to generate map and record the markers.
- **moby_marker.launch.py**: start publish marker pose (respect to camera coordinate), id and transform
- **moby_save_marker.py**: save the marker pose **[left camera]** (respect to map coordinate) to SQL database **[This function can only work when a map coordinate exist]**
```bash
ros2 launch moby_marker_detection moby_marker.launch.py
ros2 run moby_marker_detection moby_save_marker.py
```

- **moby_rails_pose_from_markers.py**: to calculate and save rail position to database.
```bash
python3 moby_rails_pose_from_markers.py
```

- **moby_scenario_task.py**: send navigation goal to robot, move robot on the rail.
  - When the function starts, it will ask to confirm. Press Enter to confirm.
```bash
ros2 run moby_marker_detection moby_scenario_task.py
```

### Pairing PG-9023S with External Bluetooth Dongle
* [Prerequisite] Disable onboard bluetooth as described in BIOS Setting section
* After boot, Login and Open Bluetooth setting
* Push and hold **HOME + X** on *PG-9023S* until SEARCH LED blinks rapidly
* Find *PG-9023S* on the bluetooth device list and connect.
