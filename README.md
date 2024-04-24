# ROS 2 Iron

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools

sudo apt update

sudo apt upgrade

sudo apt install ros-iron-desktop

echo 'source  /opt/ros/iron/setup.bash' >> ~/.bashrc

#ROS WS

sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

mkdir -p ~/ws_moveit/src

cd ~/ws_moveit
colcon build

echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc


# MoveIT2

sudo apt install ros-iron-moveit

sudo apt install ros-iron-ros2-control

sudo apt install ros-iron-ros2-controllers

sudo apt install ros-iron-moveit-planners-chomp

sudo apt-get install ros-iron-imu-tools

sudo apt-get install can-utils

git clone …
colcon build —-packages-select ruka

# Run

ros2 launch ruka with_cont.launch.py  //на распбери
ros2 launch ruka without_cont.launch.py 
