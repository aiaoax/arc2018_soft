
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 &&
sudo apt update &&
sudo apt -y install ros-lunar-desktop-full &&

sudo rosdep init &&
rosdep update &&
echo "source /opt/ros/lunar/setup.bash" >> ~/.bashrc &&

source ~/.bashrc &&
sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential ros-lunar-navigation ros-lunar-viz ros-lunar-gazebo-ros-pkgs ros-lunar-gazebo-ros-control git &&

echo "source ~/arc_osaka/2018/catkin_ws/devel/setup.bash" >> ~/.bashrc &&
echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc &&
echo "export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311" >> ~/.bashrc &&
echo "alias cw='cd ~/arc_osaka/2018/catkin_ws'" >> ~/.bashrc &&
echo "alias cs='cd ~/arc_osaka/2018/catkin_ws/src'" >> ~/.bashrc &&
echo "alias cm='cd ~/arc_osaka/2018/catkin_ws && catkin_make'" >> ~/.bashrc &&

cd &&

git clone https://github.com/aiaoax/arc_osaka.git &&

source ~/.bashrc &&
echo "All OK!"

#mkdir -p ~/arc_osaka/2018/catkin_ws/src &&
#cd ~/arc_osaka/2018/catkin_ws/src &&
#catkin_init_workspace &&
#cd ~/arc_osaka/2018/catkin_ws/ &&
#catkin_make &&
#ls &&
#source devel/setup.bash &&


