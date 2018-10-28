if ! pgrep roslaunch > /dev/null; then
    source /opt/ros/lunar/setup.bash
    source ~/arc_osaka/2018/arc_ws/devel/setup.bash
    export ROS_HOSTNAME=ros-raspi.local
    export ROS_MASTER_URI=http://${ROS_HOSTNAME}:11311
    alias cw='cd ~/arc_osaka/2018/arc_ws'
    alias cs='cd ~/arc_osaka/2018/arc_ws/src'
    alias cm='cd ~/arc_osaka/2018/arc_ws && catkin_make'

    #sleep 10s
    echo "starting..." > /usr/local/www/mode.txt
    /opt/ros/lunar/bin/roslaunch tama tama.launch
    
else
    echo "roslaunch killing"
    killall -w roslaunch
fi
