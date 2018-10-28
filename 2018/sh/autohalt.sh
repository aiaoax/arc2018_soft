#!/bin/bash
rosrun=false
res0=$(gpio -1 read 40)
res=${res0}
while [ ${res} = ${res0} ]
do
    if ! pgrep roslaunch > /dev/null; then
        if [ rosrun ]; then
            echo "STOP" > /usr/local/www/mode.txt
        fi
        rosrun=false
    else
        rosrun=true
    fi
    sleep 2s
    res=$(gpio -1 read 40)
    #echo ${res}
done
echo "shutdown"
echo "SHUTDOWN" > /usr/local/www/mode.txt
#shutdown -h now
