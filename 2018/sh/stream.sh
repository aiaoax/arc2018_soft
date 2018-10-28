#!/bin/bash
 
if pgrep mjpg_streamer > /dev/null
then
  echo "mjpg_streamer already running"
else
  mjpg_streamer -i "/usr/local/lib/input_uvc.so -f 10 -r 320x240 -d /dev/video0 -y -n" -o "/usr/local/lib/output_http.so -w /usr/local/www -p 9000"&
  echo "mjpg_streamer started"
fi
