# SSU RAI&D Lab: TIAGo Documentation

# Instructions:
To run apriltag_scanner.py and human_tag_monitor on TIAGo

go into your workspace that contains both files
$ cd <workspace>

build the files
$ colcon build --packages-select <file> <file>

source the files
$ source install/setup.bash

in another terminal that has been sourced, run apriltag_scanner
and in another terminal run hri_tag_monitor
$ ros2 run <file_location> <filename>

If you want to test on rviz2
activate the hri_body_detector on the robot's terminal
$ pal module start hri_body_detector

open rviz2 and your config if you have one
$ rviz2 -d <config_location>

add the displays for humans and robot
on the humans display set the topic to apriltag_scanner
this will allow the camera to detect both humans and apriltags at the same time
