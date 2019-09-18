#!/bin/bash
roscore &
rosrun rviz rviz -d ../config/local.rviz
