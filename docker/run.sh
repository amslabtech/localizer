#!/bin/bash

path_name=$(pwd)
image_name=${path_name##*/}

xhost +
nvidia-docker run -it --rm \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--net=host \
	karrykarry_ndt_localizer:latest
	# $image_name:latest
