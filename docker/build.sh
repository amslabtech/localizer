#!/bin/bash

path_name=$(pwd)
image_name=${path_name##*/}
# docker build -t $image_name:latest .
docker build -t karrykarry_ndt_localizer:latest .
