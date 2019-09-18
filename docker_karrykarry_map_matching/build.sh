#!/bin/bash

path_name=$(pwd)
image_name=${path_name##*/}
docker build -t $image_name:latest .
