#!/bin/sh

g++ ./stereo_calibrate.cpp -o ./stereo_calibrate -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_contrib
