#! /bin/bash

g++ ./calibration.cpp -o calibration -lopencv_highgui -lopencv_imgproc -lopencv_core -lopencv_video -lopencv_calib3d
g++ ./calibration_zed_mono.cpp -o calibration_zed_mono -lopencv_highgui -lopencv_imgproc -lopencv_core -lopencv_video -lopencv_calib3d
