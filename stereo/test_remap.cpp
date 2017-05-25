#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{
  VideoCapture capture0;
  int dev = 0;
  if(argc != 1)
    dev = atoi(argv[1]);

  capture0.open(dev);
  if (!(capture0.isOpened()) ) {
    cout << "capture device failed." << endl;
    return 1;
  }

  int width = 1280;
  int height = 720;
  Mat img(width*2,height,CV_32FC1);
  capture0.set(CV_CAP_PROP_FRAME_WIDTH, width * 2);
  capture0.set(CV_CAP_PROP_FRAME_HEIGHT, height);

  double cm1[] = {
    693.96186, 0,          668.16147, 
    0,         689.00037,  360.17446,
    0,         0,          1
  };
  double d1[] = {-0.17075, 0.01759, -0.00015, 0.00038, 0};
  double cm2[] = {
    692.63093, 0,            681.73066,
    0,         687.52713,    343.05926,
    0,         0,            1
  };
  double d2[] = {-0.17419, 0.02076, -0.00005, 0.00008, 0};
  double r[] = {
    0.999978, -0.000973, 0.006479,
    0.000966, 0.9999989, 0.001143, 
    -0.006481, -0.001136, 0.99998
  };
  double t[] = { -0.119051, -0.000032, -0.00184};

  Mat CM1(3,3,CV_64FC1,cm1);
  Mat D1(1,5,CV_64FC1,d1);
  Mat CM2(3,3,CV_64FC1,cm2);
  Mat D2(1,5,CV_64FC1,d2);
  Mat R(3,3,CV_64FC1,r);
  Mat T(3,1,CV_64FC1,t);

  printf("start.\n");
  Mat R1, R2, P1, P2, Q;
  stereoRectify(CM1, D1, CM2, D2, Size(width,height), R, T, R1, R2, P1, P2, Q);
  fprintf(stderr,"123\n");
  printf("Done Rectification\n");
  printf("Applying Undistort\n");
  std::cout<<"R1\n"<<R1<<endl;
  std::cout<<"R2\n"<<R2<<endl;
  std::cout<<"P1\n"<<P1<<endl;
  std::cout<<"P2\n"<<P2<<endl;

  Mat map1x, map1y, map2x, map2y;
  Mat imgU1, imgU2;

  initUndistortRectifyMap(CM1, D1, R1, P1, Size(width,height), CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(CM2, D2, R2, P2, Size(width,height), CV_32FC1, map2x, map2y);

  printf("Undistort complete\n");

  Mat img1,img2;
  while(1) {    
    capture0 >> img;
    if(img.empty())
      break;
    Rect roi_rect0 = cv::Rect(0,0,width,height);
    Rect roi_rect1 = cv::Rect(width,0,width,height);
    img1 = img(roi_rect0);
    img2 = img(roi_rect1);

    remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

    imshow("image1_o", img1);
    imshow("image1", imgU1);
    imshow("image2_o", img2);
    imshow("image2", imgU2);

    int k = waitKey(5) & 0xff;

    if(k==27) {
      break;
    }
  }

  capture0.release();

  return(0);
}
