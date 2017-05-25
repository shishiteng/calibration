#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

#define PI 3.141592

bool angle(Point2f a,Point2f b)
{
    float ab=a.x*b.x+a.y*b.y;
    float a1=sqrt(a.x*a.x+a.y*a.y);
    float b1=sqrt(b.x*b.x+b.y*b.y);
    float cosr=ab/a1/b1;

    return acos(cosr)*180/PI < 90 ? true:false;
}

bool checkCorner(vector<Point2f> &cornerL,vector<Point2f> &cornerR, int boardWidth, int boardHeight)
{
    // 取出四个角落的角点
    Point2f L1 = cornerL[0];
    Point2f L2 = cornerL[boardWidth-1];
    Point2f L3 = cornerL[boardWidth*(boardHeight-1)];
    Point2f L4 = cornerL[boardWidth*boardHeight-1];

    Point2f R1 = cornerR[0];
    Point2f R2 = cornerR[boardWidth-1];
    Point2f R3 = cornerR[boardWidth*(boardHeight-1)];
    Point2f R4 = cornerR[boardWidth*boardHeight-1];

    Point2f L12 = L2 - L1;
    Point2f L23 = L3 - L2;
    Point2f L34 = L4 - L3;
    Point2f L41 = L1 - L4;
    Point2f R12 = R2 - R1;
    Point2f R23 = R3 - R2;
    Point2f R34 = R4 - R3;
    Point2f R41 = R1 - R4;

    if(angle(L12,R12) && angle(L23,R23) && angle(L34,R34) && angle(L41,R41))
    {
        return true;
    }

    return false;
}

int main(int argc, char* argv[])
{
  int numBoards = 0;
  int board_w = 9;
  int board_h = 7;
  int board_len = 56;//56mm
  Size board_sz = Size(board_w, board_h);
  int board_n = board_w*board_h;

  vector<vector<Point3f> > object_points;
  vector<vector<Point2f> > imagePoints1, imagePoints2;
  vector<Point2f> corners1, corners2;

  vector<Point3f> obj;
  for (int j=0; j<board_n; j++) {
    obj.push_back(Point3f(j/board_w*board_len, j%board_w*board_len, 0.0f));
  }

  Mat img, img1, img2, gray1, gray2;
  VideoCapture capture0;
  int dev = 0;
  if(argc != 1)
    dev = atoi(argv[1]);

  capture0.open(dev);
  if (!(capture0.isOpened()) ) {
    cout << "capture device failed." << endl;
    return 1;
  }

  namedWindow("image1");
  namedWindow("image2");

  int width = 672;
  int height = 376;
  capture0.set(CV_CAP_PROP_FRAME_WIDTH, width * 2);
  capture0.set(CV_CAP_PROP_FRAME_HEIGHT, height);

  int success = 0, k = 0;
  bool found1 = false, found2 = false;

  while (1) {
    capture0 >> img;
    if (img.empty() ) {
      cout<<"grab image failed."<<endl;
      return -1;
    }

    Rect roi_rect0 = cv::Rect(0,0,width,height);
    Rect roi_rect1 = cv::Rect(width,0,width,height);
    img1 = img(roi_rect0);
    img2 = img(roi_rect1);

    imshow("image1", img1);
    imshow("image2", img2);
    cvtColor(img1, gray1, CV_BGR2GRAY);
    cvtColor(img2, gray2, CV_BGR2GRAY);

    k = waitKey(10) & 0xff;
    if (k == 27) {
      break;
    } else if(k != ' ')
      continue;

    found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    

    // 检测左右棋盘格是否角点都一一对应
    if (found1 && found2) {
      bool checkStatus = checkCorner(corners1,corners2,board_w,board_h);
      if( !checkStatus ) 
	continue;
    }

    if (found1) {
      cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray1, board_sz, corners1, found1);
    }

    if (found2) {
      cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      drawChessboardCorners(gray2, board_sz, corners2, found2);
    }
        
    if (found1 && found2) {
      imshow("image1", gray1);
      imshow("image2", gray2);
      char str1[64] = {0};
      char str2[64] = {0};
      sprintf(str1,"./image/cam0/%03d.png",success);
      sprintf(str2,"./image/cam1/%03d.png",success);
      imwrite(str1,gray1);
      imwrite(str2,gray2);
      k = waitKey(0) & 0xff ;
    }

    if (found1 !=0 && found2 != 0) {
      imagePoints1.push_back(corners1);
      imagePoints2.push_back(corners2);
      object_points.push_back(obj);
      success++;
      printf ("Corners stored:%d\n",success);
    }
  }

  destroyAllWindows();
  printf("Starting Calibration\n");
  Mat CM1 = Mat(3, 3, CV_64FC1);
  Mat CM2 = Mat(3, 3, CV_64FC1);
  Mat D1, D2;
  Mat R, T, E, F;

  vector<Mat> rotation_vectors;
  vector<Mat> translation_vectors;
  //stereoCalibrate( objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1,  cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6), int flags=CALIB_FIX_INTRINSIC )

  calibrateCamera(object_points, imagePoints1, img1.size(),  CM1  ,D1, rotation_vectors, translation_vectors);
  printf("cam 0...\n");
  cout<<"matrix1:"<<CM1<<endl;
  cout<<"d1:"<<D1<<endl;
  calibrateCamera(object_points, imagePoints2, img1.size(),  CM2  ,D2, rotation_vectors, translation_vectors);
  printf("cam 1...\n");
  cout<<"matrix1:"<<CM2<<endl;
  cout<<"d1:"<<D2<<endl;
  stereoCalibrate(object_points, imagePoints1, imagePoints2, 
		  CM1, D1, 
		  CM2, D2, 
		  img1.size(), R, T, E, F);
		  //cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
		  //CV_CALIB_SAME_FOCAL_LENGTH);

  FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
  fs1 << "CM1" << CM1;
  fs1 << "CM2" << CM2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "E" << E;
  fs1 << "F" << F;

  printf("Done Calibration\n");
  printf("Starting Rectification\n");

  Mat R1, R2, P1, P2, Q;
  stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");
  printf("Applying Undistort\n");

  Mat map1x, map1y, map2x, map2y;
  Mat imgU1, imgU2;

  initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
  initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);

  printf("Undistort complete\n");

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

    imshow("image1", imgU1);
    imshow("image2", imgU2);

    k = waitKey(5) & 0xff;

    if(k==27) {
      break;
    }
  }

  capture0.release();
  destroyAllWindows();

  return(0);
}
