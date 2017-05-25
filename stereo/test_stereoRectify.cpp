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
  Mat CM1,CM2,D1,D2,R,T;
  FileStorage fs("in.yml", FileStorage::READ);
  if(!fs.isOpened())
    {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
    }


  cerr<<".";
  fs["CM1"] >> CM1;
  cerr<<".";
  fs["CM2"] >> CM2;
  cerr<<".";
  fs["D1"] >> D1;
  cerr<<".";
  fs["D2"] >> D2;
  cerr<<".";
  fs["R"] >> R;
  cerr<<".";
  fs["T"] >> T;
  cerr<<".";

  printf("Starting Rectification\n");

  FileStorage fs1("out.yml", FileStorage::WRITE);
  Mat img1(720,1280,CV_32FC1);
  imshow("1",img1);
  waitKey(0);
  Mat R1, R2, P1, P2, Q;
  stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
  fs1 << "CM1" << CM1;
  fs1 << "CM2" << CM2;
  fs1 << "D1" << D1;
  fs1 << "D2" << D2;
  fs1 << "R" << R;
  fs1 << "T" << T;
  fs1 << "R1" << R1;
  fs1 << "R2" << R2;
  fs1 << "P1" << P1;
  fs1 << "P2" << P2;
  fs1 << "Q" << Q;

  printf("Done Rectification\n");

  return 0;
}
