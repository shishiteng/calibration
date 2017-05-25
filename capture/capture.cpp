#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

#define WIDTH  1280
#define HEIGHT 720


unsigned long long micros() 
{
  struct timeval dwTime;
  gettimeofday(&dwTime, NULL);
  unsigned long long us = (unsigned long long)(1000000 * dwTime.tv_sec + dwTime.tv_usec);

  return us;
}

unsigned long long nanosec()
{
  struct timespec time_start={0, 0},time_end={0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);    //有4组稍微大于7或者小于3的                                                                                                                                    
  //clock_gettime(CLOCK_MONOTONIC, &time_start); //有很多组间隔小于1的                                                                                                                                          
  //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time_start); //很多组大于10的                                                                                                                                      
  //clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time_start);                                                                                                                                                        
  unsigned long long ns = (unsigned long long)(time_start.tv_sec * 1000000000 + time_start.tv_nsec);
 
  return ns;
}

int main(int argc, char ** argv)
{
  VideoCapture capture0;
  int dev = 0;
  int width;
  int height;

  if(argc != 4) {
    cout << "para error:capture [dev] [width] [height]"<<endl;
    return -1;
  }

  dev = atoi(argv[1]);
  width = atoi(argv[2]);
  height = atoi(argv[3]);

  capture0.open(dev);
  if (!(capture0.isOpened()) ) {
    cout << "capture device failed." << endl;
    return 1;
  }

  capture0.set(CV_CAP_PROP_FRAME_WIDTH, width);
  capture0.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  capture0.set(CV_CAP_PROP_BUFFER_SIZE, 3);

  int delay = 1;
  int n = 0;
  Mat frame;
  for (;;) {
    capture0 >> frame;
    unsigned long long timestamp = nanosec();

    if (frame.empty()) {
      cout <<" get frame failed."<<endl;
      break;
    }
    
    imshow("cap",frame);
    n++;

    int key = waitKey(delay) & 0xff;
    if(key == ' ') {
      char str[12] = {0};
      sprintf(str,"data/%llu.png",timestamp);
      imwrite(str,frame);
    }
    if(key == 'q')
      break;
  }

  capture0.release();
  cout << "capture over."<<endl;
  return 0;
}

