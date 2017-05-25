#include <stdio.h>
#include <iostream>
#include <sys/time.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

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
  while(1) {
    cout<<nanosec()<<endl;
  }

  return 0;
}
