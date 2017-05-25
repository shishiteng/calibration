#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace cv;
using namespace std;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
int main(int argc, char **argv)
{ 
  int id = 0;
  if(argc != 3) {
    printf("参数错误,caibration_dataset [dataset] [image count]\n");
    return -1;
  }

  /************************************************************************  
           从摄像机中读取多幅图像,从中提取出角点，然后对角点进行亚像素精确化 
  *************************************************************************/ 
  Mat frame;
  Size image_size;                         /****     图像的尺寸      ****/   
  Size board_size = Size(9,7);            /****    定标板上每行、列的角点数       ****/  
  vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
  vector<vector<Point2f> >  corners_Seq;    /****  保存检测到的所有角点       ****/   

  int input_count = atoi(argv[2]);
  char *path = argv[1];

  int baseLine;
  Size textSize;
  int k;

  int n = 1;//image count
  int count = 0;//corners count
  int image_count = 0;
  while(n < input_count ) {
    char str[64] = {0};
    sprintf(str,"%s/%d.bmp",path,n);
    frame = imread(str);
    image_size = frame.size();
    /* 提取角点 */   
    Mat imageGray;
    if(3 == frame.channels())
      cvtColor(frame, imageGray , CV_RGB2GRAY);
    else 
      imageGray = frame.clone();
    bool bfound = findChessboardCorners(frame, board_size, corners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );
    if (bfound) {    
      image_count++;
      /* 亚像素精确化 */
      cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      count += corners.size();
      corners_Seq.push_back(corners);
    }
    else{
      printf("detect failed:%s\n",str);
    }
    n++;
  }			
  cout<<"角点提取完成\n"; 


  /************************************************************************  
           摄像机定标  
  *************************************************************************/   
  std::cout<<"开始定标………………"<<endl;   
  Size square_size = Size(56,56);                                      /**** 实际测量得到的定标板上每个棋盘格的大小   ****/  
  vector< vector<Point3f> >  object_Points;                                      /****  保存定标板上角点的三维坐标   ****/

  Mat image_points = Mat(1, count , CV_32FC2, Scalar::all(0));          /*****   保存提取的所有角点   *****/   
  vector<int>  point_counts;                                          /*****    每幅图像中角点的数量    ****/   
  Mat intrinsic_matrix = Mat(3,3, CV_32FC1, Scalar::all(0));                /*****    摄像机内参数矩阵    ****/   
  Mat distortion_coeffs = Mat(1,5, CV_32FC1, Scalar::all(0));            /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */ 
  vector<Mat> rotation_vectors;                                      /* 每幅图像的旋转向量 */  
  vector<Mat> translation_vectors;                                  /* 每幅图像的平移向量 */  
     
  /* 初始化定标板上角点的三维坐标 */     
  for (int t = 0; t < image_count; t++) {   
    vector<Point3f> tempPointSet;
    for (int i = 0; i < board_size.height; i++) {   
      for (int j = 0; j < board_size.width; j++) {   
	/* 假设定标板放在世界坐标系中z=0的平面上 */   
	Point3f tempPoint;
	tempPoint.x = i * square_size.width;
	tempPoint.y = j * square_size.height;
	tempPoint.z = 0;
	tempPointSet.push_back(tempPoint);
      }   
    }
    object_Points.push_back(tempPointSet);
    fprintf(stderr,".");
  }   
   
  /* 初始化每幅图像中的角点数，这里我们假设每幅图像中都可以看到完整的定标板 */   
  for (int i=0; i< image_count; i++) {
    point_counts.push_back(board_size.width * board_size.height);   
  }
       
  /* 开始定标 */   
  calibrateCamera(object_Points, corners_Seq, image_size,  intrinsic_matrix  ,distortion_coeffs, rotation_vectors, translation_vectors);   
  cout<<endl<<"定标完成！\n";
       
  /************************************************************************  
           对定标结果进行评价  
  *************************************************************************/   
  std::cout<<"开始评价定标结果………………"<<endl;   
  double total_err = 0.0;                   /* 所有图像的平均误差的总和 */   
  double err = 0.0;                        /* 每幅图像的平均误差 */   
  vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/   
   
  cout<<"每幅图像的定标误差："<<endl;   
  for (int i = 0;  i<image_count;  i++) {
    vector<Point3f> tempPointSet = object_Points[i];
    /****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
    projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
    /* 计算新的投影点和旧的投影点之间的误差*/  
    vector<Point2f> tempImagePoint = corners_Seq[i];
    Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
    Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
    for (int j = 0 ; j < tempImagePoint.size(); j++) {
      image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
      tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
    }
    err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
    total_err += err/=  pow(point_counts[i],0.5);
    cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;   
  }   
  cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;   
  cerr<<"评价完成！"<<endl;   
   
  /************************************************************************  
           保存定标结果  
  *************************************************************************/   
  cout<<"开始保存定标结果………………"<<endl;       
  Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */   
       
  cout<<"相机内参数矩阵："<<endl;   
  cout<<intrinsic_matrix<<endl<<endl;   
  cout<<"畸变系数：\n";   
  cout<<distortion_coeffs<<endl<<endl<<endl;   
  for (int i=0 ; i<image_count; i++) { 
    cout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;   
    cout<<rotation_vectors[i]<<endl;   
  
    /* 将旋转向量转换为相对应的旋转矩阵 */   
    Rodrigues(rotation_vectors[i],rotation_matrix);   
    cout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;   
    cout<<rotation_matrix<<endl;   
    cout<<"第"<<i+1<<"幅图像的平移向量："<<endl;   
    cout<<translation_vectors[i]<<endl<<endl;   
  }   
  cout<<"完成保存"<<endl; 

  /************************************************************************  
           显示定标结果  
  *************************************************************************/
  Mat mapx = Mat(image_size,CV_32FC1);
  Mat mapy = Mat(image_size,CV_32FC1);
  Mat R = Mat::eye(3,3,CV_32F);
  initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);

  cout<<"保存矫正图像"<<endl;
  for (int i = 1 ; i < input_count ; i++) {
    char str[64]={0};
    sprintf(str,"%s/%d.bmp",path,i);
    cout<<"image:"<<str<<"..."<<endl;
    Mat t = imread(str);
    Mat newimage = t;
    remap(t,newimage,mapx, mapy, INTER_LINEAR);
    sprintf(str,"./image/_%06d.png",i);
    imwrite(str,newimage);
  }

  cout<<"保存结束\n\n"<<endl;
  cout<<"intrinsic matrix:\n"<<intrinsic_matrix<<endl;
  cout<<"distortion ceoffs:\n"<<distortion_coeffs<<endl<<endl;

  printf("Camera.fx: %.6f\n",intrinsic_matrix.at<double>(0,0));
  printf("Camera.fy: %.6f\n",intrinsic_matrix.at<double>(1,1));
  printf("Camera.cx: %.6f\n",intrinsic_matrix.at<double>(0,2));
  printf("Camera.cy: %.6f\n",intrinsic_matrix.at<double>(1,2));
  printf("Camera.k1: %.6f\n",distortion_coeffs.at<double>(0));
  printf("Camera.k2: %.6f\n",distortion_coeffs.at<double>(1));
  printf("Camera.p1: %.6f\n",distortion_coeffs.at<double>(2));
  printf("Camera.p2: %.6f\n",distortion_coeffs.at<double>(3));
  printf("Camera.k3: %.6f\n",distortion_coeffs.at<double>(4));

  cout<<endl;

  return 0;
}
