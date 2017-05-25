#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{ 
  /************************************************************************  
           从摄像机中读取多幅图像,从中提取出角点，然后对角点进行亚像素精确化 
  *************************************************************************/ 
  int image_count =  3;                    /****    图像数量     ****/  
  Mat frame;
  Size image_size;                         /****     图像的尺寸      ****/   
  Size board_size = Size(9,7);            /****    定标板上每行、列的角点数       ****/  
  vector<Point2f> corners;                  /****    缓存每幅图像上检测到的角点       ****/
  vector<vector<Point2f> >  corners_Seq;    /****  保存检测到的所有角点       ****/   
  ofstream fout("calibration_result.txt");  /**    保存定标结果的文件     **/

  int id = 0;
  if(argc == 2)
    id = atoi(argv[1]);

  VideoCapture cap(id);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
  if(!cap.isOpened()){
    std::cout<<"打开摄像头失败，退出";
    exit(-1);
  }
  namedWindow("Calibration");
  namedWindow("corners");

  int count = 0,n=0;
  stringstream tempname;
  string filename;
  int key;
  string msg;
  int baseLine;
  Size textSize;
  int k;

  int aa = 0;
  while(n < image_count ) {
    cap>>frame;
    imshow("Calibration",frame);
    int key = waitKey(10);
    key = key & 0xff;
    if(key == 'q')
      break;

    //3秒采一张
    if((aa++)%90 != 0)
      continue;
    
    image_size = frame.size();
    /* 提取角点 */   
    Mat imageGray;
    cvtColor(frame, imageGray , CV_RGB2GRAY);
    bool patternfound = findChessboardCorners(frame, board_size, corners,CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );
    if (patternfound) {    
      n++;
      tempname<<n;
      tempname>>filename;
      filename+=".jpg";
      filename = "./image/"+filename;
      /* 亚像素精确化 */
      cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      count += corners.size();
      corners_Seq.push_back(corners);
      imwrite(filename,frame);
      drawChessboardCorners(frame, board_size, corners,  patternfound); 
      tempname.clear();
      filename.clear();
      imshow("corners",frame);
      waitKey(1);
    }
  }


  std::cout<<"角点提取完成\n"; 

  /************************************************************************  
           摄像机定标  
  *************************************************************************/   
  std::cout<<"开始定标………………"<<endl;   
  Size square_size = Size(25,25);                                      /**** 实际测量得到的定标板上每个棋盘格的大小   ****/  
  vector<vector<Point3f> >  object_Points;                                      /****  保存定标板上角点的三维坐标   ****/

  Mat image_points = Mat(1, count , CV_32FC2, Scalar::all(0));          /*****   保存提取的所有角点   *****/   
  vector<int>  point_counts;                                          /*****    每幅图像中角点的数量    ****/   
  Mat intrinsic_matrix = Mat(3,3, CV_32FC1, Scalar::all(0));                /*****    摄像机内参数矩阵    ****/   
  Mat distortion_coeffs = Mat(1,5, CV_32FC1, Scalar::all(0));            /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */ 
  vector<Mat> rotation_vectors;                                      /* 每幅图像的旋转向量 */  
  vector<Mat> translation_vectors;                                  /* 每幅图像的平移向量 */  
     
  /* 初始化定标板上角点的三维坐标 */     
  for (int t=0;t<image_count;t++) 
    {   
      vector<Point3f> tempPointSet;
      for (int i=0;i<board_size.height;i++) 
	{   
	  for (int j=0;j<board_size.width;j++) 
	    {   
	      /* 假设定标板放在世界坐标系中z=0的平面上 */   
	      Point3f tempPoint;
	      tempPoint.x = i*square_size.width;
	      tempPoint.y = j*square_size.height;
	      tempPoint.z = 0;
	      tempPointSet.push_back(tempPoint);
	    }   
	}
      object_Points.push_back(tempPointSet);
    }   
   
  /* 初始化每幅图像中的角点数，这里我们假设每幅图像中都可以看到完整的定标板 */   
  for (int i=0; i< image_count; i++)   
    {
      point_counts.push_back(board_size.width*board_size.height);   
    }
       
  /* 开始定标 */   
  calibrateCamera(object_Points, corners_Seq, image_size,  intrinsic_matrix  ,distortion_coeffs, rotation_vectors, translation_vectors);   
  std::cout<<"定标完成！\n";   
       
  /************************************************************************  
           对定标结果进行评价  
  *************************************************************************/   
  std::cout<<"开始评价定标结果………………"<<endl;   
  double total_err = 0.0;                   /* 所有图像的平均误差的总和 */   
  double err = 0.0;                        /* 每幅图像的平均误差 */   
  vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/   
   
  std::cout<<"每幅图像的定标误差："<<endl;   
  fout<<"每幅图像的定标误差："<<endl<<endl;   
  for (int i=0;  i<image_count;  i++) 
    {
      vector<Point3f> tempPointSet = object_Points[i];
      /****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
      projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);
      /* 计算新的投影点和旧的投影点之间的误差*/  
      vector<Point2f> tempImagePoint = corners_Seq[i];
      Mat tempImagePointMat = Mat(1,tempImagePoint.size(),CV_32FC2);
      Mat image_points2Mat = Mat(1,image_points2.size(), CV_32FC2);
      for (int j = 0 ; j < tempImagePoint.size(); j++)
	{
	  image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
	  tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
	}
      err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
      total_err += err/=  pow(point_counts[i],0.5);
      std::cout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;   
      fout<<"第"<<i+1<<"幅图像的平均误差："<<err<<"像素"<<endl;   
    }   
  std::cout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl;   
  fout<<"总体平均误差："<<total_err/image_count<<"像素"<<endl<<endl;   
  std::cout<<"评价完成！"<<endl;   
   
  /************************************************************************  
           保存定标结果  
  *************************************************************************/   
  std::cout<<"开始保存定标结果………………"<<endl;       
  Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */   
       
  fout<<"相机内参数矩阵："<<endl;   
  fout<<intrinsic_matrix<<endl<<endl;   
  fout<<"畸变系数：\n";   
  fout<<distortion_coeffs<<endl<<endl<<endl;   
  for (int i=0; i<image_count; i++) 
    { 
      fout<<"第"<<i+1<<"幅图像的旋转向量："<<endl;   
      fout<<rotation_vectors[i]<<endl;   
  
      /* 将旋转向量转换为相对应的旋转矩阵 */   
      Rodrigues(rotation_vectors[i],rotation_matrix);   
      fout<<"第"<<i+1<<"幅图像的旋转矩阵："<<endl;   
      fout<<rotation_matrix<<endl;   
      fout<<"第"<<i+1<<"幅图像的平移向量："<<endl;   
      fout<<translation_vectors[i]<<endl<<endl;   
    }   
  std::cout<<"完成保存"<<endl; 
  fout<<endl;

  /************************************************************************  
           显示定标结果  
  *************************************************************************/
  Mat mapx = Mat(image_size,CV_32FC1);
  Mat mapy = Mat(image_size,CV_32FC1);
  Mat R = Mat::eye(3,3,CV_32F);
  std::cout<<"保存矫正图像"<<endl;
  string imageFileName;
  std::stringstream StrStm;
  for (int i = 0 ; i != image_count ; i++)
    {
      std::cout<<"Frame #"<<i+1<<"..."<<endl;
      Mat newCameraMatrix = Mat(3,3,CV_32FC1,Scalar::all(0));
      initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
      StrStm.clear();
      imageFileName.clear();
      StrStm<<i+1;
      StrStm>>imageFileName;
      imageFileName += ".jpg";
      imageFileName = "./image/"+imageFileName;
      Mat t = imread(imageFileName);
      Mat newimage = t.clone();
      cv::remap(t,newimage,mapx, mapy, INTER_LINEAR);
      StrStm.clear();
      imageFileName.clear();
      StrStm<<i+1;
      StrStm>>imageFileName;
      imageFileName += "_d.jpg";
      imageFileName = "./image/"+imageFileName;
      imwrite(imageFileName,newimage);
    }
  std::cout<<"保存结束"<<endl;

  return 0;
}
