#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;


#define WIDTH 1280
#define HEIGHT 720

int main()
{
  double intrinsicLeft[] = {
    693.96186, 0,          668.16147, 
    0,         689.00037,  360.17446,
    0,         0,          1
  };
  double distortionLeft[] = {-0.17075, 0.01759, -0.00015, 0.00038, 0};
  double intrinsicRight[] = {
    692.63093, 0,            681.73066,
    0,         687.52713,    343.05926,
    0,         0,            1
  };
  double distortionRight[] = {-0.17419, 0.02076, -0.00005, 0.00008, 0};
  double R_matrix_arr[] = {
    0.999978, -0.000973, 0.006479,
    0.000966, 0.9999989, 0.001143, 
    -0.006481, -0.001136, 0.99998
  };
  double T_arr[] = { -0.119051, -0.000032, -0.00184};


  CvMat intrinsic_left,
    intrinsic_right,
    distortion_left,
    distortion_right,
    R, T;
  CvMat R_matrix;

  cvInitMatHeader(&intrinsic_left, 3, 3, CV_64F, intrinsicLeft,CV_AUTOSTEP);
  cvInitMatHeader(&intrinsic_right, 3, 3, CV_64F, intrinsicRight,CV_AUTOSTEP);
  cvInitMatHeader(&distortion_left, 1, 5, CV_64F, distortionLeft,CV_AUTOSTEP);
  cvInitMatHeader(&distortion_right, 1, 5, CV_64F, distortionRight,CV_AUTOSTEP);
  cvInitMatHeader(&R, 3, 3, CV_64F, R_matrix_arr,CV_AUTOSTEP);
  cvInitMatHeader(&T, 3, 1, CV_64F, T_arr,CV_AUTOSTEP);
  cvInitMatHeader(&R_matrix, 3, 3, CV_64F, R_matrix_arr,CV_AUTOSTEP);
  // 将向量转换成旋转矩阵
  //cvRodrigues2(&R, &R_matrix,0);

  // 将上面的CvMat转换成Mat
  Mat cameraMatrixL = cvarrToMat(&intrinsic_left);
  Mat cameraMatrixR = cvarrToMat(&intrinsic_right);
  Mat distortionCoefficientsL = cvarrToMat(&distortion_left);
  Mat distortionCoefficientsR = cvarrToMat(&distortion_right);
  Mat rotation = cvarrToMat(&R_matrix);
  Mat translation = cvarrToMat(&T);

  Size imageSize = Size(WIDTH,HEIGHT);

  // 下面开始通过双目参数计算出remap
  Mat mX1 = cv::Mat(imageSize, CV_32FC1);
  Mat mY1 = cv::Mat(imageSize, CV_32FC1);
  Mat mX2 = cv::Mat(imageSize, CV_32FC1);
  Mat mY2 = cv::Mat(imageSize, CV_32FC1);

  cv::Mat R1, R2, P1, P2, Q;
  cv::Rect roi1, roi2;
  double alpha = -1;

  //执行双目校正
  stereoRectify(
		cameraMatrixL,
		distortionCoefficientsL,
		cameraMatrixR,
		distortionCoefficientsR,
		imageSize,
		rotation,
		translation,
		R1,R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY,
		alpha,
		imageSize,
		&roi1, &roi2);

  //生成图像校正所需的像素映射矩阵
  initUndistortRectifyMap(
			  cameraMatrixL,
			  distortionCoefficientsL,
			  R1, P1,
			  imageSize,
			  CV_16SC2,
			  mX1, mY1);

  initUndistortRectifyMap(
			  cameraMatrixR,
			  distortionCoefficientsR,
			  R2, P2,
			  imageSize,
			  CV_16SC2,
			  mX2, mY2);

  // 写入文件
  cv::FileStorage fs("./calib_paras_zed.xml", cv::FileStorage::WRITE);

  if( fs.isOpened() )
    {
      time_t rawtime;
      time(&rawtime);
      fs << "calibrationDate" << asctime(localtime(&rawtime));

      fs << "imageSize" << "[" << imageSize.width << imageSize.height << "]";

      fs << "leftCameraMatrix"			<< cameraMatrixL;
      fs << "leftDistortCoefficients"		<< distortionCoefficientsL;
      fs << "rightCameraMatrix"			<< cameraMatrixR;
      fs << "rightDistortCoefficients"	<< distortionCoefficientsR;
      fs << "rotationMatrix"				<< rotation;
      fs << "translationVector"			<< translation;

      fs << "rectifyMethod" << "BOUGUET";
      fs << "leftValidArea" << "[:"
	 << roi1.x << roi1.y
	 << roi1.width << roi1.height << "]";
      fs << "rightValidArea" << "[:"
	 << roi2.x << roi2.y
	 << roi2.width << roi2.height << "]";
      fs << "QMatrix" << Q;

      fs << "remapX1" << mX1;
      fs << "remapY1" << mY1;
      fs << "remapX2" << mX2;
      fs << "remapY2" << mY2;

      fs.release();
    }

}
