#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

int main(int argc, char** argv)
{
  //Left Camera
  cv::Mat leftDistCoeffs;
  cv::Mat leftCameraMatrix;
  cv::FileStorage fs("../left_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> leftDistCoeffs;
  fs["Intrinsic"] >> leftCameraMatrix;
  fs.release();

  //Right Camera
  cv::Mat rightDistCoeffs;
  cv::Mat rightCameraMatrix;
  fs = cv::FileStorage("../right_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> rightDistCoeffs;
  fs["Intrinsic"] >> rightCameraMatrix;
  fs.release();

  //Stereo Params
  cv::Mat R;
  cv::Mat T;
  cv::Mat E;
  cv::Mat F;
  fs = cv::FileStorage("../stereo_params.yaml",cv::FileStorage::READ);
  fs["R"] >> R;
  fs["T"] >> T;
  fs["E"] >> E;
  fs["F"] >> F;
  fs.release();

  cv::Mat leftImage = cv::imread("../combinedCalibrate/aL00.bmp");
  cv::Mat rightImage = cv::imread("../combinedCalibrate/aR00.bmp");
  cv::Mat leftChangedImage;
  cv::Mat rightChangedImage;

  cv::undistort(leftImage, leftChangedImage, leftCameraMatrix, leftDistCoeffs);
  cv::undistort(rightImage, rightChangedImage, rightCameraMatrix, rightDistCoeffs);


  int board_height = 7;
  int board_width = 10;
  cv::Size board_size = cv::Size(board_width, board_height);

  std::vector<cv::Point2f> leftCorners;
  std::vector<cv::Point2f> rightCorners;

  cv::findChessboardCorners(leftChangedImage, board_size, leftCorners, CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE);
  cv::findChessboardCorners(rightChangedImage, board_size, rightCorners, CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE);

  std::vector<cv::Point2f> leftPoints{leftCorners[3],leftCorners[10],leftCorners[30]};
  std::vector<cv::Point2f> rightPoints{rightCorners[40],rightCorners[56],rightCorners[63]};

  // std::cout << "here" << std::endl;
  // std::cout << leftCorners << std::endl;
  // std::cout << leftCorners[3] << std::endl;
  // std::cout << leftPoints[0] << std::endl;

  cv::Mat rlines;
  cv::Mat llines;
  cv::computeCorrespondEpilines(leftPoints,1,F,rlines);
  cv::computeCorrespondEpilines(rightPoints,2,F,llines);
  // std::cout << lines << std::endl;
  double x1 = 10.0;
  double x2 = 620.0;
  for(int i=0;i<3;i++)
  {
    double ry1= (-rlines.at<float>(i,0)*x1-rlines.at<float>(i,2))/rlines.at<float>(i,1);
    double ry2= (-rlines.at<float>(i,0)*x2-rlines.at<float>(i,2))/rlines.at<float>(i,1);

    double ly1= (-llines.at<float>(i,0)*x1-llines.at<float>(i,2))/llines.at<float>(i,1);
    double ly2= (-llines.at<float>(i,0)*x2-llines.at<float>(i,2))/llines.at<float>(i,1);

    // std::cout << ry1 << " " << ry2 << std::endl;
    cv::line(leftChangedImage, cv::Point(x1,ly1), cv::Point(x2,ly2), cv::Scalar(0,255,0), 1);
    cv::line(rightChangedImage, cv::Point(x1,ry1), cv::Point(x2,ry2), cv::Scalar(0,255,0), 1);
    cv::circle(leftChangedImage, leftPoints[i], 5, cv::Scalar(0,0,255), 1);
    cv::circle(rightChangedImage, rightPoints[i], 5, cv::Scalar(0,0,255), 1);
  }

  cv::imshow("Left", leftChangedImage);
  cv::imshow("Right", rightChangedImage);
  cv::waitKey(0);
  cv::imwrite("../leftEpilines.png", leftChangedImage);
  cv::imwrite("../rightEpilines.png", rightChangedImage);

  return 0;
}
