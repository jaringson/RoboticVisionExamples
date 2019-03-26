#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <iostream>

int main(int argc, char** argv)
{
  // std::cout << CV_MAJOR_VERSION << std::endl;
  double square_size = 3.88636;
  int board_height = 7;
  int board_width = 10;
  cv::Size board_size(board_width, board_height);

  //TO READ
  cv::Mat leftDistCoeffs;
  cv::Mat leftCameraMatrix;
  cv::FileStorage fs("../left_params.yaml",cv::FileStorage::READ);
  fs["Intrinsic"] >> leftCameraMatrix;
  fs["Distortion"] >> leftDistCoeffs;
  fs.release();

  //TO READ
  cv::Mat rightDistCoeffs;
  cv::Mat rightCameraMatrix;
  fs.open("../right_params.yaml",cv::FileStorage::READ);
  fs["Intrinsic"] >> rightCameraMatrix;
  fs["Distortion"] >> rightDistCoeffs;
  fs.release();

  // cv::String path("../combinedCalibrate/aL*.bmp");
  // std::vector<cv::String> left_fn;
  //
  // cv::glob(path,left_fn,true);
  //
  // path = cv::String("../combinedCalibrate/aR*.bmp");
  // std::vector<cv::String> right_fn;
  //
  // cv::glob(path,right_fn,true);

  bool foundL;
  bool foundR;
  cv::Mat imL;
  cv::Mat grayL;
  cv::Mat imR;
  cv::Mat grayR;
  std::string leftFile;
  std::string rightFile;

  std::vector<std::vector<cv::Point2f>> leftImagePoints;
  std::vector<std::vector<cv::Point2f>> rightImagePoints;
  std::vector<std::vector<cv::Point3f>> objectPoints;

  std::vector<cv::Point3f> obj;
  for (int i = 0; i < board_height; i++)
  {
    for (int j = 0; j < board_width; j++)
    {
      obj.push_back(cv::Point3f(j*square_size, i*square_size, 0.0));
    }
  }



  int flags(cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001);

  for (int k=0; k<100; k++)
  {
      if(k<=9)
        leftFile = "../combinedCalibrate/aL0"+std::to_string(k)+".bmp";//left_fn[k/2];
      else
        leftFile = "../combinedCalibrate/aL"+std::to_string(k)+".bmp";

      if(k<=9)
        rightFile = "../combinedCalibrate/aR0"+std::to_string(k)+".bmp";//right_fn[k/2];
      else
        rightFile = "../combinedCalibrate/aR"+std::to_string(k)+".bmp";

    imL = cv::imread(leftFile);
    cv::cvtColor(imL, grayL, cv::COLOR_RGB2GRAY);
    imR = cv::imread(rightFile);
    cv::cvtColor(imR, grayR, cv::COLOR_RGB2GRAY);
    std::vector<cv::Point2f> cornersL;
    std::vector<cv::Point2f> cornersR;
    foundL = cv::findChessboardCorners(grayL, board_size, cornersL, flags);
    foundR = cv::findChessboardCorners(grayR, board_size, cornersR, flags);

    if(foundL && foundR)
    {
      cv::cornerSubPix(grayL, cornersL, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      cv::cornerSubPix(grayR, cornersR, cv::Size(5, 5), cv::Size(-1, -1), criteria);

      objectPoints.push_back(obj);
      leftImagePoints.push_back(cornersL);
      rightImagePoints.push_back(cornersR);

    }
    else
      std::cout << "Can't find Chessboard Corners: " << leftFile << " " << rightFile << std::endl;

  }
  // std::cout << objectPoints.size() << "\n";
  // std::cout << leftImagePoints.size() << "\n";
  // std::cout << rightImagePoints.size() << "\n";
  cv::Mat R;
  cv::Mat T;
  cv::Mat E;
  cv::Mat F;

  cv::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints,
    leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs,
    grayL.size(),
    R, T, E, F,
    cv::CALIB_FIX_INTRINSIC, criteria);

  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;
  std::cout << "E: " << E << std::endl;
  std::cout << "F: " << F << std::endl;

  cv::FileStorage fs_out("../stereo_params.yaml",cv::FileStorage::WRITE);
  fs_out << "R" << R;
  fs_out << "T" << T;
  fs_out << "E" << E;
  fs_out << "F" << F;
  fs_out.release();


  return 0;
}
