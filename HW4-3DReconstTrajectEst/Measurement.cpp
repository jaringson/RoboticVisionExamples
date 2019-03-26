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
  cv::Mat Rl;
  cv::Mat Rr;
  cv::Mat Pl;
  cv::Mat Pr;
  cv::Mat Q;
  fs = cv::FileStorage("../stereo_rectify_params.yaml",cv::FileStorage::READ);
  fs["Rl"] >> Rl;
  fs["Rr"] >> Rr;
  fs["Pl"] >> Pl;
  fs["Pr"] >> Pr;
  fs["Q"] >> Q;
  fs.release();

  cv::Mat leftImage = cv::imread("../chessboard_images/aL30.bmp");
  cv::Mat rightImage = cv::imread("../chessboard_images/aR30.bmp");
  cv::Mat leftGray;
  cv::cvtColor(leftImage, leftGray, cv::COLOR_BGR2GRAY);

  cv::Mat rightGray;
  cv::cvtColor(rightImage, rightGray, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> leftCorners;
  std::vector<cv::Point2f> rightCorners;

  bool leftFound = false;
  bool rightFound = false;
  int boardHeight = 7;
  int boardWidth = 10;
  cv::Size boardSize = cv::Size(boardWidth, boardHeight);

  int flags(cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001);

  leftFound = cv::findChessboardCorners(leftImage, boardSize, leftCorners, flags);
  rightFound = cv::findChessboardCorners(rightImage, boardSize, rightCorners, flags);
  if(leftFound && rightFound)
  {
    cv::cornerSubPix(leftGray, leftCorners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
    cv::cornerSubPix(rightGray, rightCorners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    std::vector<cv::Point2f> leftUndisPoints;
    std::vector<cv::Point2f> rightUndisPoints;

    std::cout << leftCorners[0] << std::endl;
    std::cout << rightCorners[0] << std::endl;

    cv::undistortPoints(leftCorners, leftUndisPoints, leftCameraMatrix, leftDistCoeffs, Rl, Pl); //, cv::noArray(), leftCameraMatrix);
    cv::undistortPoints(rightCorners, rightUndisPoints, rightCameraMatrix, rightDistCoeffs, Rr, Pl); //, cv::noArray(), leftCameraMatrix);

    std::cout << "Left UnDist: " << leftUndisPoints[0] <<std::endl;
    std::cout << "Right UnDist: " << rightUndisPoints[0] <<std::endl;

    std::vector<cv::Point3f> perspLeft;
    std::vector<cv::Point3f> perspRight;
    for(int i=0;i<leftUndisPoints.size();i++)
    {
      perspLeft.push_back(cv::Point3f(leftUndisPoints[i].x,
        leftUndisPoints[i].y,
        leftUndisPoints[i].x-rightUndisPoints[i].x));
      // std::cout << perspLeft[i] << std::endl;
      // left3D.push_back(cv::Point3f(0,0,0));
      perspRight.push_back(cv::Point3f(rightUndisPoints[i].x,
        rightUndisPoints[i].y,
        leftUndisPoints[i].x-rightUndisPoints[i].x));
    }


    std::vector<cv::Point3f> left3D;
    cv::perspectiveTransform(perspLeft, left3D, Q);
    std::cout << left3D[0] << std::endl;

    std::vector<cv::Point3f> right3D;
    cv::perspectiveTransform(perspRight, right3D, Q);
    std::cout << right3D[0] << std::endl;

    std::vector<int> places{0,9,60,69};
    for(int i=0;i<places.size();i++)
    {
      std::cout << "Left " << places[i] << ": " << left3D[places[i]] << std::endl;
      std::cout << "Right " << places[i] << ": " << right3D[places[i]] << std::endl;
      cv::circle(leftImage, leftCorners[places[i]], 2, cv::Scalar(0, 0, 255), 2, 8);
      cv::circle(rightImage, rightCorners[places[i]], 2, cv::Scalar(0, 0, 255), 2, 8);
    }
    cv::imwrite("../left_four_corners.png", leftImage);
    cv::imwrite("../right_four_corners.png", rightImage);

  }

}
