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

  cv::Size imageSize = cv::Size(640,480);

  cv::Mat Rl;
  cv::Mat Rr;
  cv::Mat Pl;
  cv::Mat Pr;
  cv::Mat Q;

  cv::stereoRectify(leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs,
    imageSize, R, T, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY);

  cv::Mat newCameraMatrix;
  cv::Mat lmap1;
  cv::Mat lmap2;
  cv::initUndistortRectifyMap(leftCameraMatrix, leftDistCoeffs, Rl, Pl,
    imageSize, CV_32FC1, lmap1, lmap2);
  cv::remap(leftImage, leftChangedImage, lmap1, lmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

  cv::Mat rmap1;
  cv::Mat rmap2;
  cv::initUndistortRectifyMap(rightCameraMatrix, rightDistCoeffs, Rr, Pr,
    imageSize, CV_32FC1, rmap1, rmap2);
  cv::remap(rightImage, rightChangedImage, rmap1, rmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

  cv::Mat leftDiffImage;
  cv::Mat rightDiffImage;
  cv::absdiff(leftChangedImage, leftImage, leftDiffImage);
  cv::absdiff(rightChangedImage, rightImage, rightDiffImage);

  double x1 = 10.0;
  double x2 = 620.0;
  for(int i = 0;i<20;i++)
  {
    double y = i*24;
    cv::line(leftChangedImage, cv::Point(x1,y), cv::Point(x2,y), cv::Scalar(0,255,0), 1);
    cv::line(rightChangedImage, cv::Point(x1,y), cv::Point(x2,y), cv::Scalar(0,255,0), 1);

  }

  cv::imshow("Left", leftChangedImage);
  cv::imshow("Right", rightChangedImage);
  cv::imshow("Diff Left", leftDiffImage);
  cv::imshow("Diff Right", rightDiffImage);
  cv::waitKey(0);

  cv::imwrite("../leftOriginal.png", leftImage);
  cv::imwrite("../rightOriginal.png", rightImage);
  cv::imwrite("../leftRectification.png", leftChangedImage);
  cv::imwrite("../rightRectification.png", rightChangedImage);
  cv::imwrite("../leftDiff.png", leftDiffImage);
  cv::imwrite("../rightDiff.png" , rightDiffImage);

  return 0;
}
