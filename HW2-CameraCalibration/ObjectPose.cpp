#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>


int main(int argc, char** argv)
{
  //TO READ
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::FileStorage fs("../params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> distCoeffs;
  fs["Intrinsic"] >> cameraMatrix;
  fs.release();
  std::ifstream infile("../Data_Points.txt");

  float a, b, c;
  int lineNumber{0};
  std::vector<cv::Point2f> imagePoints;
  std::vector<cv::Point3f> objectPoints;
  while (infile >> a >> b)
  {
      if(lineNumber < 20)
      {
        // std::cout << a << " " << b << "\n";
        cv::Point2f point{a,b};
        imagePoints.push_back(point);
      }
      else
      {
        infile >> c;
        // std::cout << a << " " << b << " " << c << "\n";
        cv::Point3f point{a,b,c};
        objectPoints.push_back(point);
      }

      lineNumber++;
  }
  // std::cout << imagePoints.size() << "\n";
  // std::cout << objectPoints.size() << "\n";
  cv::Mat rvecs;
  cv::Mat tvecs;
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs);
  cv::Rodrigues(rvecs, rvecs);

  std::cout << "Rotation = "<< std::endl << " "  << rvecs << std::endl << std::endl;
  std::cout << "Translation = "<< std::endl << " "  << tvecs << std::endl << std::endl;

}
