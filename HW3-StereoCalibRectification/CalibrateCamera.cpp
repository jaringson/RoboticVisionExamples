#include <stdio.h>
#include <opencv2/opencv.hpp>



int main(int argc, char** argv)
{
  cv::String path("../rightCalibrate/*.bmp");
  std::vector<cv::String> fn;

  cv::glob(path,fn,true);

  cv::Mat im_prev = cv::imread(fn[0]);
  cv::namedWindow("Ellingson", CV_WINDOW_AUTOSIZE);

  std::vector<std::vector<cv::Point2f>> all_corners;
  std::vector<std::vector<cv::Point3f>> world_points;

  int square_size = 1;//3.88636;
  int board_height = 7;
  int board_width = 10;
  cv::Size board_size = cv::Size(board_width, board_height);

  for (size_t k=0; k<fn.size(); ++k)
  {
    cv::Mat im = cv::imread(fn[k]);
    cv::Mat gray;
    cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;

    bool found = false;
    found = cv::findChessboardCorners(im, board_size, corners);// , CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE);
    if(found)
    {
      cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 30, 0.001));
      all_corners.push_back(corners);

      std::vector<cv::Point3f> obj;
      for (int i = 0; i < board_height; i++)
        for (int j = 0; j < board_width; j++)
          obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));
      world_points.push_back(obj);

      //// Part 1
      // drawChessboardCorners(im, board_size, corners, found);

      // cv::imshow("Ellingson", im);
      // cv::imwrite("part1.jpg",im);
      // char c = cv::waitKey();
      // if(c=='q') break;
    }
    else
      std::cout << "Can't find Chessboard Corners: " << fn[k] << std::endl;
  }
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::Size imageSize = cv::Size(640,480);

  cv::calibrateCamera(world_points, all_corners, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

  std::cout << "Intrinsic = "<< std::endl << " "  << cameraMatrix << std::endl << std::endl;
  std::cout << "Distortion = "<< std::endl << " "  << distCoeffs << std::endl << std::endl;

  cv::FileStorage fs("../right_params.yaml",cv::FileStorage::WRITE);
  fs << "Intrinsic" << cameraMatrix;
  fs << "Distortion" << distCoeffs;
  fs.release();

  return 0;
}
