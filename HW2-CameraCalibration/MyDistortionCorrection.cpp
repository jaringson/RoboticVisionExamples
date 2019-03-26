#include <stdio.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
  cv::VideoCapture video(0);
  cv::Mat im;

  //TO READ
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::FileStorage fs("../my_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> distCoeffs;
  fs["Intrinsic"] >> cameraMatrix;
  fs.release();

  std::cout << "Distortion = "<< std::endl << " "  << distCoeffs << std::endl << std::endl;
  cv::Mat image;

  for(;;)
  {
    video >> image;

    // cv::imwrite("test.jpg", image);
    cv::imshow("Ellingson", image);
    char c = cv::waitKey(30);
    if(c=='s')
    {
      cv::Mat changedImage;
      cv::undistort(image, changedImage, cameraMatrix, distCoeffs);

      cv::absdiff(image, changedImage, changedImage);

      cv::imshow("Image", image);
      cv::imshow("Changed", changedImage);

      cv::imwrite("../MyImage.jpg", image);
      cv::imwrite("../MyAbsDiff.jpg", changedImage);
      char c = cv::waitKey();
      if(c=='q') break;
    }



  }

  return 0;
}
