#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

int mouseX, mouseY;
std::vector<cv::Point> points;

void on_mouse(int evt, int x, int y, int flags, void* param) {
   if(evt == cv::EVENT_LBUTTONDOWN) { //CV_EVENT_LBUTTONDOWN
       mouseX = x;
       mouseY = y;
       std::cout << mouseX << " " << mouseY << "\n";
   }
}

int main(int argc, char** argv)
{
  std::string file = "../ParallelCube/ParallelCube10.jpg";
  cv::Mat image = cv::imread(file);
  cv::namedWindow("ImageDisplay", 1);
  cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);
  cv::imshow("ImageDisplay", image);
  cv::waitKey(0);

  cv::destroyWindow("ImageDisplay");



}
