#include <stdio.h>
#include <opencv2/opencv.hpp>



int main(int argc, char** argv)
{
  cv::String path("../Baseball_Practice_Images/*.jpg"); //select only jpg
  std::vector<cv::String> fn;
  // std::vector<cv::Mat> data;
  cv::glob(path,fn,true);

  cv::Mat im_prev = cv::imread(fn[0]);
  cv::Mat diffImage;
  cv::namedWindow("Ellingson", CV_WINDOW_AUTOSIZE);

  cv::Mat first_im;

  cv::VideoWriter VOut;
  VOut.open("VideoOut2.avi", CV_FOURCC('M', 'P', 'E', 'G'), 30, cv::Size(640,480), 1);

  for (size_t k=0; k<fn.size(); ++k)
  {
    if(k <= 35)
      first_im = cv::imread(fn[0]);
    else
      first_im = cv::imread(fn[36]);

    cv::Mat im = cv::imread(fn[k]);

    if (im.empty()) continue;

    cv::absdiff(im, first_im, diffImage);

    cv::Mat binary;
    cv::threshold(diffImage, binary, 5, 255, cv::THRESH_BINARY);

    cv::Mat erode;
    int erosion_size = 4;
    int erosion_type = cv::MORPH_ELLIPSE;
    cv::Mat erosion_element = cv::getStructuringElement( erosion_type,
                                  cv::Size( 2*erosion_size+1, 2*erosion_size+1),
                                  cv::Point( erosion_size, erosion_size ) );

    cv::erode(binary, erode, erosion_element);

    cv::cvtColor(erode, erode, CV_BGR2GRAY);
    cv::Moments mu = cv::moments(erode, true);

    cv::Point center;
    center.x = mu.m10 / mu.m00;
    center.y = mu.m01 / mu.m00;

    cv::Mat result;
    cv::cvtColor(erode, result, CV_GRAY2BGR);

    cv::circle(im, center, 2, cv::Scalar(0,0,255), 2);

    cv::imshow("Ellingson", im);
    VOut << im;
    // cv::imshow("Ellingson", im);
    char c = cv::waitKey();
    if(c=='q') break;

    // im_prev = im.clone();

       // data.push_back(im);
  }

}
