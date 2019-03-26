#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>


std::vector<cv::Point2f> get_features(cv::Mat frameGray)
{
  std::vector<cv::Point2f> corners;
  int maxCorners = 500;
  double minDistance = 10;
  int blockSize = 3;
  double qualityLevel = 0.01;
  bool useHarrisDetector = false;
  double k = 0.04;
  cv::Mat output;
  cv::Mat dst_norm_scaled;
  cv::Size winSize = cv::Size( 5, 5 );
  cv::Size zeroZone = cv::Size( -1, -1 );
  cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );


  cv::goodFeaturesToTrack(frameGray,
                    corners,
                    maxCorners,
                    qualityLevel,
                    minDistance,
                    cv::Mat(), blockSize, useHarrisDetector, k);


  cv::cornerSubPix(frameGray, corners, winSize, zeroZone, criteria);

  // int r = 4;
  // for( int i = 0; i < corners.size(); i++ )
  // {
  //   circle(frameGray, corners[i], r, cv::Scalar(0,0,0), -1, 8, 0);
  // }
  // cv::imshow("Good Features",frameGray);
  // cv::waitKey(0);
  return corners;
}

void run(int nSkip, cv::VideoWriter VOut)
{
  cv::VideoCapture video("../MotionFieldVideo.mp4");
  cv::Mat frame;
  cv::Mat frameGray;
  cv::Mat framePrev;
  cv::Mat frameGrayPrev;

  int counter{-1};
  std::vector<cv::Point2f> corners;
  std::vector<cv::Point2f> prevCorners;

  cv::Size winSize(21,21);
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01);

  for(;;)
  {
    counter++;
    video >> frame;
    // std::cout << "rows " << frame.rows <<std::endl;
    // std::cout << "cols " << frame.cols <<std::endl;
    cv::cvtColor(frame, frameGray, CV_RGB2GRAY);

    if(counter == 0)
    {
      std::cout << "In continue: " << counter << std::endl;
      frame.copyTo(framePrev);
      prevCorners = get_features(frameGray);
      continue;
    }

    std::cout << counter <<std::endl;

    if(counter % nSkip == 0)
    {
      // corners = get_features(frameGray);
      std::vector<cv::Point2f> corners;
      std::vector<uchar> status;
      std::vector<float> err;

      cv::cvtColor(framePrev, frameGrayPrev, CV_RGB2GRAY);
      calcOpticalFlowPyrLK(frameGrayPrev, frameGray, prevCorners,
            corners, status, err, winSize,
            3, termcrit, 0, 0.001);

      int countGoodPoints{0};
      for(int i=0;i<corners.size();i++)
      {
        // std::cout << std::string(status[i],1) << std::endl;
        if( !status[i] || err[i] > 20)
          continue;
        countGoodPoints++;
        circle(framePrev, prevCorners[i], 3, cv::Scalar(0,255,0), -1, 8, 0);
        // std::cout <<"p1: "<<prevCorners[i]<<" p2: "<<corners[i]<<std::endl;
        line(framePrev, prevCorners[i], corners[i], cv::Scalar(0,0,255), 2);
      }

      // std::cout << prevCorners.size() << "  " << countGoodPoints << "\n";
      if((double)countGoodPoints/prevCorners.size() < 0.5)
      {
        std::cout <<(double)countGoodPoints/prevCorners.size() <<std::endl;
        // break;
      }


      prevCorners = get_features(frameGray);


      // cv::imshow("Vectors",framePrev);
      VOut << framePrev;
      frame.copyTo(framePrev);
      // cv::waitKey(1);

    }
    if(counter==1026)
      break;
  }
  video.release();
}
int main(int argc, char** argv)
{
  cv::VideoWriter VOut;
  VOut.open("../OpticalFlow3Pyr.avi", CV_FOURCC('M', 'P', 'E', 'G'), 20, cv::Size(1920,1080), 1);
  run(1, VOut);
  run(10, VOut);
  VOut.release();
}
