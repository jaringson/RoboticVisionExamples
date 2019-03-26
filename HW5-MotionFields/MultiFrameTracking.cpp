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

  return corners;
}

cv::Mat get_area_in_image(cv::Mat input, cv::Point pt, cv::Size size)
{
  cv::Point newPt = cv::Point(pt.x-size.width/2.0,pt.y-size.height/2.0);
  cv::Rect roi{newPt, size};
  int cols = input.cols;
  int rows = input.rows;
  // std::cout << "image size" << input.size() << std::endl;
  if(newPt.x<0)
  {
    // std::cout << "x small" << newPt.x << std::endl;
    return get_area_in_image(input, cv::Point(pt.x+1,pt.y), size);
  }
  else if(newPt.x+size.width>cols)
  {
    // std::cout << "x large" << newPt.x+size.width << std::endl;
    return get_area_in_image(input, cv::Point(pt.x-1,pt.y), size);
  }
  else if(newPt.y<0)
  {
    // std::cout << "y small" << newPt.y << std::endl;
    return get_area_in_image(input, cv::Point(pt.x,pt.y+1), size);
  }
  else if(newPt.y+size.height>rows)
  {
    // std::cout << "y large" << newPt.y+size.height << std::endl;
    return get_area_in_image(input, cv::Point(pt.x,pt.y-1), size);
  }
  else
  {
    return input(roi);
  }
}

void run(int nSkip, cv::VideoWriter VOut)
{
  cv::VideoCapture video("../MotionFieldVideo.mp4");
  cv::Mat frame;
  cv::Mat frameGray;
  cv::Mat framePrev;
  cv::Mat frameGrayPrev;

  int counter{-1};
  std::vector<cv::Point2f> prevCorners;

  // cv::Size winSize(21,21);
  // cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01);

  int matchMethod = cv::TM_SQDIFF;
  cv::Size templateSize{30, 30};
  cv::Size searchSize{55, 55};
  // std::vector<

  for(;;)
  {
    counter++;
    video >> frame;
    cv::cvtColor(frame, frameGray, CV_RGB2GRAY);

    if(counter == 0)
    {
      std::cout << "In continue: " << counter << std::endl;
      frame.copyTo(framePrev);
      prevCorners = get_features(frameGray);
      continue;
    }

    std::cout << counter << std::endl;

    std::vector<cv::Point2f> corners;
    for(cv::Point2f pt : prevCorners)
    {
      // std::cout << pt << std::endl;
      cv::cvtColor(framePrev, frameGrayPrev, CV_RGB2GRAY);

      cv::Mat temp = get_area_in_image(frameGrayPrev, pt, templateSize);
      cv::Mat searchArea = get_area_in_image(frameGray, pt, searchSize);

      cv::Mat result;
      cv::matchTemplate(searchArea,temp,result,matchMethod);

      cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

      /// Localizing the best match with minMaxLoc
      double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
      cv::Point matchLoc;

      cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());


      // cv::imshow("Result",result);
      // cv::imshow("Temp", temp);
      // std::cout << result.size() << "\n";
      // cv::waitKey(0);
      matchLoc = cv::Point(pt.x-(searchSize.width-templateSize.width+1)/2.0+minLoc.x,
          pt.y-(searchSize.height-templateSize.height+1)/2.0+minLoc.y);
      corners.push_back(matchLoc);
      // rectangle(framePrev, matchLoc, cv::Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), cv::Scalar::all(0), 2, 8, 0 );
      // rectangle( framePrev, matchLoc, cv::Point( matchLoc.x + temp.cols , matchLoc.y + temp.rows ), cv::Scalar::all(0), 2, 8, 0 );
    }
    cv::Mat status;
    cv::findFundamentalMat(prevCorners, corners, CV_FM_RANSAC, 3, 0.99, status);

    std::vector<cv::Point2f> finalCorners;
    // std::cout << prevCorners.size() << "\n";
    for(int i=0;i<corners.size();i++)
    {
      // std::cout << "here" << std::endl;
      if(!status.at<unsigned char>(0,i))
        continue;
      finalCorners.push_back(corners[i]);
      circle(framePrev, prevCorners[i], 3, cv::Scalar(0,255,0), -1, 8, 0);
      line(framePrev, prevCorners[i], corners[i], cv::Scalar(0,0,255));
    }

    if(counter % nSkip == 0)
    {
      std::cout << "Get Features: " << counter << std::endl;
      prevCorners = get_features(frameGray);
      // prevCorners = finalCorners;
    }
    else
    {
      prevCorners = finalCorners;
    }
    VOut << framePrev;
    // cv::imshow("Vectors",framePrev);
    // cv::waitKey(1);
    frame.copyTo(framePrev);

    if(counter==1026)
      break;
  }
  video.release();

}

int main(int argc, char** argv)
{
  cv::VideoWriter VOut;
  VOut.open("../MultiFrameTracking.avi", CV_FOURCC('M', 'P', 'E', 'G'), 20, cv::Size(1920,1080), 1);
  // run(1, VOut);
  run(10, VOut);
  VOut.release();
}
