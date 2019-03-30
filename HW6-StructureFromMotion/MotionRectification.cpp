#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

std::vector<cv::Point2f> get_features(cv::Mat frameGray)
{
  std::vector<cv::Point2f> corners;
  int maxCorners = 500;
  double minDistance = 15;
  int blockSize = 3;
  double qualityLevel = 0.01;
  bool useHarrisDetector = false;
  double k = 0.04;

  cv::goodFeaturesToTrack(frameGray,
                    corners,
                    maxCorners,
                    qualityLevel,
                    minDistance,
                    cv::Mat(), blockSize, useHarrisDetector, k);

  cv::Size winSize = cv::Size( 5, 5 );
  cv::Size zeroZone = cv::Size( -1, -1 );
  cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

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

void run(int folderChoice, cv::VideoWriter VOut)
{

  std::vector<std::string> folders{"ParallelCube", "ParallelReal",
                                    "TurnCube", "TurnReal"};

  int startFrame = 10;
  int endFrame = 15;
  int nSkip = endFrame-startFrame;

  cv::Mat frame;
  cv::Mat frameGray;
  cv::Mat framePrev;
  cv::Mat frameGrayPrev;

  int counter{-1};
  std::vector<cv::Point2f> prevCorners;

  std::vector<cv::Point2f> firstCorners;
  std::vector<cv::Point2f> lastCorners;
  cv::Mat firstFrame;
  cv::Mat lastFrame;

  int matchMethod = cv::TM_SQDIFF;
  cv::Size templateSize{15, 15};
  cv::Size searchSize{65, 65};

  for(int j=startFrame;j<=endFrame;j++)
  {
    counter++;
    std::string fileName = "../"+folders[folderChoice]+"/"+folders[folderChoice]+std::to_string(j)+".jpg";
    std::cout << fileName << "\n";
    frame = cv::imread(fileName);

    if(frame.channels() > 1)
      cv::cvtColor(frame, frameGray, CV_RGB2GRAY);
    else
      frame.copyTo(frameGray);

    if(counter == 0)
    {
      frame.copyTo(framePrev);
      prevCorners = get_features(frameGray);
      frame.copyTo(firstFrame);
      firstCorners = prevCorners;
      continue;
    }

    std::vector<cv::Point2f> corners;
    for(cv::Point2f pt : prevCorners)
    {
      // std::cout << pt << std::endl;
      if(framePrev.channels() > 1)
        cv::cvtColor(framePrev, frameGrayPrev, CV_RGB2GRAY);
      else
        framePrev.copyTo(frameGrayPrev);

      cv::Mat temp = get_area_in_image(frameGrayPrev, pt, templateSize);
      cv::Mat searchArea = get_area_in_image(frameGray, pt, searchSize);

      cv::Mat result;
      cv::matchTemplate(searchArea,temp,result,matchMethod);

      cv::normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

      /// Localizing the best match with minMaxLoc
      double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
      cv::Point matchLoc;

      cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());


      matchLoc = cv::Point(pt.x-(searchSize.width-templateSize.width+1)/2.0+minLoc.x+1,
          pt.y-(searchSize.height-templateSize.height+1)/2.0+minLoc.y+1);
      corners.push_back(matchLoc);

    }
    cv::Mat status;
    cv::findFundamentalMat(prevCorners, corners, CV_FM_RANSAC, 3, 0.99, status);

    std::vector<cv::Point2f> finalCorners;

    std::vector<cv::Point2f> tempFirstCorners;

    for(int i=0;i<corners.size();i++)
    {
      if(!status.at<unsigned char>(0,i))
      {
        continue;
      }
      tempFirstCorners.push_back(firstCorners[i]);
      finalCorners.push_back(corners[i]);
      circle(framePrev, prevCorners[i], 2, cv::Scalar(0,255,0), -1, 8, 0);
      line(framePrev, prevCorners[i], corners[i], cv::Scalar(0,0,255));
      // std::cout << "prev " << prevCorners[i] << " now " << corners[i] << std::endl;
    }
    firstCorners = tempFirstCorners;

    if(counter % nSkip == 0)
    {
      std::cout << "Get Features: " << counter << std::endl;
      // prevCorners = get_features(frameGray);
      // prevCorners = finalCorners;
      lastCorners = finalCorners;
      frame.copyTo(lastFrame);
    }
    else
    {
      prevCorners = finalCorners;
    }
    // VOut << framePrevi;
    cv::imshow("Vectors",framePrev);
    cv::waitKey(0);
    frame.copyTo(framePrev);

  }


  //Camera Guesses
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::FileStorage fs("../guess_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> distCoeffs;
  fs["Intrinsic"] >> cameraMatrix;
  fs.release();

  cv::Size imageSize = cv::Size(640,480);

  cv::Mat status;

  cv::Mat F = cv::findFundamentalMat(firstCorners, lastCorners, CV_FM_RANSAC, 3, 0.99, status);
  cv::Mat H1;
  cv::Mat H2;
  cv::stereoRectifyUncalibrated(firstCorners, lastCorners, F, imageSize,
      H1, H2, 5.0);


  cv::Mat R1 = cameraMatrix.inv()*H1*cameraMatrix;
  cv::Mat R2 = cameraMatrix.inv()*H2*cameraMatrix;
  cv::Mat lmap1;
  cv::Mat lmap2;

  cv::Mat firstFrameChanged;
  cv::Mat lastFrameChanged;

  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, cameraMatrix,
    imageSize, CV_32FC1, lmap1, lmap2);

  cv::remap(firstFrame, firstFrameChanged, lmap1, lmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

  cv::Mat rmap1;
  cv::Mat rmap2;
  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R2, cameraMatrix,
    imageSize, CV_32FC1, rmap1, rmap2);

  cv::remap(lastFrame, lastFrameChanged, rmap1, rmap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

  double x1 = 10.0;
  double x2 = 620.0;
  for(int i = 0;i<20;i++)
  {
    double y = i*24;
    cv::line(firstFrameChanged, cv::Point(x1,y), cv::Point(x2,y), cv::Scalar(0,255,0), 1);
    cv::line(lastFrameChanged, cv::Point(x1,y), cv::Point(x2,y), cv::Scalar(0,255,0), 1);
  }


  // cv::imshow("First Warp",firstFrameChanged);
  // cv::imshow("Last Warp",lastFrameChanged);
  // cv::waitKey(0);

  cv::imwrite("../First"+folders[folderChoice]+".png", firstFrameChanged);
  cv::imwrite("../Last"+folders[folderChoice]+".png", lastFrameChanged);
}

int main(int argc, char** argv)
{
  cv::VideoWriter VOut;
  VOut.open("../MultiFrameTracking.avi", CV_FOURCC('M', 'P', 'E', 'G'), 20, cv::Size(1920,1080), 1);
  // run(1, VOut);


  run(0, VOut);
  // run(1, VOut);
  // run(2, VOut);
  // run(3, VOut);


  VOut.release();
}
