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

void run(int folderChoice, double scale, int closestI, int whichR, int whichT)
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
    // cv::imshow("Vectors",framePrev);
    // cv::waitKey(0);
    frame.copyTo(framePrev);

  }


  //Camera Guesses
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::FileStorage fs("../SFMCameraParameters.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> distCoeffs;
  fs["Intrinsic"] >> cameraMatrix;
  fs.release();

  std::vector<cv::Point2f> firstCornersUnd;
  std::vector<cv::Point2f> lastCornersUnd;
  cv::undistortPoints(firstCorners, firstCornersUnd, cameraMatrix, distCoeffs);
  cv::undistortPoints(lastCorners, lastCornersUnd, cameraMatrix, distCoeffs);

  for( int t=0;t<firstCornersUnd.size();t++)
  {
    firstCornersUnd[t].x = firstCornersUnd[t].x
      * cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
    firstCornersUnd[t].y = firstCornersUnd[t].y
      * cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);
    lastCornersUnd[t].x = lastCornersUnd[t].x
      * cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
    lastCornersUnd[t].y = lastCornersUnd[t].y
      * cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);
  }
  cv::Size imageSize = cv::Size(640,480);

  cv::Mat status;
  cv::Mat F = cv::findFundamentalMat(firstCornersUnd, lastCornersUnd, CV_FM_RANSAC, 3, 0.99, status);
  cv::Mat E = cameraMatrix.t()*F*cameraMatrix;

  // cv::normalize(E, E, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
  cv::Mat R1;
  cv::Mat R2;
  cv::Mat T;
  cv::decomposeEssentialMat(E, R1, R2, T);

  // cv::Mat w, u, vt;
  // cv::SVD::compute(E,w,u,vt);
  // cv::Mat Rz =(cv::Mat_<double>(3,3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
  // cv::Mat Sigma =(cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 0);
  // cv::Mat R = u*Rz*vt;
  // cv::Mat T_hat = u*Rz*Sigma*u.t();
  // cv::Mat T = (cv::Mat_<double>(3,1) << T_hat.at<double>(2,1), T_hat.at<double>(0,2), T_hat.at<double>(1,0));
  T *= scale;
  std::cout << "F=\n" << F << std::endl;
  std::cout << "E=\n" << E << std::endl;
  std::cout << "R1=\n"<< R1 << std::endl;

  std::cout << "R2=\n"<< R2 << std::endl;
  // std::cout << "T_hat=\n"<< T_hat << std::endl;
  std::cout << "T=\n"<< T << std::endl;

  cv::Mat R;

  if(whichR == 1)
    R = R1;
  else
    R = R2;
  if(whichT == 2)
    T = -1*T;

  //
  // cv::Mat Rrp;
  // cv::Mat Trp;
  // cv::Mat tr;
  //
  // cv::Point2d pp = cv::Point2d(cameraMatrix.at<double>(0,2),cameraMatrix.at<double>(1,2));
  // double focal = cameraMatrix.at<double>(0,0);
  //
  // std::cout << pp << std::endl;
  //
  // cv::recoverPose(E, firstCornersUnd, lastCornersUnd, Rrp, Trp, focal, pp, tr);
  // std::cout << "Rrp=\n"<< Rrp << std::endl;
  //
  // std::cout << "Trp=\n"<< Trp << std::endl;

  cv::Mat Rl;
  cv::Mat Rr;
  cv::Mat Pl;
  cv::Mat Pr;
  cv::Mat Q;

  cv::stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs,
    imageSize, R, T, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY);
  // std::cout << "Q=\n" << Q << std::endl;

  std::vector<cv::Point3f> firstDiparity;
  std::vector<cv::Point3f> first3D;
  for(int i=0;i<firstCornersUnd.size();i++)
  {
    firstDiparity.push_back(cv::Point3f(firstCornersUnd[i].x,
      firstCornersUnd[i].y,
      firstCornersUnd[i].x-lastCornersUnd[i].x));
  }
  cv::perspectiveTransform(firstDiparity, first3D, Q);

  double closest = 1000;
  // closestI = -1;
  // std::cout << "All I's" << std::endl;
  // for(int i=0;i<first3D.size();i++)
  // {
  //   if(first3D[i].z < 0)
  //     continue;
  //   // std::cout << first3D[i] << std::endl;
  //   if(firstCorners[i].x > 292 && firstCorners[i].x < 298) //296
  //   {
  //     std::cout << i << "\n";
  //     closest = first3D[i].z;
  //     closestI = i;
  //   }
  // }
  closest = first3D[closestI].z;

  std::cout <<"One Closest Corner: " << closest << std::endl;
  // std::cout << "I: " <<closestI << "\n";
  circle(firstFrame, firstCorners[closestI], 4, cv::Scalar(0,255,0), -1, 8, 0);
  std::cout << "Point: " << closestI << " 3D Location: " << first3D[closestI] << "\n";

  std::vector<int> otherPoints{1,50,100};
  for(int i=0;i<otherPoints.size();i++)
  {
    circle(firstFrame, firstCorners[otherPoints[i]], 4, cv::Scalar(0,255,0), -1, 8, 0);
    std::cout << "Point: " << otherPoints[i] << " 3D Location: " << first3D[i] << "\n";
  }

  cv::imshow("First Frame",firstFrame);
  cv::waitKey(0);

  cv::imwrite("../SelectPoints"+folders[folderChoice]+".png", firstFrame);



}

int main(int argc, char** argv)
{
  cv::VideoWriter VOut;
  VOut.open("../MultiFrameTracking.avi", CV_FOURCC('M', 'P', 'E', 'G'), 20, cv::Size(1920,1080), 1);
  // run(1, VOut);


  run(0, 2.875, 141, 2, 1);
  run(1, 2.875, 141, 2, 1);
  run(2, 2.570, 51, 2, 1);
  run(3, 2.570, 51, 2, 1);
//   10
// 75
// 86
// 88
// 98
// 103
// 104
// 106
// 109
// 141
// 144



  VOut.release();
}
