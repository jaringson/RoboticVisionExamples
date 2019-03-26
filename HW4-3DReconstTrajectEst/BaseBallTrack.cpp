#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

int mouseX, mouseY;
std::vector<cv::Point> points;

//Left Camera
cv::Mat leftDistCoeffs;
cv::Mat leftCameraMatrix;


//Right Camera
cv::Mat rightDistCoeffs;
cv::Mat rightCameraMatrix;


//Stereo Params
cv::Mat Rl;
cv::Mat Rr;
cv::Mat Pl;
cv::Mat Pr;
cv::Mat Q;

//ROIs
std::vector<cv::Rect> allLeftROIs;
std::vector<cv::Rect> allRightROIs;
std::vector<int> savedInts;


void on_mouse(int evt, int x, int y, int flags, void* param)
{
    if(evt == cv::EVENT_LBUTTONDOWN)
    { //CV_EVENT_LBUTTONDOWN
        mouseX = x;
        mouseY = y;
    }
}

void calibrate_camera(cv::Mat frame, cv::Rect& calibrationRect)
{
    cv::namedWindow("ImageDisplay", 1);
    cv::setMouseCallback("ImageDisplay", on_mouse, (void*)&points);

    std::cout << "Please click on Top Left Corner. Then press space to Continue." << "\n";
    cv::imshow("ImageDisplay", frame);
    cv::waitKey(0);
    cv::Point tl(mouseX, mouseY);
    std::cout << "Point: " << mouseX << " " << mouseY << "\n";

    std::cout << "Please click on Bottom Right Corner. Then press space to Continue." << "\n";
    cv::imshow("ImageDisplay", frame);
    cv::waitKey(0);
    cv::Point br(mouseX, mouseY);
    std::cout << "Point: " << mouseX << " " << mouseY << "\n";

    calibrationRect = cv::Rect(tl,br);

    cv::destroyWindow("ImageDisplay");
}

cv::SimpleBlobDetector::Params setupParams()
{
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 100;
    params.maxThreshold = 255; //maybe try by circularity also
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByArea = true;
    params.minArea = 70;
    params.maxArea = 1256;
    params.filterByCircularity = true;
    params.filterByConvexity = false;
    params.filterByInertia = false;

    return params;
}

cv::Mat cleanUpNoise(cv::Mat noisy_img)
{
    cv::Mat img;

    //Maybe make this 3, 3
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::erode(noisy_img, img, element);
    cv::dilate(noisy_img, img, dilate_element);

    return img;
}

void get_3d_points(std::vector<cv::Point2f> leftInput,
  std::vector<cv::Point2f> rightInput,
  std::vector<cv::Point3f>& left3D,
  std::vector<cv::Point3f>& right3D)
{
  std::vector<cv::Point2f> leftUndisPoints;
  std::vector<cv::Point2f> rightUndisPoints;
  // std::cout << leftCameraMatrix << std::endl;
  // std::cout << leftDistCoeffs << std::endl;
  cv::undistortPoints(leftInput, leftUndisPoints, leftCameraMatrix, leftDistCoeffs, Rl, Pl); //, cv::noArray(), leftCameraMatrix);
  cv::undistortPoints(rightInput, rightUndisPoints, rightCameraMatrix, rightDistCoeffs, Rr, Pl); //, cv::noArray(), leftCameraMatrix);

  std::vector<cv::Point3f> perspLeft;
  std::vector<cv::Point3f> perspRight;
  for(int i=0;i<leftUndisPoints.size();i++)
  {
    perspLeft.push_back(cv::Point3f(leftUndisPoints[i].x,
      leftUndisPoints[i].y,
      leftUndisPoints[i].x-rightUndisPoints[i].x));
    // std::cout << perspLeft[i] << std::endl;
    // left3D.push_back(cv::Point3f(0,0,0));
    perspRight.push_back(cv::Point3f(rightUndisPoints[i].x,
      rightUndisPoints[i].y,
      leftUndisPoints[i].x-rightUndisPoints[i].x));
  }

  cv::perspectiveTransform(perspLeft, left3D, Q);
  cv::perspectiveTransform(perspRight, right3D, Q);
}


int main(int argc, char** argv)
{
  cv::FileStorage fs("../left_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> leftDistCoeffs;
  fs["Intrinsic"] >> leftCameraMatrix;
  fs.release();

  fs = cv::FileStorage("../right_params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> rightDistCoeffs;
  fs["Intrinsic"] >> rightCameraMatrix;
  fs.release();

  fs = cv::FileStorage("../stereo_rectify_params.yaml",cv::FileStorage::READ);
  fs["Rl"] >> Rl;
  fs["Rr"] >> Rr;
  fs["Pl"] >> Pl;
  fs["Pr"] >> Pr;
  fs["Q"] >> Q;
  fs.release();

  fs = cv::FileStorage("../all_calibration_roi.yaml",cv::FileStorage::READ);
  fs["LeftROIs"] >> allLeftROIs;
  fs["RightROIs"] >> allRightROIs;
  fs["SavedInts"] >> savedInts;
  fs.release();

  //Set up blob detector
  cv::SimpleBlobDetector::Params params = setupParams();
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  //Set up Rectangles
  cv::Rect leftRect(cv::Point(-1,-1),cv::Point(-1,-1));
  cv::Rect rightRect(cv::Point(-1,-1),cv::Point(-1,-1));

  cv::Mat leftImage = cv::imread("../baseball/ballL00.bmp");
  cv::Mat rightImage = cv::imread("../baseball/ballR00.bmp");

  cv::Mat leftGray;
  cv::Mat rightGray;

  cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2GRAY);
  cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2GRAY);

  cv::Mat leftAverage = cv::Mat::zeros(leftImage.size(), leftImage.type());
  cv::Mat rightAverage = cv::Mat::zeros(rightImage.size(), rightImage.type());

  leftAverage.convertTo(leftAverage, CV_32F);
  rightAverage.convertTo(rightAverage, CV_32F);

  double numOfDiff{21};
  int locSavedInts{0};

  cv::FileStorage fs_out("../all_xyz.yaml",cv::FileStorage::WRITE);
  std::vector<cv::Point3f> left3DPoints;
  std::vector<cv::Point3f> right3DPoints;

  for(int i=0;i<100;i++)
  {
    std::cout << "Iteration: " << i << std::endl;
    if(i<=9)
    {
      leftImage = cv::imread("../baseball/ballL0"+std::to_string(i)+".bmp");
      rightImage = cv::imread("../baseball/ballR0"+std::to_string(i)+".bmp");
    }
    else
    {
      leftImage = cv::imread("../baseball/ballL"+std::to_string(i)+".bmp");
      rightImage = cv::imread("../baseball/ballR"+std::to_string(i)+".bmp");
    }

    cv::cvtColor(leftImage, leftImage, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage, cv::COLOR_BGR2GRAY);

    if(i<=numOfDiff)
    {
      leftImage.convertTo(leftImage, CV_32F);
      rightImage.convertTo(rightImage, CV_32F);
      leftAverage += leftImage;
      rightAverage += rightImage;
      continue;
    }
    if(i==numOfDiff+1)
    {
      leftAverage /= numOfDiff;
      rightAverage /= numOfDiff;

      leftAverage.convertTo(leftAverage, CV_8U);
      rightAverage.convertTo(rightAverage, CV_8U);
    }
    if(locSavedInts <savedInts.size() && i==savedInts[locSavedInts])
    {
      cv::Rect leftROI = allLeftROIs[locSavedInts];
      cv::Rect rightROI = allRightROIs[locSavedInts];

      cv::Mat leftImageROI = leftImage(leftROI);
      cv::Mat rightImageROI = rightImage(rightROI);

      cv::Mat leftAverageROI = leftAverage(leftROI);
      cv::Mat rightAverageROI = rightAverage(rightROI);



      cv::Mat leftDiff;
      cv::Mat rightDiff;
      cv::absdiff(leftImageROI, leftAverageROI, leftDiff);
      cv::absdiff(rightImageROI, rightAverageROI, rightDiff);

      // cv::threshold(leftDiff, leftDiff, 10, 255, cv::THRESH_BINARY);
      // cv::threshold(rightDiff, rightDiff, 10, 255, cv::THRESH_BINARY);

      // leftDiff = cleanUpNoise(leftDiff);
      // rightDiff = cleanUpNoise(rightDiff);

      // std::vector<cv::KeyPoint> leftKeypts;
      // std::vector<cv::KeyPoint> rightKeypts;
      // detector->detect(leftDiff, leftKeypts);
      // detector->detect(rightDiff, rightKeypts);
      std::vector<cv::Point2f> leftPoints;
      // cv::KeyPoint::convert(leftKeypts, leftPoints);
      std::vector<cv::Point2f> rightPoints;
      // cv::KeyPoint::convert(rightKeypts, rightPoints);

      cv::Moments leftM = cv::moments(leftDiff, true);
      cv::Moments rightM = cv::moments(rightDiff, true);

      leftPoints.push_back(cv::Point2f(leftM.m10/leftM.m00, leftM.m01/leftM.m00));
      rightPoints.push_back(cv::Point2f(rightM.m10/rightM.m00, rightM.m01/rightM.m00));

      cv::cvtColor(leftImage, leftImage, cv::COLOR_GRAY2BGR);
      cv::cvtColor(rightImage, rightImage, cv::COLOR_GRAY2BGR);
      for(int j=0;j<leftPoints.size();j++)
      {
        leftPoints[j] = cv::Point2f(leftPoints[j].x+leftROI.tl().x,
        leftPoints[j].y+leftROI.tl().y);
        cv::circle(leftImage, leftPoints[j], 2, cv::Scalar(0, 0, 255), 1, 8);

        for(int j=0;j<rightPoints.size();j++)
        {
          rightPoints[j] = cv::Point2f(rightPoints[j].x+rightROI.tl().x,
          rightPoints[j].y+rightROI.tl().y);
          cv::circle(rightImage, rightPoints[j], 2, cv::Scalar(0, 0, 255), 1, 8);
        }
      }

      if(locSavedInts==0||locSavedInts==4||locSavedInts==9||locSavedInts==14||locSavedInts==19)
      {
        cv::imwrite("../LeftTrackBall_"+std::to_string(locSavedInts)+".png", leftImage);
        cv::imwrite("../RightTrackBall_"+std::to_string(locSavedInts)+".png", rightImage);
      }


      std::vector<cv::Point3f> left3D;
      std::vector<cv::Point3f> right3D;

      if(leftPoints.size()>0 && rightPoints.size()>0)
      {
        get_3d_points(leftPoints, rightPoints, left3D, right3D);
        std::cout << left3D[0] << std::endl;
        std::cout << right3D[0] << std::endl;
        left3DPoints.push_back(left3D[0]);
        right3DPoints.push_back(right3D[0]);
      }


      std::cout << "L pts: " << leftPoints.size() << " R pts: " << rightPoints.size() << std::endl;


      cv::imshow("L Diff", leftDiff);
      cv::imshow("R Diff", rightDiff);
      cv::imshow("L Image", leftImage);
      cv::imshow("R Image", rightImage);
      cv::waitKey(0);

      locSavedInts++;
    }


  }

  fs_out << "left3DPoints" << left3DPoints;
  fs_out << "right3DPoints" << right3DPoints;
  fs_out.release();

}
