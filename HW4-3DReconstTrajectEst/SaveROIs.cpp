#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

int mouseX, mouseY;
std::vector<cv::Point> points;


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

int main(int argc, char** argv)
{


  cv::Mat leftImage = cv::imread("../baseball/ballL00.bmp");
  cv::Mat rightImage = cv::imread("../baseball/ballR00.bmp");

  std::vector<cv::Rect> leftROIs;
  std::vector<cv::Rect> rightROIs;
  std::vector<int> savedInts;

  std::cout << "c==calibration p==play" <<std::endl;
  cv::imshow("Wait Image", leftImage);
  char key = cv::waitKey(0);
  if(key=='c')
  {
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

      std::cout << "s==save calibration" <<std::endl;
      cv::imshow("Save ROI?",leftImage);
      char save = cv::waitKey(0);
      if(save=='s')
      {
        cv::Rect leftRect;
        cv::Rect rightRect;
        calibrate_camera(leftImage, leftRect);
        calibrate_camera(rightImage, rightRect);

        cv::Rect leftROI(leftRect.tl().x,
          leftRect.tl().y,
          leftRect.br().x-leftRect.tl().x,
          leftRect.br().y-leftRect.tl().y);

        cv::Rect rightROI(rightRect.tl().x,
          rightRect.tl().y,
          rightRect.br().x-rightRect.tl().x,
          rightRect.br().y-rightRect.tl().y);

        leftROIs.push_back(leftROI);
        rightROIs.push_back(rightROI);
        savedInts.push_back(i);
      }

    }

    cv::FileStorage fs_out("../all_calibration_roi.yaml",cv::FileStorage::WRITE);
    fs_out << "LeftROIs" << leftROIs;
    fs_out << "RightROIs" << rightROIs;
    fs_out << "SavedInts" << savedInts;
    fs_out.release();
  }
  if(key=='p')
  {
    cv::FileStorage fs_in("../all_calibration_roi.yaml",cv::FileStorage::READ);
    fs_in["LeftROIs"] >> leftROIs;
    fs_in["RightROIs"] >> rightROIs;
    fs_in["SavedInts"] >> savedInts;
    fs_in.release();

    int locSavedInts{0};
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
      if(i==savedInts[locSavedInts])
      {
        std::cout << "i: " << i << " Loc: " << locSavedInts <<  " Saved Int: " << savedInts[locSavedInts] << std::endl;
        std::cout << leftROIs[locSavedInts] << std::endl;
        cv::imshow("Cut Left",leftImage(leftROIs[locSavedInts]));
        cv::imshow("Cut Right",rightImage(rightROIs[locSavedInts]));
        cv::waitKey(0);
        locSavedInts++;
      }

    }

  }
}
