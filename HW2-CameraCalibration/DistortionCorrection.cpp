#include <stdio.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

void split(std::string str, std::string splitBy, std::vector<std::string>& tokens)
{
    tokens.push_back(str);
    size_t splitAt;
    size_t splitLen = splitBy.size();
    std::string frag;
    while(true)
    {
        frag = tokens.back();
        splitAt = frag.find(splitBy);
        if(splitAt == std::string::npos)
        {
            break;
        }
        tokens.back() = frag.substr(0, splitAt);
        tokens.push_back(frag.substr(splitAt+splitLen, frag.size()-(splitAt+splitLen)));
    }
}

int main(int argc, char** argv)
{
  //TO READ
  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  cv::FileStorage fs("../params.yaml",cv::FileStorage::READ);
  fs["Distortion"] >> distCoeffs;
  fs["Intrinsic"] >> cameraMatrix;
  fs.release();

  std::cout << "Distortion = "<< std::endl << " "  << distCoeffs << std::endl << std::endl;

  std::string imageFar = "../CalibrationImagesJPG/Far.jpg";
  std::string imageClose = "../CalibrationImagesJPG/Close.jpg";
  std::string imageTurn = "../CalibrationImagesJPG/Turn.jpg";

  std::vector<std::string> allStrings;
  allStrings.push_back(imageFar);
  allStrings.push_back(imageClose);
  allStrings.push_back(imageTurn);

  for(int i=0;i<allStrings.size();i++)
  {
    cv::Mat image = cv::imread(allStrings[i]);
    cv::Mat changedImage;
    cv::undistort(image, changedImage, cameraMatrix, distCoeffs);

    cv::absdiff(image, changedImage, changedImage);

    cv::imshow("Image", image);
    cv::imshow("Changed", changedImage);

    std::vector<std::string> results;
    split(allStrings[i], "/", results);

    cv::imwrite("../AbsDiff_"+results[results.size()-1], changedImage);
    char c = cv::waitKey();
    if(c=='q') break;
  }

  return 0;
}
