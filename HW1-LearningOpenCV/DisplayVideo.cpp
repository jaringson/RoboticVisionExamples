#include <stdio.h>
#include <opencv2/opencv.hpp>



int main(int argc, char** argv)
{
  cv::VideoCapture video(0);

  // int frame_width = video.get(cv::CV_CAP_PROP_FRAME_WIDTH);
  // int frame_height = video.get(CV_CAP_PROP_FRAME_HEIGHT);
  cv::VideoWriter VOut;
  VOut.open("VideoOut.avi", CV_FOURCC('M', 'P', 'E', 'G'), 30, cv::Size(640,480), 1);
  // VOut.open("VideoOut.avi", -1, 30, cv::Size(640,480), 1);



  cv::Mat frame;
  cv::Mat frame_gray;
  cv::Mat frame_prev;
  cv::namedWindow("Ellingson", CV_WINDOW_AUTOSIZE);

  char function{'n'};

  cv::Mat output_image;

  for(;;)
  {

    video >> frame;
    cvtColor(frame, frame_gray, CV_BGR2GRAY);

    switch(function)
    {
      case 'n':
        // cv::imshow("Ellingson", frame);
        output_image = frame;
        break;

      case 'c':
      {
        cv::Mat contours;
        cv::Canny(frame, contours, 50, 200, 3);
        // cv::imshow("Ellingson", contours);
        cvtColor(contours, contours, CV_GRAY2BGR);
        output_image = contours;
        break;
      }
      case 'b':
      {
        cv::Mat binary;
        cv::threshold(frame_gray, binary, 127, 255, cv::THRESH_BINARY);
        // cv::imshow("Ellingson", binary);
        cvtColor(binary, binary, CV_GRAY2BGR);
        output_image = binary;
        break;
      }
      case 'f':
      {
        std::vector<cv::Point2f> corners;
        int maxCorners = 100;
        double minDistance = 10;
        int blockSize = 3;
        double qualityLevel = 0.01;
        bool useHarrisDetector = false;
        double k = 0.04;
        cv::Mat output;
        cv::Mat dst_norm_scaled;
        cv::Size winSize = cv::Size( 5, 5 );
        cv::Size zeroZone = cv::Size( -1, -1 );
        cv::TermCriteria criteria = cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

        cv::goodFeaturesToTrack(frame_gray,
                          corners,
                          maxCorners,
                          qualityLevel,
                          minDistance,
                          cv::Mat(), blockSize, useHarrisDetector, k);
        int r = 4;
        cv::RNG rng(12345);
        for( int i = 0; i < corners.size(); i++ )
        {
          circle(frame_gray, corners[i], r, cv::Scalar(0,0,0), -1, 8, 0);
        }


        cv::cornerSubPix(frame_gray, corners, winSize, zeroZone, criteria);

        // cv::imshow("Ellingson", frame_gray);
        cvtColor(frame_gray, frame_gray, CV_GRAY2BGR);
        output_image = frame_gray;

        break;
      }


      case 'l':
      {
        cv::Mat contours;
        cv::Canny(frame, contours, 50, 200, 3);
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(contours, lines, 1, CV_PI/180, 50, 100, 5);
        for( size_t i = 0; i < lines.size(); i++ )
        {
          cv::Vec4i l = lines[i];
          cv::line(frame_gray, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        }
        // cv::imshow("Ellingson", frame_gray);
        cvtColor(frame_gray, frame_gray, CV_GRAY2BGR);
        output_image = frame_gray;
        break;
      }

      case 'd':
      {
        cv::Mat diffImage;
        cv::absdiff(frame, frame_prev, diffImage);
        // cv::imshow("Ellingson", diffImage);
        output_image = diffImage;
        break;
      }

      default:
        break;

    }
    cv::imshow("Ellingson", output_image);

    VOut << output_image;

    char c = cv::waitKey(30);

    switch(c)
    {
      case 'n':
        function = 'n';
        break;

      case 'c':
        function = 'c';
        break;

      case 'b':
        function = 'b';
        break;

      case 'f':
        function = 'f';
        break;

      case 'l':
        function = 'l';
        break;

      case 'd':
        function = 'd';
        break;
    }

    if(c == 'q') break;

    frame_prev = frame.clone();
  }


  // cv::waitKey(0);

  return 0;
}
