// #include <string>
// #include <vector>

// #include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "order_roi.hpp"

// using namespace cv;
// using namespace std;

//ROI ORDERING
void orderROI(cv::Mat const & img, std::vector<cv::Vec3f> circles, int* pointer)
{
    // Load image from file
    // cv::Mat img = cv::imread(filename.c_str());
    // if(img.empty()) {
    //     throw std::runtime_error("Failed to open the file " + filename);
    // }
    
    cv::Mat circles_img = img.clone();

    int next = 1;
    for( int i = 0; i < circles.size(); ++i )
    {
        for(int j = 0; j< circles.size(); ++j)
        {
            //Compare recognized digits in ROI with digits in order
            if(pointer[j] == next)
            {
                //Draw a center of ROI in order
                cv::Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));             
                cv::circle( circles_img, center, 10, cv::Scalar(0, 250 ,255), -1, 8, 0 );
                std::cout << "  Point number " << (next) << " is at coordinates : [" <<  round(circles[j][0])  <<  ", " << round(circles[j][1]) << "] " <<std::endl;
                std::string name = "Ordered Circles";
                cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
                cv::resizeWindow(name.c_str(), 640, 512);
                cv::imshow(name.c_str(), circles_img);

                cv::waitKey(0);

                next++;
            }
        }
                

    }
}
