//main.cpp
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "camera_calibration.hpp"
#include "map_extraction.hpp"
#include "shape_detection.hpp"
#include "digit_recognition.hpp"
#include "order_roi.hpp"
#include "reader.hpp"
#include "dubins.hpp"

int main(int argc, char* argv[])
{
    //MENU
    // std::cout << "Please choose an option: " << std::endl;
    // if (argc != 2) 
    // {
    //     std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
    //     return 0;
    // }

    // std::string filename = argv[1];

    // calibrateCamera();//+

    // std::string outputFilename = extractMap(filename);//+

    // std::vector<cv::Vec3f> circles = processImage(outputFilename);

    // int *pointer = recognizeDigits(outputFilename, circles);

    // orderROI(outputFilename, circles, pointer);//+

    readData("data/output/output.txt");

    double x0 = 171, y0 = 292, xf = 790, yf = 298; 
    double th0 = -(2*M_PI/3), thf = M_PI/3, Kmax = 0.01;
    double radius = 50;

    DubinsCurve curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);

    std::cout<<"Arc1: "<<std::endl;
    std::cout<<"x0: "<<curve.arc1.x0<<std::endl<<"y0: "<<curve.arc1.y0<<std::endl<<"th0: "<<curve.arc1.th0<<std::endl;				
    std::cout<<"k: "<<curve.arc1.k<<std::endl<<"L: "<<curve.arc1.L<<std::endl;			
    std::cout<<"xf: "<<curve.arc1.xf<<std::endl<<"yf: "<<curve.arc1.yf<<std::endl<<"thf: "<<curve.arc1.thf<<std::endl;	
    std::cout<<"Arc2: "<<std::endl;
    std::cout<<"x0: "<<curve.arc2.x0<<std::endl<<"y0: "<<curve.arc2.y0<<std::endl<<"th0: "<<curve.arc2.th0<<std::endl;				
    std::cout<<"k: "<<curve.arc2.k<<std::endl<<"L: "<<curve.arc2.L<<std::endl;			
    std::cout<<"xf: "<<curve.arc2.xf<<std::endl<<"yf: "<<curve.arc2.yf<<std::endl<<"thf: "<<curve.arc2.thf<<std::endl;
    std::cout<<"Arc3: "<<std::endl;
    std::cout<<"x0: "<<curve.arc3.x0<<std::endl<<"y0: "<<curve.arc3.y0<<std::endl<<"th0: "<<curve.arc3.th0<<std::endl;				
    std::cout<<"k: "<<curve.arc3.k<<std::endl<<"L: "<<curve.arc3.L<<std::endl;			
    std::cout<<"xf: "<<curve.arc3.xf<<std::endl<<"yf: "<<curve.arc3.yf<<std::endl<<"thf: "<<curve.arc3.thf<<std::endl;			
    std::cout<<"Length: "<<curve.L<<std::endl;    

    cv::Mat out(1470, 970, CV_8UC3, cv::Scalar(0,0,0));
    cv::circle(out, cv::Point(x0,y0), radius , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
    cv::line(out, cv::Point(x0,y0) , cv::Point(x0+radius*cos(th0), y0+radius*sin(th0)) , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0);

    int npts = 1000;
    for(int j=0; j<npts;++j)
    {
        double s = curve.arc1.L/npts*j;
        circline(s,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
        cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );
    }

    cv::line(out, cv::Point(curve.arc2.x0,curve.arc2.y0) , cv::Point(curve.arc2.xf,curve.arc2.yf) , cv::Scalar( 0 , 200 , 0 ), 5);

    for(int j=0; j<npts;++j)
    {
        double s = curve.arc3.L/npts*j;
        circline(s,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
        cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );
    }

    cv::circle(out, cv::Point(xf,yf), radius , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
    cv::line(out, cv::Point(xf,yf) , cv::Point(xf+radius*cos(thf), yf+radius*sin(thf)) , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0);

    std::string name = "Corners";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 512, 640);
    cv::imshow(name.c_str(), out); 
    cv::waitKey(0); 

    return 0;
}