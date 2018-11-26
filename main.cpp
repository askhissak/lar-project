//main.cpp
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "camera_calibration.hpp"
#include "map_extraction.hpp"
#include "shape_detection.hpp"
#include "digit_recognition.hpp"
#include "order_roi.hpp"
#include "reader.hpp"

int main(int argc, char* argv[])
{
    //MENU
    // std::cout << "Please choose an option: " << std::endl;
    if (argc != 2) 
    {
        std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
        return 0;
    }

    std::string filename = argv[1];

    // calibrateCamera();//+

    std::string outputFilename = extractMap(filename);//+

    std::vector<cv::Vec3f> circles = processImage(outputFilename);

    int *pointer = recognizeDigits(outputFilename, circles);

    orderROI(outputFilename, circles, pointer);//+

    readData("data/output/output.txt");

    return 0;
}