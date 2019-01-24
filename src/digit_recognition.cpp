// digit_recognition.cpp:
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include "digit_recognition.hpp"
#include "map_extraction.hpp"

//DIGIT RECOGNITION
int* recognizeDigits(cv::Mat const & img, std::vector<cv::Vec3f> circles)
{

    // int orderedArray[4] = {0};//hardcoded

    // Load image from file
    // cv::Mat img = cv::imread(filename.c_str());
    // if(img.empty()) {
    //     throw std::runtime_error("Failed to open the file " + filename);
    // }
    
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(55, 70, 75), cv::Scalar(75, 255, 255), green_mask); //75
    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(green_mask, green_mask, kernel);	
    cv::erode(green_mask, green_mask, kernel);
    // cv::erode(green_mask, green_mask, kernel);
    cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask  
    
    img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

    // Create Tesseract object
    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    // Initialize tesseract to use English (eng) 
    ocr->Init(NULL, "eng");
    // Set Page segmentation mode to PSM_SINGLE_CHAR (10)
    ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
    // Only digits are valid output characters
    ocr->SetVariable("tessedit_char_whitelist", "0123456789");
    
    // For each green blob in the original image containing a digit
    for (int i=0; i<circles.size(); ++i)
    {
        //Find bounding box for a ROI
        cv::Rect bbox(circles[i][0]-circles[i][2],circles[i][1]-circles[i][2],2*circles[i][2],2*circles[i][2]);
    
        // extract the ROI containing the digit 
        cv::Mat processROI(filtered, bbox); 
        cv::Mat rotatedProcessROI(filtered, bbox);
        
        if (processROI.empty()) continue;
        
        cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);
        cv::threshold( processROI, processROI, 60, 255, cv::THRESH_BINARY_INV ); // threshold and binarize the image, to suppress some noise

        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);

        // Show the actual image passed to the ocr engine
        cv::imshow("ROI", processROI);
        rotatedProcessROI = processROI;
        std::vector<char *> recognizedDigits(36);
        std::vector<int> confidences(36);
        int maxConfidence = 0, maxIndex;

        //Rotate the ROI to recognize a digit
        for(int j=0;j<36;++j)
        {
            ocr->SetImage(rotatedProcessROI.data, rotatedProcessROI.cols, rotatedProcessROI.rows, 3, rotatedProcessROI.step);
            recognizedDigits[j] = ocr->GetUTF8Text();
            confidences[j] = ocr->MeanTextConf();
            rotatedProcessROI = rotate(processROI, 10*j);

        }

        //Find the digit with the highest confidence
        for(int k=0;k<recognizedDigits.size();++k)
        {
            if(*recognizedDigits[k] == ' ') continue;

            if(confidences[k]>maxConfidence)
            {
                maxConfidence = confidences[k];
                maxIndex = k;
            }
        }

        if(*recognizedDigits[maxIndex] == ' ') continue;
        std::cout << "Recognized digit: " << std::string(recognizedDigits[maxIndex]);
        std::cout << "Confidence: " << maxConfidence << std::endl<<std::endl<<std::endl;
        orderedArray[i] = std::stoi(std::string(recognizedDigits[maxIndex]));
        
        cv::waitKey(0);
    }

    ocr->End(); // destroy the ocr object (release resources)

    return orderedArray;
                
}