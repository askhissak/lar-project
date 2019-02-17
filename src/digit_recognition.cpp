// digit_recognition.cpp: DIGIT RECOGNITION
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
#include "map_construction.hpp"

#include <chrono>


bool DR_developer_session = false ; // if true  -> Retrieves desired debugging and log content 
								 // if false -> Process everything without graphical output 
								 
int approximation = 3 ;  // if 1  -> Use just Tesseract 
						 // if 2  -> Use just Template Matching   
						 // if 3  -> Use Tesseract and if there's no useful results use Template Matching 
						 // if 4  -> Use Template Matching and if there's no useful results use Tesseract 
								 
using namespace std::chrono;

//Properly rotate an image
cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point pt(src.cols/2., src.rows/2.);  
 
    cv::Mat r = cv::getRotationMatrix2D(pt, -angle, 1.0);
    
    cv::Rect2f bbox = cv::RotatedRect(cv::Point(), src.size(), angle).boundingRect2f();

    r.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    r.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;
       
    warpAffine(src, dst, r, bbox.size());
    return dst;
}

// void printAngle(cv::RotatedRect calculatedRect){
//     if(calculatedRect.size.width < calculatedRect.size.height){
//         printf("Angle along longer side: %7.2f\n", calculatedRect.angle+180);
//     }else{
//         printf("Angle along longer side: %7.2f\n", calculatedRect.angle+90);
//     }
// }

bool useTesseract(cv::Mat const & map, std::vector<Victim> victims, int* index)
{
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);
    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(55, 70, 75), cv::Scalar(80, 255, 255), green_mask); //75
    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(green_mask, green_mask, kernel);	
    cv::erode(green_mask, green_mask, kernel);
    // cv::erode(green_mask, green_mask, kernel);
    cv::Mat green_mask_inv, filtered(map.rows, map.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask  
    
    map.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

    // Create Tesseract object
    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    // Initialize tesseract to use English (eng) 
    ocr->Init(NULL, "eng");
    // Set Page segmentation mode to PSM_SINGLE_CHAR (10)
    ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
    // Only digits are valid output characters
    ocr->SetVariable("tessedit_char_whitelist", "012345");
    std::cout << "Size: " << victims.size() << std::endl<<std::endl<<std::endl;

    // For each green blob in the original image containing a digit
    for (int i=0; i<victims.size(); ++i)
    {
        //Find bounding box for a ROI
        cv::Rect bbox(victims[i].center.x-victims[i].radius,victims[i].center.y-victims[i].radius,2*victims[i].radius,2*victims[i].radius);
    
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
        // cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        // cv::erode(processROI, processROI, kernel);

        // Show the actual image passed to the ocr engine
        if (DR_developer_session == true) cv::imshow("ROI", processROI);
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
            if(*recognizedDigits[k] == ' ' || !isdigit(*recognizedDigits[k])) continue;

            if(confidences[k]>maxConfidence)
            {
                maxConfidence = confidences[k]; 
                maxIndex = k;
                
            }
            
           
        }
        
        if(*recognizedDigits[maxIndex] == ' ' || !isdigit(*recognizedDigits[maxIndex])) continue;
        
        std::cout << " Recognized digit: " << std::string(recognizedDigits[maxIndex]) << std::endl ;
        std::cout << " Confidence: " << maxConfidence << std::endl ;
        
        
        if ( *recognizedDigits[maxIndex] == '7' ) {index[i] = std::stoi("1");std::cout << " Saved: " << index[i] << "\n\n " <<std::endl;}
        else index[i] = std::stoi(std::string(recognizedDigits[maxIndex]));
		
		
        // if(maxConfidence<75) return false;
        
    }

    ocr->End(); // destroy the ocr object (release resources)

    return true;
}

bool useTemplateMatching(cv::Mat const & map, std::vector<Victim> victims, int* index)
{
	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	
    // Display original image
    if (DR_developer_session == true) showImage("Original", map);
  
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);
    
    
    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(55, 70, 75), cv::Scalar(80, 255, 255), green_mask);
    
    // Apply some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(green_mask, green_mask, kernel);
    cv::erode(green_mask, green_mask, kernel);
    
    // Display image
    if (DR_developer_session == true) showImage("GREEN_filter", green_mask);  
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    contours_img = map.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
        
    std::vector<cv::Rect> boundRect(contours.size());
    // std::vector<cv::RotatedRect> boundRotRect(contours.size()); 
    for (int i=0; i<contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
        approxPolyDP(contours[i], approx_curve, 2, true);
        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
        boundRect[i] = boundingRect(cv::Mat(approx_curve)); // find bounding box for each green blob
        // boundRotRect[i] = minAreaRect(approx_curve);
    }
    if (DR_developer_session == true) showImage("Original", contours_img);        
    
    cv::Mat green_mask_inv, filtered(map.rows, map.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    
    if (DR_developer_session == true) showImage("Numbers", green_mask_inv);
    
    // Load digits template images
    std::vector<cv::Mat> templROIs;
    for (int i=0; i<=9; ++i) {
        templROIs.emplace_back(cv::imread("data/template/" + std::to_string(i) + ".png"));
    }  
    
    map.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes
    
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
    
    // For each green blob in the original image containing a digit
    for (int i=0; i<victims.size(); ++i)
    {
        //Find bounding box for a ROI
        cv::Rect bbox(victims[i].center.x-victims[i].radius,victims[i].center.y-victims[i].radius,2*victims[i].radius,2*victims[i].radius);
    
        // extract the ROI containing the digit 
        cv::Mat processROI(filtered, bbox); 
        cv::Mat rotatedROI(filtered, bbox);
        
        // cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
        // cv::Mat rotatedROI(filtered, boundRect[i]);

        if (processROI.empty()) continue;
        
        cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
        cv::threshold( processROI, processROI, 60, 255, 0 ); // threshold and binarize the image, to suppress some noise
        
        // Apply some additional smoothing and filtering
        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);
        
        // Show the actual image used for the template matching
        if (DR_developer_session == true) showImage("ROI", processROI);

        // Convert color space from BGR to grayscale
        cv::Mat grayROI, grayTemplROI;
        cv::cvtColor(processROI, grayROI, cv::COLOR_BGR2GRAY);

        double maxScore = 1;
        int maxIdx = -1;

        for (int j=0; j<templROIs.size(); ++j) 
        {
            cv::cvtColor(templROIs[j], grayTemplROI, cv::COLOR_BGR2GRAY);
            double score = cv::matchShapes(grayROI,grayTemplROI,2,0.0);
            // cv::Mat result;
            // cv::matchTemplate(rotatedROI, templROIs[j], result, cv::TM_CCOEFF);
            // double score;
            // cv::minMaxLoc(result, nullptr, &score); 
            std::cout<<"Score "<<score<<std::endl;
            if (score < maxScore) {
                maxScore = score;
                maxIdx = j;
            }
        }

        //Rotate the ROI to recognize a digit
        for(int j=0;j<36;++j)
        {
            rotatedROI = rotate(processROI, 10*j);
            //showImage("Rotated ROI", rotatedROI);

            for (int j=0; j<templROIs.size(); ++j) {
                cv::Mat result;
                cv::matchTemplate(rotatedROI, templROIs[j], result, cv::TM_CCOEFF);
                double score;
                cv::minMaxLoc(result, nullptr, &score); 
                if (score > maxScore) {
                    maxScore = score;
                    maxIdx = j;
                }
            }
        }
		
		if (maxIdx == 7) maxIdx = 1;
		
        index[i] = maxIdx;
    
        std::cout << "Best fitting template: " << maxIdx << std::endl;
        //if(maxScore>0.01) return false;
     
    }
    
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

	auto duration = duration_cast<microseconds>( t2 - t1 ).count();

	std::cout << "\n\n PROCESSING TIME : " << duration/1000 << "MILISECONDS \n\n " ;

    return true;
}

bool recognizeDigits(cv::Mat const & map, std::vector<Victim> victims, std::vector<int> & order)
{
	int index[victims.size()] = {0};
    
    bool change_approach = false;
    
    int next = 1 ;
    
    switch(approximation)
    
    {
		case 1 : //Just Tesseract
		
			if(!useTesseract(map, victims , index))
     		{
				std::cerr << "(Critical) Failed to recognize the digits using Tesseract!" << std::endl;
				return false;
			}
			
		break;
		
		case 2 : //Just Template Matching
		
			if(!useTemplateMatching(map, victims, index))
			{
				std::cerr << "(Critical) Failed to recognize the digits using template matching!" << std::endl;         
				return false;
			}
		
		break;
		
		case 3 : //Tesseract and if not succeed then Template Matching
		
			if(!useTesseract(map, victims , index))
     		{
				std::cerr << "(Critical) Failed to recognize the digits using Tesseract, changing to Template Matching!" << std::endl;
				change_approach = true;
			}
			
			if (change_approach == true)
			{
				if(!useTemplateMatching(map, victims, index))
				{
					std::cerr << "(Critical) Failed to recognize the digits using Tesseract and also Template Matching!" << std::endl;         
					return false;
				}
			} 
			
		break;
		
		case 4 : //Template Matching and if not succeed then Tesseract
		
			if(!useTemplateMatching(map, victims, index))
			{
				std::cerr << "(Critical) Failed to recognize the digits using template matching, changing to Tesseract!" << std::endl;         
				change_approach = true ;
			}
			
			if (change_approach == true)
			{
				if(!useTesseract(map, victims , index))
				{
					std::cerr << "(Critical) Failed to recognize the digits using Template Matching and also Tesseract!" << std::endl;
					return false;
				}
			
			}
			
		break;
	}

	for(int i=0;i<victims.size();++i)
	{
		for(int j=0;j<victims.size();++j)
		{ 
			if(index[j]==next) order.push_back(j); 
		}
		next++;
	}
    

    return true;
                
}
