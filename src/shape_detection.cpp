// shape_detection.cpp:
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "shape_detection.hpp"

//SHAPE DETECTION
std::vector<cv::Vec3f> processImage(cv::Mat const & img)
{
  std::vector<cv::Vec3f> circles;

  // Load image from file
  // cv::Mat img = cv::imread(filename.c_str());
  // if(img.empty()) 
  // {
  //   throw std::runtime_error("Failed to open the file " + filename);
  // }
  
  // Display original image
  std::string name = "Original";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), img);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str()); 

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  // Display HSV image
  name = "HSV Transformation";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), hsv_img);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str());

  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;
  
 /////////////////////////////////////////// R E D /////////////////////////////////////////////////////////////

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << " O B S T A C L E S ----------------------------------------------------" << std::endl;
  std::cout << std::endl;


  // Find red regions: h values around 0 (positive and negative angles)
  cv::Mat red_mask_low, red_mask_high, red_mask;
 
                               // 0  30 30             20
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(18, 255, 255), red_mask_low);
                               // 150  30 30             180         
  cv::inRange(hsv_img, cv::Scalar(150, 0, 0), cv::Scalar(200, 255, 255), red_mask_high);
  
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks
  
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)); //5,5
  
  cv::erode(red_mask , red_mask , kernel, cv::Point(-1, -1), 2);
  
  name = "Obstacles";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), red_mask);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str());        

  // Process red mask (Obstacles)
  cv::Mat red_mask_temp = img.clone();
  cv::findContours(red_mask, contours,  cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE); //Find closed contours on the image via the defined approximation
  
  drawContours(red_mask_temp, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);  //Draw the found contours 
  
   
  
  
  
  int obs_flag=0; //Temporal variable for controlling parameters inside the "for" loop
 
  std::ofstream ofs; //new

  ofs.open("data/output/output.txt"); //new


  for (int i=0; i<contours.size(); ++i) //Area filtering loop (controlled by the number of contours found in the approximation)
  {	
    int red_area=cv::contourArea(contours[i]); // With the measure of the respective contour's area we can dismiss false positives and filter noise
    
    if(red_area>500) //avoiding noise and minimal contours by dismissing smallers sizes compared with a given threshold
    {
     
     obs_flag++ ; // Parameter to control the format of the output in the Terminal and the file
     
     if (obs_flag == 1) {std::cout << " "  << contours.size() << "Obstacles Detected! " << std::endl;} //Triggering the detection alert in the first event 
     if (obs_flag >= 1 ) {std::cout << std::endl;}
     
     approxPolyDP(contours[i], approx_curve, 18 , true); //How Strict the Filter is to Aproximate the contour to a Polygon
     contours_approx = {approx_curve}; 
     drawContours(red_mask_temp, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA); //draw Polygon approximations 
     
     
     switch (approx_curve.size())  
     {
       case 3:  std::cout << " The Obstacle number " << obs_flag << " has 3 sides so it's a triangle ! "      << std::endl; break;  //
       case 4:  std::cout << " The Obstacle number " << obs_flag << " has 4 sides so it's a rectangle ! "     << std::endl; break;  //
       case 5:  std::cout << " The Obstacle number " << obs_flag << " has 5 sides so it's a pentagon ! "      << std::endl; break;  //
       case 6:  std::cout << " The Obstacle number " << obs_flag << " has 6 sides so it's an hexagon ! "      << std::endl; break;  // Layout Printer
       case 7:  std::cout << " The Obstacle number " << obs_flag << " has 7 sides so it's an heptagon ! "     << std::endl; break;  // for Obstacle Id tag     
       case 8:  std::cout << " The Obstacle number " << obs_flag << " has 8 sides so it's an octagon ! "      << std::endl; break;  // and sides number
       case 9:  std::cout << " The Obstacle number " << obs_flag << " has 9 sides so it's an nonagon ! "      << std::endl; break;  // 
       default: std::cout << " The Obstacle number " << obs_flag << " has " << approx_curve.size() << " sides" << std::endl; break;  //   
     }
  
     
     for ( int i = 0 ; i < approx_curve.size() ; ++i ) // Coordinates Loop
     {
   
      std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; // Obstacle's Corners Labeler

	if (i ==0){ ofs << "0 " << obs_flag-1  << " " << approx_curve.size() << " " << approx_curve[i].x << " " << approx_curve[i].y; } //new

	else{ofs << " " << approx_curve[i].x << " " << approx_curve[i].y ;} //new

      cv::circle( red_mask_temp , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); // Circles Printer for the shape's corners
      
     }
  
     ofs << "\n" ; //new

     name = "Obstacle(s) Boundaries";
     cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
     cv::resizeWindow(name.c_str(), 640, 512);
     cv::imshow(name.c_str(), red_mask_temp);
     //cv::waitKey(0);
     cv::destroyWindow(name.c_str());   

    }
    
    
  }

  if (obs_flag=0) {std::cout << "No Obstacles Detected!" << std::endl;}

 /////////////////////////////////////////// G R E E N /////////////////////////////////////////////////////////////

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << " R E G I O N S  O F  I N T E R E S T-------------------------------------" << std::endl;
  std::cout << std::endl;


  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(55, 70, 75), cv::Scalar(80, 255, 255), green_mask);
  
  name = "Regions of interest";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), green_mask);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str());     
 
  HoughCircles( green_mask , circles, cv::HOUGH_GRADIENT, 1, green_mask.rows/10, 40, 20 );
    
  std::cout << std::endl;
  
  std::cout << " There are " <<circles.size() << " Regions of Interest !" << std::endl;
 

  cv::Mat circles_img = img.clone();

  for( int i = 0; i < circles.size(); ++i )
  {
    
   cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
   
   //int radius = cvRound(circles[i][2]);
   
   // circle center
   circle( circles_img, center, 10, cv::Scalar(0, 250 ,255), -1, 8, 0 );
   
   // circle outline
   //circle( circles_img, center, radius, Scalar(0,0,255), 3, 8, 0 );
   
   std::cout << "  Region number " << (i+1) << " is at coordinates : [" <<  round(circles[i][0])  <<  ", " << round(circles[i][1]) << "] " <<std::endl;

   ofs << "2 " << i  << " 2 " << round(circles[i][0]) << " " << round(circles[i][1]) <<  "\n" ; //new



   name = "Circles Detected";
   cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
   cv::resizeWindow(name.c_str(), 640, 512);
   cv::imshow(name.c_str(), circles_img);
   //cv::waitKey(0);
   cv::destroyWindow(name.c_str());   

  }

 /////////////////////////////////////////// B L U E /////////////////////////////////////////////////////////////

  //Gate Finder

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "G A T E S -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;
  // Find blue regions
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(100, 150, 150), cv::Scalar(130, 255, 200), blue_mask);

  name = "Gate";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), blue_mask);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str()); 

  // Process blue mask (Gates)
  cv::Mat blue_mask_temp;
  blue_mask_temp = img.clone();
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(blue_mask_temp, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  
  
  
  int gate_flag =0;
  int gate_count=0;
  for (int i=0; i<contours.size(); ++i) //Dimension Filtering
  {
    int blue_area = cv::contourArea(contours[i]); //Area variable for current selection
      
    if (blue_area >100) //Area Threshold
    {
       
     gate_flag++;//Local variable for controlling the "Gates detected trigger"
     
     if(gate_flag==1) {std::cout << std::endl; std::cout << "Gates Detected !" << std::endl;std::cout << std::endl;} //Terminal Printing

     //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
     approxPolyDP(contours[i], approx_curve, 10, true); //Polygonal Approach for the detected contour
     contours_approx = {approx_curve};
     drawContours(blue_mask_temp, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA); //Gate Shape Output to viewer
     

     switch (approx_curve.size())  
      {
    	case 3:  std::cout << " The Gate number " << gate_flag << " has 3 sides so it's a triangle ! "      << std::endl; break; //
    	case 4:  std::cout << " The Gate number " << gate_flag << " has 4 sides so it's a rectangle ! "     << std::endl; break; //
    	case 5:  std::cout << " The Gate number " << gate_flag << " has 5 sides so it's a pentagon ! "      << std::endl; break; // Categorizing shape by it's 
    	case 6:  std::cout << " The Gate number " << gate_flag << " has 6 sides so it's an hexagon ! "      << std::endl; break; // sides number
    	case 7:  std::cout << " The Gate number " << gate_flag << " has 7 sides so it's an heptagon ! "     << std::endl; break; //     
    	case 8:  std::cout << " The Gate number " << gate_flag << " has 8 sides so it's an octagon ! "      << std::endl; break; //
    	case 9:  std::cout << " The Gate has 9 sides so it's an nonagon ! "      << std::endl; break;
    	default: std::cout << " Number of Sides of the Gate " << i+1 << " : "<< approx_curve.size() << std::endl; break;     
      }     
     
     for ( int i = 0 ; i < approx_curve.size() ; ++i ) //Loop for listing the Gate's properties
     {
   
      std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; //Listing the coordinates of the corners 

      if (i==0){ofs << "1 " << i << " " << approx_curve.size() << " " << approx_curve[i].x << " " << approx_curve[i].y ;} //new
      else {ofs << " " << approx_curve[i].x << " " << approx_curve[i].y;} //new

      cv::circle(blue_mask_temp , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); //Graphical Print of the Corners
      
     }

    }
  }
 
  if(gate_flag==0) {std::cout << "No Gates Detected !" << std::endl;} // "No gates Dectected" Case
  std::cout << std::endl;
  
  name = "Gate(s) Boundaries";
  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(name.c_str(), 640, 512);
  cv::imshow(name.c_str(), blue_mask_temp);
  //cv::waitKey(0);
  cv::destroyWindow(name.c_str());    
  
  ofs.close(); //new
  
  return circles;

}
