// map_construction.cpp: MAP CONSTRUCTION
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

#include "map_construction.hpp"
#include "map_extraction.hpp"
#include "path.h"

std::vector<cv::Point> Polygon::getContours(cv::Mat map)
{
  std::vector<cv::Point> contours;

  for(int j=0;j<this->corners.size();++j)
  {
    if((j+1)!=this->corners.size())
    {
        cv::LineIterator lit(map, this->corners[j], corners[j+1]);
        contours.reserve(lit.count);
        for (int i = 0; i < lit.count; ++i, ++lit)
        {
            contours.push_back(lit.pos());
        }
    }
    else
    {
        cv::LineIterator lit(map, this->corners[j], corners[0]);
        contours.reserve(lit.count);
        for (int i = 0; i < lit.count; ++i, ++lit)
        {
            contours.push_back(lit.pos());
        }
    }
  }
  
  return contours;
}

cv::Point Robot::getDirectionSidePoint()
{
  double minSide = 1000.0, s;
  int index;

  for(int i=0;i<this->corners.size();++i)
  {
    if((i+1)==this->corners.size())
    {
        s = std::hypot(this->corners[i].x-this->corners[0].x, this->corners[i].y-this->corners[0].y);
    }
    else
    {
        s = std::hypot(this->corners[i].x-this->corners[i+1].x, this->corners[i].y-this->corners[i+1].y);
    }

    if(s<minSide)
    {
        minSide = s;
        index = i;
    }
  }

  // std::cout<<"MinSide "<<minSide<<std::endl;

  if((index+1)==this->corners.size())
  {
      return cv::Point((this->corners[index].x+this->corners[0].x)/2,(this->corners[index].y+this->corners[0].y)/2);
  }else
  {
      return cv::Point((this->corners[index].x+this->corners[index+1].x)/2,(this->corners[index].y+this->corners[index+1].y)/2);
  }
}

void Map::printAll()
{
  for(int i = 0; i<this->obstacles.size();++i)
  {
      std::cout <<"Obstacle "<<i;
      for(int j = 0; j<this->obstacles[i].corners.size();++j)
      {   
          std::cout <<" "<<this->obstacles[i].corners[j];
      }
      std::cout << std::endl;
  }
  std::cout <<"Border ";
  for(int j = 0; j<this->border.corners.size();++j)
  {   
      std::cout <<" "<<this->border.corners[j];
  }
  std::cout << std::endl; 
  for(int i = 0; i<this->victims.size();++i)
  {
      std::cout <<"Victim "<<i<<" "<<this->victims[i].center<<" "<<this->victims[i].radius<<" "<<this->victims[i].index<<std::endl;
  }
  std::cout <<"Gate ";
  for(int j = 0; j<this->gate.corners.size();++j)
  {   
      std::cout <<" "<<this->gate.corners[j];
  }
  std::cout << std::endl; 
  std::cout <<"Robot ";
  for(int j = 0; j<this->robot.corners.size();++j)
  {   
      std::cout <<" "<<this->robot.corners[j];
  }
  std::cout << std::endl; 
}

cv::Mat Map::showMap()
{

  cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));

  for(int i=0;i<this->obstacles.size();++i)
  {
      for(int j=0;j<this->obstacles[i].corners.size();++j)
      {
          if((j+1)!=this->obstacles[i].corners.size())
          {
              cv::circle(out , this->obstacles[i].corners[j], 0.5 , cv::Scalar( 0 , 0 , 250 ), 5, 8, 0 );
              cv::line(out, this->obstacles[i].corners[j] , this->obstacles[i].corners[j+1] , cv::Scalar( 0 , 0 ,150), 5);
          }
          else
          {
              cv::circle(out , this->obstacles[i].corners[j], 0.5 , cv::Scalar( 0 , 0 , 250 ), 5, 8, 0 );
              cv::line(out, this->obstacles[i].corners[j] , this->obstacles[i].corners[0] , cv::Scalar( 0 , 0 ,150), 5);
          }
          
      }
  }

  for(int i=0;i<this->border.corners.size();++i)
  {
      if((i+1)!=this->border.corners.size())
      {
          cv::circle(out , this->border.corners[i], 0.5 , cv::Scalar( 250 , 0 , 0 ), 5, 8, 0 );
          cv::line(out, this->border.corners[i], this->border.corners[i+1], cv::Scalar( 0 , 0 , 250 ), 5);
      }
      else
      {
          cv::circle(out , this->border.corners[i], 0.5 , cv::Scalar( 250 , 0 , 0 ), 5, 8, 0 );
          cv::line(out, this->border.corners[i], this->border.corners[0], cv::Scalar( 0 , 0 , 250 ), 5);
      }
  }

  for(int i=0;i<this->gate.corners.size();++i)
  {
      if((i+1)!=this->gate.corners.size())
      {
          cv::circle(out , this->gate.corners[i], 0.5 , cv::Scalar( 250 , 0 , 0 ), 5, 8, 0 );
          cv::line(out, this->gate.corners[i], this->gate.corners[i+1], cv::Scalar( 250 , 0 , 0 ), 5);
      }
      else
      {
          cv::circle(out , this->gate.corners[i], 0.5 , cv::Scalar( 250 , 0 , 0 ), 5, 8, 0 );
          cv::line(out, this->gate.corners[i], this->gate.corners[0], cv::Scalar( 250 , 0 , 0 ), 5);
      }
  }

  for(int i=0;i<this->victims.size();++i)
  {
      cv::circle(out, this->victims[i].center, this->victims[i].radius , cv::Scalar( 0 , 250 , 0 ), 5, 8, 0 );
  }

  for(int i=0;i<this->robot.corners.size();++i)
  {
      if((i+1)!=this->robot.corners.size())
      {
          cv::circle(out , this->robot.corners[i], 0.5 , cv::Scalar( 250 , 100 , 0 ), 5, 8, 0 );
          cv::line(out, this->robot.corners[i], this->robot.corners[i+1], cv::Scalar( 250 , 200 , 0 ), 5);
      }
      else
      {
          cv::circle(out , this->robot.corners[i], 0.5 , cv::Scalar( 250 , 100 , 0 ), 5, 8, 0 );
          cv::line(out, this->robot.corners[i], this->robot.corners[0], cv::Scalar( 250 , 200 , 0 ), 5);
      }
  }

  showImage("Generated Map", out);

  return out;

}

bool findObstacles(cv::Mat const & map, Map & map_object)
{  
  std::cout << std::endl;
  std::cout << "OBSTACLES -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  std::vector<std::vector<cv::Point>> mc_contours, mc_contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;
  cv::Mat hsv_img;

  // Convert color space from BGR to HSV
  cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);

  // Find red regions: h values around 0 (positive and negative angles)
  cv::Mat red_mask_low, red_mask_high, red_mask;
                              // 0  30 30             20
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(18, 255, 255), red_mask_low);
                              // 150  30 30             180         
  cv::inRange(hsv_img, cv::Scalar(150, 0, 0), cv::Scalar(200, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)); //5,5
  cv::erode(red_mask , red_mask , kernel, cv::Point(-1, -1), 2);

  // Process red mask (Obstacles)
  cv::Mat red_mask_temp = map.clone();
  cv::findContours(red_mask, mc_contours,  cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE); //Find closed contours on the image via the defined approximation
  drawContours(red_mask_temp, mc_contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);  //Draw the found contours 

  int obs_flag=0; //Temporal variable for controlling parameters inside the "for" loop

  for (int i=0; i<mc_contours.size(); ++i) //Area filtering loop (controlled by the number of contours found in the approximation)
  {	
    int red_area=cv::contourArea(mc_contours[i]); // With the measure of the respective contour's area we can dismiss false positives and filter noise
    
    if(red_area>500) //avoiding noise and minimal contours by dismissing smallers sizes compared with a given threshold
    {
    
      obs_flag++ ; // Parameter to control the format of the output in the Terminal and the file
            
      approxPolyDP(mc_contours[i], approx_curve, 18 , true); //How Strict the Filter is to Aproximate the contour to a Polygon
      mc_contours_approx = {approx_curve}; 
      drawContours(red_mask_temp, mc_contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA); //draw Polygon approximations 
      
      for ( int i = 0 ; i < approx_curve.size() ; ++i ) // Coordinates Loop
      {
        std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; //Listing the coordinates of the corners 
        cv::circle( red_mask_temp , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); // Circles Printer for the shape's corners
      }

      Obstacle obs = Obstacle(approx_curve);
      map_object.obstacles.push_back(obs);

    }    
  }

  if (obs_flag==0) 
  {
    return false;
  }
  else
  {
    showImage("Obstacles", red_mask_temp);
    return true;
  }
  
}

bool findBorders(cv::Mat const & map, Map & map_object)
{  
  std::cout << std::endl;
  std::cout << "BORDERS -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  std::vector<std::vector<cv::Point>> mc_contours, mc_contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;
  // cv::Mat hsv_img;
  int bord_flag=0; //Temporal variable for controlling parameters inside the "for" loop

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);

  // Find black regions (filter on saturation and value)
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 5, 5), cv::Scalar(180, 255, 70), black_mask);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
  cv::dilate(black_mask, black_mask, kernel);	
  cv::erode(black_mask, black_mask, kernel);

  // // Convert color space from BGR to grayscale
  // cv::Mat gray_img;
  // cv::cvtColor(map, gray_img, cv::COLOR_BGR2GRAY);

  // // Find image filtered by adaptive threshold 
  // cv::Mat adaptive_mask;
  // cv::adaptiveThreshold(gray_img, adaptive_mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 0);
  // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
  // cv::dilate(adaptive_mask, adaptive_mask, kernel);

  showImage("Adaptive mask", black_mask);

  // Process black mask
  cv::findContours(black_mask, mc_contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob

  contours_img = map.clone();
  drawContours(contours_img, mc_contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  for (int i=0; i<mc_contours.size(); ++i)
  {
      double area = cv::contourArea(mc_contours[i]);
      if (area < MIN_MAP_AREA_SIZE) continue; // filter too small contours to remove false positives
      
      bord_flag++ ; // Parameter to control the format of the output in the Terminal and the file

      approxPolyDP(mc_contours[i], approx_curve, 80, true); // fit a closed polygon (with less vertices) to the given contour
      
      if(approx_curve.size()!=4) continue;

      mc_contours_approx = {approx_curve};
      drawContours(contours_img, mc_contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);

      for ( int i = 0 ; i < approx_curve.size() ; ++i ) // Coordinates Loop
      {
        std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; //Listing the coordinates of the corners 
        cv::circle( contours_img , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); // Circles Printer for the shape's corners
      }

      map_object.border = Border(approx_curve);
  }

  if (bord_flag==0) 
  {
    return false;
  }
  else
  {
    showImage("Border", contours_img);
    return true;
  }
  
}

bool findROI(cv::Mat const & map, Map & map_object)
{
  std::cout << std::endl;
  std::cout << "VICTIMS -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;

  std::vector<cv::Vec3f> mc_circles;
  cv::Mat hsv_img;


  // Convert color space from BGR to HSV
  cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);

  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(55, 70, 75), cv::Scalar(80, 255, 255), green_mask);
 
  HoughCircles( green_mask , mc_circles, cv::HOUGH_GRADIENT, 2, green_mask.rows/10, 40, 20, 85, 95 ); 

  cv::Mat circles_img = map.clone();

  for( int i = 0; i < mc_circles.size(); ++i )
  {
    
    cv::Point center(cvRound(mc_circles[i][0]), cvRound(mc_circles[i][1]));
    int radius = cvRound(mc_circles[i][2]);
    cv::circle( circles_img, center, radius, cv::Scalar(0, 0 ,255), -1, 8, 0 );
    std::cout << "  Coordinates of the center " << i+1  << ": " << center << std::endl; 
    Victim vict = Victim(cv::Point(round(mc_circles[i][0]),round(mc_circles[i][1])), mc_circles[i][2]);
    map_object.victims.push_back(vict);

  }

  if (mc_circles.size()<=0) 
  {
    return false;
  }
  else
  {
    showImage("Circles Detected", circles_img);
    return true;

  }

}

bool findGate(cv::Mat const & map, Map & map_object)
{
  std::cout << std::endl;
  std::cout << "GATE -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;
  
  std::vector<std::vector<cv::Point>> mc_contours, mc_contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;  
  cv::Mat hsv_img;

  // Convert color space from BGR to HSV
  cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);
  
  // Find blue regions
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(100, 150, 150), cv::Scalar(130, 255, 200), blue_mask);

  // Process blue mask (Gates)
  cv::Mat blue_mask_temp;
  blue_mask_temp = map.clone();
  cv::findContours(blue_mask, mc_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(blue_mask_temp, mc_contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  
  cv::Mat blue_mask_2;
  blue_mask_2 = blue_mask.clone(); 
  
  int gate_flag =0;
  int gate_count=0;
  for (int i=0; i<mc_contours.size(); ++i) //Dimension Filtering
  {
    int blue_area = cv::contourArea(mc_contours[i]); //Area variable for current selection
      
    if (blue_area >100) //Area Threshold
    {  
	    gate_flag++;//Local variable for controlling the "Gates detected trigger"
	     	
	    approxPolyDP(mc_contours[i], approx_curve, 10, true); //Polygonal Approach for the detected contour
	    mc_contours_approx = {approx_curve};
	    drawContours(blue_mask_temp, mc_contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA); //Gate Shape Output to viewer   
	     
      for ( int i = 0 ; i < approx_curve.size() ; ++i ) //Loop for listing the Gate's properties
      {
        std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; //Listing the coordinates of the corners 
        cv::circle(blue_mask_temp , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); //Graphical Print of the Corners
      }

      map_object.gate = Gate(approx_curve);

    }
  }
 
  if (gate_flag==0) 
  {
    return false;
  }
  else
  {
    showImage("Gate(s) Boundaries", blue_mask_temp);
    return true;
  }

}

bool findRobot(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object)
{
  std::cout << std::endl;
  std::cout << "ROBOT -------------------------------------------------------------" << std::endl;
  std::cout << std::endl;
  
  // cv::Mat ground_map;
  // ground_map = map.clone(); 
  // cv::warpPerspective(robot_plane, ground_map, map_object.toMap, cv::Size(MAP_LENGTH,MAP_WIDTH));

  std::vector<std::vector<cv::Point>> mc_contours, mc_contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;  
  cv::Mat hsv_img;

  // Convert color space from BGR to HSV
  cv::cvtColor(robot_plane, hsv_img, cv::COLOR_BGR2HSV);
  
  // Find robot
  cv::Mat robot_mask;
  cv::inRange(hsv_img, cv::Scalar(80, 150, 150), cv::Scalar(100, 200, 200), robot_mask);

  // Process robot mask
  cv::Mat robot_mask_temp;
  robot_mask_temp = robot_plane.clone();
  cv::findContours(robot_mask, mc_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(robot_mask_temp, mc_contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  

    
  int robot_flag =0;
  int robot_count=0;
  for (int i=0; i<mc_contours.size(); ++i) //Dimension Filtering
  {
    int robot_area = cv::contourArea(mc_contours[i]); //Area variable for current selection
      
    if (robot_area >100) //Area Threshold
    {  
	    robot_flag++;//Local variable for controlling the "Gates detected trigger"
	     	
	    //std::cout << (i+1) << ") Contour size: " << mc_contours[i].size() << std::endl;
	    approxPolyDP(mc_contours[i], approx_curve, 10, true); //Polygonal Approach for the detected contour
	    mc_contours_approx = {approx_curve};
	    drawContours(robot_mask_temp, mc_contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA); //Gate Shape Output to viewer 
	     
    for ( int i = 0 ; i < approx_curve.size() ; ++i ) //Loop for listing the Gate's properties
    {
      std::cout << "  Coordinates of the corner nr." << i+1  << ": " << approx_curve[i] << std::endl; //Listing the coordinates of the corners 
      cv::circle(robot_mask_temp , cv::Point(approx_curve[i]), 0.5 , cv::Scalar( 0, 0, 255 ), 5, 8 ); //Graphical Print of the Corners
    }

     map_object.robot = Robot(approx_curve);
     map_object.robot.pose.x = map_object.robot.getCenter().x;
     map_object.robot.pose.y = map_object.robot.getCenter().y;
     map_object.robot.pose.theta = getOrientation(map_object.robot.getDirectionSidePoint(), map_object.robot.getCenter());

    }
  }
 
  if (robot_flag==0) 
  {
    return false;
  }
  else
  {
    showImage("Robot Outline", robot_mask_temp);
    return true; 
  }
}

double getOrientation(cv::Point end_point, cv::Point start_point)
{
  return atan2(end_point.y-start_point.y,end_point.x-start_point.x);
}


bool buildMap(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object)
{

  //Loop over each pixel and create a point
  // for (int x = 0; x < map.cols; x++)
  //     for (int y = 0; y < map.rows; y++)
  //         mapPoints.push_back(cv::Point(x, y));

  // for (int x = 0; x < robotPlane.cols; x++)
  //   for (int y = 0; y < robotPlane.rows; y++)
  //       robotPlanePoints.push_back(cv::Point(x, y));
  
  // transform = cv::findHomography(robotPlanePoints, mapPoints);
  // cv::warpPerspective(robotPlane, map, transform, cv::Size(MAP_LENGTH,MAP_WIDTH));
  
  // Display original image
  showImage("Original map", map);

  if(!findObstacles(map, map_object))
  {
    std::cout << "No Obstacles Detected!" << std::endl;
  }

  if(!findBorders(map, map_object))
  {
    std::cout << "No Borders Detected!" << std::endl;
  }

  if(!findROI(map, map_object))
  {
    std::cout << "No ROIs Detected!" << std::endl;
  }

  if(!findGate(map, map_object))
  {
    std::cout << "No Gate Detected!" << std::endl;
  }

  if(!findRobot(map, robot_plane, map_object))
  {
    std::cout << "No Robot Detected!" << std::endl;
  }

  map_object.printAll();

  return true;

}
