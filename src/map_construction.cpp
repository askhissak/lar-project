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

bool MC_developer_session = true ; // if true  -> Retrieves desired debugging and log content 
								    // if false -> Process everything without graphical output 


const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
int track_low_H = 0, track_low_S = 0, track_low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int track_high_H = max_value_H, track_high_S = max_value, track_high_V = max_value;

cv::String original_image = "Original Image";
cv::String filtered_image = "Filtered Image";

int color = 0 ;

int Obs_H_Low_1  = 0;   int Obs_H_High_1 = 18;
    
int Obs_S_Low_1  = 0;   int Obs_S_High_1 = 255;

int Obs_V_Low_1  = 0;   int Obs_V_High_1 = 255;


int Obs_H_Low_2  = 150; int Obs_H_High_2 = 200;

int Obs_S_Low_2  = 0;   int Obs_S_High_2 = 255;

int Obs_V_Low_2  = 0;   int Obs_V_High_2 = 255;


int Gate_H_Low  = 100; 	int Gate_H_High = 130;

int Gate_S_Low  = 150; 	int Gate_S_High = 255;

int Gate_V_Low  = 111; 	int Gate_V_High = 200;



int ROI_H_Low  = 55; 	int ROI_H_High = 80;

int ROI_S_Low  = 70; 	int ROI_S_High = 255;

int ROI_V_Low  = 75; 	int ROI_V_High = 255;



int Robot_H_Low  = 80; 	int Robot_H_High = 100;

int Robot_S_Low  = 150; int Robot_S_High = 200;

int Robot_V_Low  = 119; int Robot_V_High = 200;


int Black_H_Low  = 0; 	int Black_H_High = 180;

int Black_S_Low  = 5; 	int Black_S_High = 255;

int Black_V_Low  = 5; 	int Black_V_High = 70;


static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = cv::min(high_H-1, low_H);
    setTrackbarPos("Low H", filtered_image, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = cv::max(high_H, low_H+1);
    setTrackbarPos("High H", filtered_image, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = cv::min(high_S-1, low_S);
    setTrackbarPos("Low S", filtered_image, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = cv::max(high_S, low_S+1);
    setTrackbarPos("High S", filtered_image, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = cv::min(high_V-1, low_V);
    setTrackbarPos("Low V",filtered_image, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = cv::max(high_V, low_V+1);
    setTrackbarPos("High V", filtered_image, high_V);
}

int HSV_Calib(cv::Mat original_img , cv::Mat hsv_img , int red_mask_low1 , int red_mask_high1 , int red_mask_low2 , int red_mask_high2 , int greenmask_low , int greenmask_high , int blue_mask_low , int blue_mask_high)
{
    
    cv::resize(original_img, original_img , cv::Size(640, 400));
    cv::resize(hsv_img, hsv_img , cv::Size(640, 400));
    
    
    while(color < 6)
    {
		low_H = 0, low_S = 0, low_V = 0;
		high_H = max_value_H, high_S = max_value, high_V = max_value;
		
		switch(color)
		{
			case 0 : filtered_image = " OBSTACLES (RED) First Zone "  ; break;
			case 1 : filtered_image = " OBSTACLES (RED) Second Zone " ; break;
			case 2 : filtered_image = " GATE (DARK BLUE) "            ; break;
			case 3 : filtered_image = " REGION OF INSTEREST (GREEN) " ; break;
			case 4 : filtered_image = " ROBOT (LIGHT BLUE) "          ; break;
			case 5 : filtered_image = " BLACK REGIONS "              ; break;
		}
    
			
		cv::namedWindow(original_image);
		cv::namedWindow(filtered_image);
		// Trackbars to set thresholds for HSV values
	
		
		cv::Mat hsv_filtered;
		
		bool first_values = true ;
		
		
		while (color == 5) // Black Regions
		{  
			
			if(first_values == true)
			{
				
				
				low_H = Black_H_Low ;
				low_S = Black_S_Low ;
				low_V = Black_V_Low ;
		
				high_H = Black_H_High;
				high_S = Black_S_High;
				high_V = Black_V_High;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("Low H", filtered_image, &low_H, max_value_H, on_low_H_thresh_trackbar);
				createTrackbar("Low S", filtered_image, &low_S, max_value, on_low_S_thresh_trackbar);
				createTrackbar("Low V", filtered_image, &low_V, max_value, on_low_V_thresh_trackbar);
		
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				
				
				first_values = false ;
			}
		
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);

			
			cv::imshow(original_image, original_img);
					
			cv::imshow(filtered_image, hsv_filtered);
			
			
			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				Black_H_Low  = low_H ;  Black_H_High = high_H;
		
				Black_S_Low  = low_S ;  Black_S_High = high_S;
		
				Black_V_Low  = low_V ;  Black_V_High = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"\n Values of Filters for the Robot : " << " H_LOW: " << Black_H_Low <<  " H_HIGH : " << Black_H_High << std::endl ;
				std::cout <<"                                 " << " S_LOW: " << Black_S_Low <<  " S_HIGH : " << Black_S_High << std::endl ;
				std::cout <<"                                 " << " V_LOW: " << Black_V_Low <<  " V_HIGH : " << Black_V_High << std::endl ;
				
				color++;
				
				break;
				
			}
		}
		
		while (color == 4) // ROBOT 
		{  
			
			if(first_values == true)
			{
				
				low_H = Robot_H_Low ;
				low_S = Robot_S_Low ;
				low_V = Robot_V_Low ;
		
				high_H = Robot_H_High;
				high_S = Robot_S_High;
				high_V = Robot_V_High;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("Low H", filtered_image, &low_H, max_value_H, on_low_H_thresh_trackbar);
				createTrackbar("Low S", filtered_image, &low_S, max_value, on_low_S_thresh_trackbar);
				createTrackbar("Low V", filtered_image, &low_V, max_value, on_low_V_thresh_trackbar);
		
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				first_values = false ;
			}
		
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);
			
			
			cv::imshow(original_image, original_img);
			
			cv::imshow(filtered_image, hsv_filtered);

			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				Robot_H_Low  = low_H ;  Robot_H_High = high_H;
		
				Robot_S_Low  = low_S ;  Robot_S_High = high_S;
		
				Robot_V_Low  = low_V ;  Robot_V_High = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"\n Values of Filters for the Robot : " << " H_LOW: " << Robot_H_Low <<  " H_HIGH : " << Robot_H_High << std::endl ;
				std::cout <<"                                 " << " S_LOW: " << Robot_S_Low <<  " S_HIGH : " << Robot_S_High << std::endl ;
				std::cout <<"                                 " << " V_LOW: " << Robot_V_Low <<  " V_HIGH : " << Robot_V_High << std::endl ;
				
				color++;
				
				break;
				
			}
		}
		
		
		while (color == 3) // REGION OF INTEREST  
		{  
			
			if(first_values == true)
			{
				low_H = ROI_H_Low ;
				low_S = ROI_S_Low ;
				low_V = ROI_V_Low ;
		
				high_H = ROI_H_High;
				high_S = ROI_S_High;
				high_V = ROI_V_High;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("Low H", filtered_image, &low_H, max_value_H, on_low_H_thresh_trackbar);
				createTrackbar("Low S", filtered_image, &low_S, max_value, on_low_S_thresh_trackbar);
				createTrackbar("Low V", filtered_image, &low_V, max_value, on_low_V_thresh_trackbar);
		
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				first_values = false ;
			}
		
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);
			
			
			cv::imshow(original_image, original_img);
			
			cv::imshow(filtered_image, hsv_filtered);
			
			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				ROI_H_Low  = low_H ;  ROI_H_High = high_H;
		
				ROI_S_Low  = low_S ;  ROI_S_High = high_S;
		
				ROI_V_Low  = low_V ;  ROI_V_High = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"\n Values of Filters for the ROIs : " << " H_LOW: " << ROI_H_Low <<  " H_HIGH : " << ROI_H_High << std::endl ;
				std::cout <<"                                 " << " S_LOW: " << ROI_S_Low <<  " S_HIGH : " << ROI_S_High << std::endl ;
				std::cout <<"                                 " << " V_LOW: " << ROI_V_Low <<  " V_HIGH : " << ROI_V_High << std::endl ;
				
				color++;
				
				break;
				
			}
		}
		
		
		while (color == 2) // Gate (Dark Blue Filter)  
		{  
			
			if(first_values == true)
			{
				
				low_H = Gate_H_Low ;
				low_S = Gate_S_Low ;
				low_V = Gate_V_Low ;
		
				high_H = Gate_H_High;
				high_S = Gate_S_High;
				high_V = Gate_V_High;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("Low H", filtered_image, &low_H, max_value_H, on_low_H_thresh_trackbar);
				createTrackbar("Low S", filtered_image, &low_S, max_value, on_low_S_thresh_trackbar);
				createTrackbar("Low V", filtered_image, &low_V, max_value, on_low_V_thresh_trackbar);
		
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				first_values = false ;
			}
		
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);
			
			
			cv::imshow(original_image, original_img);
			
			cv::imshow(filtered_image, hsv_filtered);
			
			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				Gate_H_Low  = low_H ;  Gate_H_High = high_H;
		
				Gate_S_Low  = low_S ;  Gate_S_High = high_S;
		
				Gate_V_Low  = low_V ;  Gate_V_High = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"\n Values of Filters for the Gate : " << " H_LOW: " << Gate_H_Low <<  " H_HIGH : " << Gate_H_High << std::endl ;
				std::cout <<"                                 " << " S_LOW: " << Gate_S_Low <<  " S_HIGH : " << Gate_S_High << std::endl ;
				std::cout <<"                                 " << " V_LOW: " << Gate_V_Low <<  " V_HIGH : " << Gate_V_High << std::endl ;
				
				color++;
				
				break;
				
			}
			//Print Values
		}
		
		
		while (color == 1) //Second Obstacle Filter (RED 2nd part) 
		{  
			
			if(first_values == true)
			{
				low_H = Obs_H_Low_2 ;
				low_S = Obs_S_Low_2 ;
				low_V = Obs_V_Low_2 ;
		
				high_H = Obs_H_High_2;
				high_S = Obs_S_High_2;
				high_V = Obs_V_High_2;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("Low H", filtered_image, &low_H, max_value_H, on_low_H_thresh_trackbar);
				createTrackbar("Low S", filtered_image, &low_S, max_value, on_low_S_thresh_trackbar);
				createTrackbar("Low V", filtered_image, &low_V, max_value, on_low_V_thresh_trackbar);
		
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				first_values = false ;
			}
		
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);
			
			
			cv::imshow(original_image, original_img);

			cv::imshow(filtered_image, hsv_filtered);
		
			
			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				Obs_H_Low_2  = low_H ;  Obs_H_High_2 = high_H;
		
				Obs_S_Low_2  = low_S ;  Obs_S_High_2 = high_S;
		
				Obs_V_Low_2  = low_V ;  Obs_V_High_2 = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"\n Values of Filters for the Second part of the Red Zone: " << " H_LOW: " << Obs_H_Low_2 <<  " H_HIGH : " << Obs_H_High_2 << std::endl ;
				std::cout <<"                                                   " << " S_LOW: " << Obs_S_Low_2 <<  " S_HIGH : " << Obs_S_High_2 << std::endl ;
				std::cout <<"                                                   " << " V_LOW: " << Obs_V_Low_2 <<  " V_HIGH : " << Obs_V_High_2 << std::endl ;
				
				color++;
				
				break;
				
			}
			
		}
		
		while (color == 0) //First Obstacle Filter (RED 1st part)
		{  
			//The Minimum Values for this Region are set to 0,0,0 , so we are just going to adjust the Max Values 
			
			low_H = 0 ;
			low_S = 0 ;
			low_V = 0 ;
		
			//Previously Stored Approximated Values
			
			if(first_values == true)
			{
				high_H = Obs_H_High_1;
				high_S = Obs_S_High_1;
				high_V = Obs_V_High_1;
				
				cv::namedWindow(original_image, CV_WINDOW_NORMAL);
				cv::namedWindow(filtered_image, CV_WINDOW_NORMAL);
				
				cv::moveWindow(filtered_image, 700,0);
				
				createTrackbar("High H", filtered_image, &high_H, max_value_H, on_high_H_thresh_trackbar);
				createTrackbar("High S", filtered_image, &high_S, max_value, on_high_S_thresh_trackbar);
				createTrackbar("High V", filtered_image, &high_V, max_value, on_high_V_thresh_trackbar);
				
				first_values=false;
			}	 
				
			inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), hsv_filtered);
		
			
			cv::imshow(original_image, original_img);
			
			cv::imshow(filtered_image, hsv_filtered);
			
			
			char key = (char) cv::waitKey(30);
			
			if (key == 'q' || key == 27)
			{
				Obs_H_Low_1  = low_H ;  Obs_H_High_1 = high_H;
		
				Obs_S_Low_1  = low_S ;  Obs_S_High_1 = high_S;
		
				Obs_V_Low_1  = low_V ;  Obs_V_High_1 = high_V;
				
				cv::destroyWindow(original_image);
				cv::destroyWindow(filtered_image);
				
				std::cout <<"Values of Filters for the First part of Red Zone: " << " H_LOW: " << Obs_H_Low_1 <<  " H_HIGH : " << Obs_H_High_1 << std::endl ;
				std::cout <<"                                                  " << " S_LOW: " << Obs_S_Low_1 <<  " S_HIGH : " << Obs_S_High_1 << std::endl ;
				std::cout <<"                                                  " << " V_LOW: " << Obs_V_Low_1 <<  " V_HIGH : " << Obs_V_High_1 << std::endl ;
				
				color++;
				
				break;
				
				
			}
			
		}
		
		
	}	
    return 0;
}

//Make if elses (color depending) and the print in terminal values and get out for the next one , 
//be sure to create a lot of variables to store the min and max values of each color in the HSV RANGE


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

  if(MC_developer_session == true) showImage("Generated Map", out);

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
	
  int red_mask_low1;
  int red_mask_high1; 
  int red_mask_low2; 
  int red_mask_high2;
  int greenmask_low;
  int greenmask_high;
  int blue_mask_low; 
  int blue_mask_high;
	
  
  
  
  // Convert color space from BGR to HSV
  cv::cvtColor(map, hsv_img, cv::COLOR_BGR2HSV);

	
  if(MC_developer_session == true) HSV_Calib(map , hsv_img , red_mask_low1 , red_mask_high1 , red_mask_low2 , red_mask_high2 , greenmask_low , greenmask_high , blue_mask_low , blue_mask_high);
	
  // Find red regions: h values around 0 (positive and negative angles)
  cv::Mat red_mask_low, red_mask_high, red_mask;
                              // 0  30 30             20
  cv::inRange(hsv_img, cv::Scalar(Obs_H_Low_1, Obs_S_Low_1, Obs_V_Low_1), cv::Scalar(Obs_H_High_1, Obs_S_High_1, Obs_V_High_1), red_mask_low);
                              // 150  30 30             180         
  
  cv::inRange(hsv_img, cv::Scalar(Obs_H_Low_2, Obs_S_Low_2, Obs_V_Low_2), cv::Scalar(Obs_H_High_2, Obs_S_High_2, Obs_V_High_2), red_mask_high);
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
    if(MC_developer_session == true) showImage("Obstacles", red_mask_temp);
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
  
  
  cv::inRange(hsv_img, cv::Scalar(Black_H_Low, Black_S_Low, Black_V_Low), cv::Scalar(Black_H_High, Black_S_High, Black_V_High), black_mask);
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

  if(MC_developer_session == true) showImage("Adaptive mask", black_mask);

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
    if(MC_developer_session == true) showImage("Border", contours_img);
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
  
  
  cv::inRange(hsv_img, cv::Scalar(ROI_H_Low, ROI_S_Low , ROI_V_Low), cv::Scalar(ROI_H_High, ROI_S_High, ROI_V_High), green_mask);
 
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
    if(MC_developer_session == true) showImage("Circles Detected", circles_img);
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
 
  
 
  
  cv::inRange(hsv_img, cv::Scalar(Gate_H_Low, Gate_S_Low, Gate_V_Low ) , cv::Scalar(Gate_H_High, Gate_S_High, Gate_V_High), blue_mask);

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
    if(MC_developer_session == true) showImage("Gate(s) Boundaries", blue_mask_temp);
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
  
  
  
  cv::inRange(hsv_img, cv::Scalar(Robot_H_Low, Robot_S_Low, Robot_V_Low), cv::Scalar(Robot_H_High, Robot_S_High, Robot_V_High), robot_mask);

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
    if(MC_developer_session == true) showImage("Robot Outline", robot_mask_temp);
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
  if(MC_developer_session == true) showImage("Original map", map);

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








