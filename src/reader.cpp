#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream> 
#include <opencv2/opencv.hpp>
#include "reader.hpp"


using namespace std;
using namespace cv;


bool RD_developer_session = true ; // if true  -> Retrieves desired debugging and log content 
								   // if false -> Process everything without graphical output 

std::vector<std::vector<cv::Point>> contours, contours_approx;


cv::Mat readData(const std::string filename)
{
	
    ifstream file;
    file.open(filename);
	
    int type;
    int id;
    int corners;


  cv::Mat out(1050, 1510, CV_8UC3, Scalar(0,0,0));
  cv::Mat complete_map (1050, 1510, CV_8UC3, Scalar(0,0,0));
  
  if (RD_developer_session == true)
  {
	  std::string outName = "Output Data";
	  cv::namedWindow(outName.c_str(), CV_WINDOW_NORMAL);
	  cv::resizeWindow(outName.c_str(), 640, 512);
	  cv::imshow(outName.c_str(), complete_map);
	  //cv::waitKey(0);
	  cv::destroyWindow(outName.c_str());
  }
	
  	
	
  int stepSize = 35;

  int width =  complete_map.size().width;
  int height = complete_map.size().height;

   for (int i = 0; i<height; i += stepSize)
   cv::line(complete_map, Point(0, i), Point(width, i), cv::Scalar( 100 , 100 , 100 ));

   for (int i = 0; i<width; i += stepSize)
   cv::line( complete_map , Point(i, 0), Point(i, height), cv::Scalar( 100 , 100 , 100 ));

   std::string name = "Corners";


while (!file.eof())
{

        file >> type >> id >> corners ;

	int x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8,x9,y9,x10,y10,x11,y11,x12,y12,x13,y13,x14,y14,x15,y15,x16,y16,x17,y17,x18,y18,x19,y19,x20,y20 ;

	
	if(type == 0)

	{

        switch (corners)  
         {
    		case 3:  
			std::cout << " Triangle Obstacle " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << endl; 
			
            cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );

            cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);

			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());
			}
 		break; 

		case 4:  
			std::cout << " Square Obstacle " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << endl; 

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);

            if (RD_developer_session == true)
			{            
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());
			}
 		break; 

		case 5:  
			std::cout << " Pentagon Obstacle " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);

  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
 		break; 

		case 6:  
			std::cout << " Hexagon Obstacle " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << endl;
		
			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
		
		break; 

		case 7:  
			std::cout << " Heptagon Obstacle " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
		break;

		case 8:  
			std::cout << " Octagon Obstacle" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << endl; 

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
		break;

		case 9:  
			std::cout << " Nonagon Obstacle" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 <<  y8 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7) , cv::Point(x8,y8) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);				

  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str()); 
			}  		
		break;
		
		
		case 10:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
  			
  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());  		
			}
		break;
		
		case 11:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10   ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
  			
  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());   		
			}
		break;
		
		case 12:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11   ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str()); 
			}  		
		
		break;
		
		
		case 13:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			
			           
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());   
			}		
		
		break;
		
		case 14:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			           
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
				
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);  
				cv::destroyWindow(name.c_str());
			} 		
		
		break;
		
		
		case 15:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15 >> x16 >> y16 >> x17 >> y17 >> x18 >> y18 >> x19 >> y19     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << x16 << y16 << x17 << y17 << x18 << y18 << x19 << y19 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			             
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				//cv::waitKey(0);
				cv::destroyWindow(name.c_str());   		
			}
		break;
		
		case 16:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15   ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x15,y15), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			            
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x15,y15) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x15,y15) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				//cv::waitKey(0); 
				cv::destroyWindow(name.c_str()); 
			} 		
		
		break;
		
		case 17:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15 >> x16 >> y16      ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << x16 << y16  << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x15,y15), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x16,y16), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			            
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x15,y15) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x15,y15) , cv::Point(x16,y16) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x16,y16) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);				

  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);   
				cv::destroyWindow(name.c_str());	
			}		
		
		break;
		
		case 18:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15 >> x16 >> y16 >> x17 >> y17     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << x16 << y16 << x17 << y17 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x15,y15), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x16,y16), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x17,y17), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			            
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x15,y15) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x15,y15) , cv::Point(x16,y16) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x16,y16) , cv::Point(x17,y17) , cv::Scalar( 0 , 0 ,150), 5);				
			cv::line(out, cv::Point(x17,y17) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());   		
			}		
		break;
		
		
		case 19:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15 >> x16 >> y16 >> x17 >> y17 >> x18 >> y18    ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << x16 << y16 << x17 << y17 << x18 << y18 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x15,y15), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x16,y16), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x17,y17), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x18,y18), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			            
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x15,y15) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x15,y15) , cv::Point(x16,y16) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x16,y16) , cv::Point(x17,y17) , cv::Scalar( 0 , 0 ,150), 5);				
			cv::line(out, cv::Point(x17,y17) , cv::Point(x18,y18) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x18,y18) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());  		
			}
		break;
		
		
		case 20:  
			std::cout << corners  << " Corners Obstacle"  << endl; 
			file      >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> x9 >> y9 >> x10 >> y10 >> x11 >> y11 >> x12 >> y12 >> x13 >> y13 >> x14 >> y14 >> x15 >> y15 >> x16 >> y16 >> x17 >> y17 >> x18 >> y18 >> x19 >> y19     ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 << y8 << x9 << y9 << x10 << y10 << x11 << y11 << x12 << y12 << x13 << y13 << x14 << y14 << x15 << y15 << x16 << y16 << x17 << y17 << x18 << y18 << x19 << y19 << endl;

			cv::circle( out , cv::Point(x0,y0),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x9,y9),   0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x10,y10), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x11,y11), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x12,y12), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x13,y13), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x14,y14), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x15,y15), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x16,y16), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x17,y17), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x18,y18), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x19,y19), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0)   , cv::Point(x1,y1)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x1,y1)   , cv::Point(x2,y2)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2)   , cv::Point(x3,y3)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x3,y3)   , cv::Point(x4,y4)   , cv::Scalar( 0 , 0 ,150), 5);
            cv::line(out, cv::Point(x4,y4)   , cv::Point(x5,y5)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x5,y5)   , cv::Point(x6,y6)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x6,y6)   , cv::Point(x7,y7)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x7,y7)   , cv::Point(x8,y8)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x8,y8)   , cv::Point(x9,y9)   , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x9,y9)   , cv::Point(x10,y10) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x10,y10) , cv::Point(x11,y11) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x11,y11) , cv::Point(x12,y12) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x12,y12) , cv::Point(x13,y13) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x13,y13) , cv::Point(x14,y14) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x14,y14) , cv::Point(x15,y15) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x15,y15) , cv::Point(x16,y16) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x16,y16) , cv::Point(x17,y17) , cv::Scalar( 0 , 0 ,150), 5);				
			cv::line(out, cv::Point(x17,y17) , cv::Point(x18,y18) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x18,y18) , cv::Point(x19,y19) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x19,y19) , cv::Point(x0,y0)   , cv::Scalar( 0 , 0 ,150), 5);
  			
  			
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());  		
			}
		break;
		
    		
        default: std::cout << " Error in the Obstacles " << endl; break; 
       }//switch

  
     }//if 	
     
    else if (type == 1) //Gates
     {
	  switch (corners)  
       {
    	case 3:  
			std::cout << " Triangle Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << endl; 
			
            cv::circle(  complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );

            cv::line( complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
            cv::line( complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line( complete_map, cv::Point(x2,y2) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);

  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out +  complete_map); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());
			}
 		break; 

		case 4:  
			std::cout << " Square Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << endl; 

			cv::circle(  complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(complete_map  , cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map  , cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x3,y3) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);

            if (RD_developer_session == true)
			{            
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out + complete_map); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());
			}
		break; 

		case 5:  
			std::cout << " Pentagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << endl;

			cv::circle(  complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line( complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x4,y4) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);

			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out +  complete_map); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
 		break; 

		case 6:  
			std::cout << " Hexagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << endl;
		
			cv::circle(  complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line( complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x5,y5) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out + complete_map); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str()); 
			}
		
		break; 

		case 7:  
			std::cout << " Heptagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << endl;

			cv::circle( complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(complete_map  , cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map  , cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map  , cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map  , cv::Point(x6,y6) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out + complete_map); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
		break;

		case 8:  
			std::cout << " Octagon Gate" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << endl; 

			cv::circle( complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x7,y7), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x7,y7) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out + complete_map); 
				cv::waitKey(0); 
				cv::destroyWindow(name.c_str());
			}
		break;

			cv::circle( complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x7,y7), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( complete_map , cv::Point(x8,y8), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line(complete_map, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x7,y7) , cv::Point(x8,y8) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(complete_map, cv::Point(x8,y8) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);				

  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out+complete_map); 
				cv::waitKey(0);
				cv::destroyWindow(name.c_str());  		
			}
		break;
		
		case 10:  
			std::cout << corners << " Corners Gate" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 >> y9 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 <<  y8 << y9 << endl;

			cv::circle(  complete_map , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x7,y7), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle(  complete_map , cv::Point(x8,y8), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
            cv::circle(  complete_map , cv::Point(x9,y9), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line( complete_map, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
            cv::line( complete_map, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x7,y7) , cv::Point(x8,y8) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x8,y8) , cv::Point(x9,y9) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line( complete_map, cv::Point(x8,y8) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);				

  			if (RD_developer_session == true)
			{
				cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
				cv::resizeWindow(name.c_str(), 640, 512);
				cv::imshow(name.c_str(), out + complete_map ); 
				cv::waitKey(0);  
				cv::destroyWindow(name.c_str());
			}		
		break;

    		
        default: std::cout << " Error in the Gates " << endl; break; 
       
       }//switch

     }//if
       
     
     else if (type == 2)
     {
      	file >> x0 >> y0 ;	
		std::cout << " ROI" << endl; 
		std::cout << x0 << y0 << endl; 
		cv::circle( complete_map , cv::Point(x0,y0), 8 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 );
		
		
		if (RD_developer_session == true)
		{
			cv::imshow("Corners", out); 
			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
			cv::resizeWindow(name.c_str(), 640, 512);
			cv::imshow(name.c_str(), out + complete_map); 
			cv::waitKey(0);  
			cv::destroyWindow(name.c_str()); 
		}		
     }
     


     else {std::cout << " Error in the Switch " << endl;}


  }
  
  if (RD_developer_session == true)
  { 
	  name = "Just Obstacles" ;
	  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	  cv::resizeWindow(name.c_str(), 640, 512);
	  cv::imshow(name.c_str(), out); 
	  cv::waitKey(0);
	  cv::destroyWindow(name.c_str());
	  
	  name = "Complete Map " ;
	  cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	  cv::resizeWindow(name.c_str(), 640, 512);
	  cv::imshow(name.c_str(), out + complete_map ); 
	  cv::waitKey(0);   		   		
	  cv::destroyWindow(name.c_str());
  }
  
  return out;

}
