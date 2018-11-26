#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream> 

#include <opencv2/opencv.hpp>

#include "reader.hpp"


using namespace std;
using namespace cv;

std::vector<std::vector<cv::Point>> contours, contours_approx;

void readData(const std::string filename)
{
	ifstream file;
        file.open(filename);
	
        int type;
	int id;
        int corners;


  cv::Mat out(1470, 970, CV_8UC3, Scalar(0,0,0));
  std::string outName = "Output Data";
  cv::namedWindow(outName.c_str(), CV_WINDOW_NORMAL);
  cv::resizeWindow(outName.c_str(), 512, 640);
//   cv::namedWindow("Output Data", CV_WINDOW_AUTOSIZE);
  cv::imshow(outName.c_str(), out);
  cv::waitKey(0);
  cv::destroyWindow(outName.c_str());

  int stepSize = 35;

  int width = out.size().width;
  int height = out.size().height;

   for (int i = 0; i<height; i += stepSize)
    cv::line(out, Point(0, i), Point(width, i), cv::Scalar( 100 , 100 , 100 ));

   for (int i = 0; i<width; i += stepSize)
    cv::line(out , Point(i, 0), Point(i, height), cv::Scalar( 100 , 100 , 100 ));

   std::string name = "Corners";


        while (!file.eof())
{

        file >> type >> id >> corners ;

	int x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8;

	
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

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);
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

                        
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);
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

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
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
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
		
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
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
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
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
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

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);   		
		break;

    		
               default: std::cout << " Error in the Obstacles " << endl; break; 
       }//switch

  
     }//if 	
     
     else if (type == 1) 
     {
	switch (corners)  
         {
    		case 3:  
			std::cout << " Triangle Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << endl; 
			
                        cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 50 , 50 , 250 ), 5, 8, 0 );

                        cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 0 , 0 ,150), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 0 , 0 ,150), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x0,y0) , cv::Scalar( 0 , 0 ,150), 5);

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);
 		break; 

		case 4:  
			std::cout << " Square Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << endl; 

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);

                        
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);
 		break; 

		case 5:  
			std::cout << " Pentagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x4,y4) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
 		break; 

		case 6:  
			std::cout << " Hexagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << endl;
		
			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
		
		break; 

		case 7:  
			std::cout << " Heptagon Gate " << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
		break;

		case 8:  
			std::cout << " Octagon Gate" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << endl; 

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x7,y7) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);
		
  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0); 
		break;

		case 9:  
			std::cout << " Nonagon Gate" << endl; 
			file >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3 >> x4 >> y4 >> x5 >> y5 >> x6 >> y6 >> x7 >> y7 >> x8 >> y8 ;	
			std::cout << x0 << y0 << x1 << y1 << x2 << y2 << x3 << y3 << x4 << y4 << x5 << y5 << x6 << y6 << x7 << y7 << x8 <<  y8 << endl;

			cv::circle( out , cv::Point(x0,y0), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x1,y1), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x2,y2), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x3,y3), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x4,y4), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x5,y5), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x6,y6), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x7,y7), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
			cv::circle( out , cv::Point(x8,y8), 0.5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
                        
			cv::line(out, cv::Point(x0,y0) , cv::Point(x1,y1) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x1,y1) , cv::Point(x2,y2) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x2,y2) , cv::Point(x3,y3) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x3,y3) , cv::Point(x4,y4) , cv::Scalar( 200 , 0 , 0 ), 5);
                        cv::line(out, cv::Point(x4,y4) , cv::Point(x5,y5) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x5,y5) , cv::Point(x6,y6) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x6,y6) , cv::Point(x7,y7) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x7,y7) , cv::Point(x8,y8) , cv::Scalar( 200 , 0 , 0 ), 5);
			cv::line(out, cv::Point(x8,y8) , cv::Point(x0,y0) , cv::Scalar( 200 , 0 , 0 ), 5);				

  			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  			cv::resizeWindow(name.c_str(), 512, 640);
			cv::imshow(name.c_str(), out); 
                        cv::waitKey(0);  		
		break;

    		
               default: std::cout << " Error in the Gates " << endl; break; 
       }//switch

     }//if
       
     else if (type == 2)
     {
      	file >> x0 >> y0 ;	
	std::cout << x0 << y0 << endl; 
	cv::circle( out , cv::Point(x0,y0), 8 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 );
	// cv::imshow("Corners", out); 
  	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
  	cv::resizeWindow(name.c_str(), 512, 640);
	cv::imshow(name.c_str(), out); 
        cv::waitKey(0);   		
     }


     else {std::cout << " Error in the Switch " << endl;}


}

}

