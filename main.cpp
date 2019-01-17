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
#include "collision_detection.hpp"


int main(int argc, char* argv[])
{
   
    bool stop_after_collision = false; //developer mode (false -> Don't stop after collisions

	
    //MENU
    // std::cout << "Please choose an option: " << std::endl;
    
    if (argc != 2) 
    {
        std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
        return 0;
    }

    std::string filename = argv[1];

    //calibrateCamera();

    std::string outputFilename = extractMap(filename);

    std::vector<cv::Vec3f> circles = processImage(outputFilename);

    //int *pointer = recognizeDigits(outputFilename, circles);
    
    int collision_trigger = 0 ;

    //orderROI(outputFilename, circles, pointer);
    
    cv::Mat map = readData("data/output/output.txt");

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
    
    //INITIAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(171,292), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(x0,y0) , cv::Point(x0+radius*cos(th0), y0+radius*sin(th0)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
    //FINAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(xf,yf), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(xf,yf) , cv::Point(xf+radius*cos(thf), yf+radius*sin(thf)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
    //-----------------------------------------------------------------------------------------------------------------------------
     
    int npts = 10000;
    
    int old_x=0; 
    int new_x=0;
    
    int old_y=0;
    int new_y=0;
    
    bool marker = false;
    
    if (collision_trigger == false || stop_after_collision == false)
    {
		//COLLISION DETECTION
		
		//FIRST ARC
    
		for(int j=0; j<npts;++j)
		{
			double s = curve.arc1.L/npts*j;
			circline(s,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
			
			new_x = (int)cline[0];
			new_y = (int)cline[1];
			
			if ((new_x != old_x || new_y != old_y ))
			{
				
				collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]});
				
					
				if (collision_trigger == false) 
				{
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
				} 
				
				else if(collision_trigger == true)
				{  
					cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 ); //Collision
					if (marker == false) marker=true; //To flag the first intersection
					if (stop_after_collision == true) break;
				}
				
				old_x = new_x ;
			    old_y = new_y ;
					
			}
			else if (collision_trigger == true && stop_after_collision == true ) break;
		}
		
		if (collision_trigger == false && marker == false) std::cout << "\n\n Inspection Finished , no collisions detected in the First Arc" << std::endl;
		else if(marker > 0) std::cout << "\n\n Inspection Finished , some collisions detected in the First Arc !!! \n\n" << std::endl;
		
		
		
		
		//LINE
		
		marker = false;
		
		for(int j=0; j<npts;++j)
		{
			double s = curve.arc2.L/npts*j;
			circline(s,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
			
			new_x = (int)cline[0];
			new_y = (int)cline[1];
			
			if ((new_x != old_x || new_y != old_y ))
			{
				collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]});
				
					
				if (collision_trigger == false) 
				{
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
				} 
				
				else if(collision_trigger == true)
				{  
					cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
					if (marker == false) marker=true; //To flag the first intersection
					if (stop_after_collision == true) break;
				}
				
				old_x = new_x ;
			    old_y = new_y ;
					
			}
			else if (collision_trigger == true && stop_after_collision == true ) break;
		}
		
		if (collision_trigger == false && marker == false) std::cout << "\n\n Inspection Finished , no collisions detected in the Line" << std::endl;
		else if(marker > 0) std::cout << "\n\n Inspection Finished , some collisions detected in the Line !!! \n\n" << std::endl;
		
		//SECOND ARC'
		
		marker = false;
		
		for(int j=0; j<npts;++j)
		{
			double s = curve.arc3.L/npts*j;
			circline(s,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
			
			new_x = (int)cline[0];
			new_y = (int)cline[1];
			
			if ((new_x != old_x || new_y != old_y ))
			{
				collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]});
				
					
				if (collision_trigger == false) 
				{
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
				} 
				
				else if(collision_trigger == true)
				{  
					cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );
					if (marker == false) marker=true; //To flag the first intersection
					if (stop_after_collision == true) break;
				}
				
				old_x = new_x ;
			    old_y = new_y ;
					
			}
			else if (collision_trigger == true && stop_after_collision == true ) break;
		}
		
	    if (collision_trigger == false && marker == false) std::cout << "\n\n Inspection Finished , no collisions detected in the Second Arc" << std::endl;
		else if(marker > 0) std::cout << "\n\n Inspection Finished , some collisions detected in the Second Arc !!! \n\n" << std::endl;
		
	}

	std::string name = "Corners";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 512, 640);    
    
    //cv::Mat BW_Map = print_contours(map);
    
    cv::Mat plan;
    cv::add(out,map,plan);
    
    cv::imshow(name.c_str(), plan ); 
    cv::waitKey(0);
    
    //Test for an individual point
    //cv::Point p{430,456}; 
    //collision_detection(map , p);
    
    
    return 0;
}
