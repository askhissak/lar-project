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
    
    

    //orderROI(outputFilename, circles, pointer);
    
    cv::Mat map = readData("data/output/output.txt"); //First map acquisition
	
	cv::Mat original_map = map ; 
	
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
    
    int marker = 000;
    
    int remapper = 0;
    
    int collision_trigger = 0 ;
    
    bool stop_after_collision = false; // Developer mode .... false -> Don't stop after collisions
    
    int thres_safe = 40 ; //Safe value for first approximation

	int thres_limit = 0;
	
	int thres = thres_safe ;
	
	int contour_index = 0;
    
    int new_contour_index = 0;
    
    cv::Point ini_collision;
    
    cv::Point fin_collision ;
    
    cv::Point collision_point;
    
    cv::Point contour_center(0,0) ; 
		
    int contour_radius = 0 ; 
    
    
    while(remapper < 4 )
    {
		marker = 000;
		
		collision_trigger = false;
		
		if 		(remapper == 0) thres = thres_safe;  
		
		else if (remapper == 1) {thres = thres_limit; remapper++;}
    
		else if (remapper == 2) remapper++;
		
		//COLLISION DETECTION
		
		if (stop_after_collision == false)
		{
			
			//FIRST ARC
		
			for(int j=0; j<npts;++j)
			{
				double s = curve.arc1.L/npts*j;
				circline(s,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
				
				new_x = (int)cline[0];
				new_y = (int)cline[1];
				
				if ((new_x != old_x || new_y != old_y ))
				{
					
					collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]} , thres , remapper , contour_index);
					
					if (thres==thres_limit && contour_index > new_contour_index) new_contour_index = contour_index;
						
					if (collision_trigger == false) 
					{
						cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
					} 
					
					else if(collision_trigger == true)
					{  
						if (remapper == 0) remapper++;
						
						else if (remapper != 3) {cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 ); collision_point = cv::Point((int)cline[0],(int)cline[1]);}//Collision
						else if (remapper == 3 && new_contour_index == 100 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );*/ ini_collision = cv::Point((int)cline[0],(int)cline[1]); } //Red Points
						else if (remapper == 3 && new_contour_index == 200 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 );*/ fin_collision = cv::Point((int)cline[0],(int)cline[1]); } //Blue Points
						else if (remapper == 3 && new_contour_index != 100  && new_contour_index != 200 ) cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0 ); //Delete Points
						
						
						marker=100; //To flag the first intersection
						if (stop_after_collision == true) break;
					}
					
					old_x = new_x ;
					old_y = new_y ;
						
				}
				else if (collision_trigger == true && stop_after_collision == true ) break;
			}
			
			if (collision_trigger == false && marker == 000) std::cout << "\n\n Inspection Finished , no collisions detected in the First Arc" << std::endl;
			else if(marker == 100) std::cout << "\n\n Inspection Finished , some collisions detected in the First Arc !!! \n\n" << std::endl;
			
			//LINE
			
			for(int j=0; j<npts;++j)
			{
				double s = curve.arc2.L/npts*j;
				circline(s,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
				
				new_x = (int)cline[0];
				new_y = (int)cline[1];
				
				if ((new_x != old_x || new_y != old_y ))
				{
					collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]} , thres , remapper , contour_index);
					
					//if (thres==thres_limit && contour_index > new_contour_index) new_contour_index = contour_index;
					
						
					if (collision_trigger == false) 
					{
						cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
					} 
					
					else if(collision_trigger == true)
					{  
						if (remapper == 0) remapper++;
						
						else if (remapper != 3) {cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 ); collision_point = cv::Point((int)cline[0],(int)cline[1]);} //collision
						else if (remapper == 3 && contour_index == 100 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 10 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );*/ ini_collision = cv::Point((int)cline[0],(int)cline[1]); } //First Point
						else if (remapper == 3 && contour_index == 200 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 10 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );*/ fin_collision = cv::Point((int)cline[0],(int)cline[1]); } //Last Point
						else if (remapper == 3 && contour_index != 100 && contour_index != 200 ) cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0 ); //Delete Points
						
						if (marker == 100) marker = 110;
						if (marker != 100) marker = 010; //To flag the first intersection
						
						if (stop_after_collision == true) break;
					}
					
					old_x = new_x ;
					old_y = new_y ;
						
				}
				else if (collision_trigger == true && stop_after_collision == true ) break;
			}
			
			if (collision_trigger == false && marker == false) std::cout << "\n\n Inspection Finished , no collisions detected in the Line" << std::endl;
			else if(marker==010 || marker == 110 ) std::cout << "\n\n Inspection Finished , some collisions detected in the Line !!! \n\n" << std::endl;
			
			//SECOND ARC'
			
			for(int j=0; j<npts;++j)
			{
				double s = curve.arc3.L/npts*j;
				circline(s,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
				
				new_x = (int)cline[0];
				new_y = (int)cline[1];
				
				if ((new_x != old_x || new_y != old_y ))
				{
					collision_trigger = collision_detection(map, {(int)cline[0],(int)cline[1]} , thres , remapper , contour_index);
					
					if (thres==thres_limit && contour_index > new_contour_index) new_contour_index = contour_index;
						
					if (collision_trigger == false) 
					{
						cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
					} 
					
					else if(collision_trigger == true)
					{  
						if (remapper == 0) remapper++;
						
						else if (remapper != 3) {cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 ); collision_point = cv::Point((int)cline[0],(int)cline[1]);} //Collision}
						else if (remapper == 3 && new_contour_index == 100 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 0 , 0 , 200 ), 5, 8, 0 );*/ ini_collision = cv::Point((int)cline[0],(int)cline[1]); } //Red Points
						else if (remapper == 3 && new_contour_index == 200 ) { /*cv::circle(out, cv::Point((int)cline[0],(int)cline[1]), 5 , cv::Scalar( 200 , 0 , 0 ), 5, 8, 0 ); */ fin_collision = cv::Point((int)cline[0],(int)cline[1]); } //Blue Points
						else if (remapper == 3 && new_contour_index != 100  && new_contour_index != 200 ) cv::circle(out, cv::Point((int)cline[0],(int)cline[1]),5 , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0 ); //Delete Points
						
						if (marker == 100) marker = 101;
						if (marker == 110) marker = 111;
						if (marker == 000) marker = 001;
						if (stop_after_collision == true) break;
					}
					
					old_x = new_x ;
					old_y = new_y ;
						
				}
				else if (collision_trigger == true && stop_after_collision == true ) break;
			}
			
			if ( collision_trigger == false && (marker == 110 || marker == 010 || marker == 000) ) {std::cout << "\n\n Inspection Finished , no collisions detected in the Second Arc" << std::endl;}
			else if((marker == 001 || marker == 011 || marker == 111 ) && stop_after_collision == false) std::cout << "\n\n Inspection Finished , some collisions detected in the Second Arc !!! \n\n" << std::endl;
			
		}
		
		switch(marker)
		
		{	
			case 000 : std::cout << "\n\n Inspection Finished , NO COLLLISIONS DETECTED !!! \n\n" << std::endl; break;
			
			case 001 : std::cout << "\n\n Inspection Finished , COLLISION JUST IN THE FINAL ARC !!! \n\n" << std::endl; break;
			
			case 010 : std::cout << "\n\n Inspection Finished , COLLISION JUST IN THE LINE !!! \n\n" << std::endl; break;
			
			case 011 : std::cout << "\n\n Inspection Finished , COLLISION IN THE LINE AND FINAL ARC !!! \n\n" << std::endl; break;
			
			case 100 : std::cout << "\n\n Inspection Finished , COLLISION JUST IN THE FIRST ARC !!! \n\n" << std::endl; break;
			
			case 101 : std::cout << "\n\n Inspection Finished , COLLISIONS IN THE FIRST AND LAST ARCS !!! \n\n" << std::endl; break;
			
			case 110 : std::cout << "\n\n Inspection Finished , COLLISIONS IN THE FIRST ARC AND THE LINE !!! \n\n" << std::endl; break;
			
			case 111 : std::cout << "\n\n Inspection Finished , COLLISION IN ALL SEGMENTS !!! \n\n" << std::endl; break;
		}
		
		//cv::Mat plan; //TRAJECTORY + OBSTACLES
		
		//cv::add(out,map,plan);
	
		std::string name = "TRAJECTORY + OBSTACLES" ;
		cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(name.c_str(), 512, 640);    
		cv::imshow(name.c_str(), out+map ); 
		cv::waitKey(0);
		cv::destroyWindow(name.c_str());
		
		
		
		cv::Mat rounded = map;
		
		//Aqui poner la nueva func para el new path
		
		//remapper == 1 Print rounded limits and run the collision detector with thres = 0  
		//remapper == 2 Run the new trajectory creator after the collision detection to build new path
		//remapper == 3 Build the new trajectory
		
		cv::Mat isolated_contour = out ;
		
		switch (remapper)
		{
			
			case 1 :  //To find the collision with the obstacle
				{
					rounded = rounder(map);
					
					std::cout << "\n\n REMAPPING ___ FIRST ITERATION \n\n" << std::endl;
			        map = rounded;
				
					name = " ROUNDED LIMITS " ;
					cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
					cv::resizeWindow(name.c_str(), 512, 640);   		
					cv::imshow(name.c_str(), rounded ); 
					cv::waitKey(0);
					cv::destroyWindow(name.c_str());
					break;
			
				}
			case 2 : //Remapping for the found contour
					
				{	
					isolated_contour = print_desired_contour(rounded , new_contour_index , contour_center , contour_radius  , collision_point );
					
					name = " ISOLATED CONTOUR " ;
					cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
					cv::resizeWindow(name.c_str(), 512, 640);   		
					cv::imshow(name.c_str(), isolated_contour );
					cv::waitKey(0);
					cv::destroyWindow(name.c_str());
					
					break;
					
				}	
			case 3 : //To Draw the final trajectory
				{
					std::cout << "\n\n First Point : " << ini_collision << " ,  Last Point :  "  << fin_collision << "  Center : " << contour_center << " ,  Radius :  "  << contour_radius <<  "\n\n"  << std::endl;
					
					cv::Mat cropped_contour = crop_contour( isolated_contour ,  ini_collision , fin_collision , contour_center , contour_radius );
					
					name = " Final Intended Trajectory" ;
					cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
					cv::resizeWindow(name.c_str(), 512, 640);   		
					cv::imshow(name.c_str(), cropped_contour + original_map); 
					cv::waitKey(0);
					cv::destroyWindow(name.c_str());
					remapper++;
					break;
				}	
		}
	}
	
	return 0;

}

//Test for an individual point
		//cv::Point p{430,456}; 
		//collision_detection(map , p);
	
