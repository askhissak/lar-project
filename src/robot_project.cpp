// robot_project.cpp:
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera_calibration.hpp"
#include "map_extraction.hpp"
#include "shape_detection.hpp"
#include "digit_recognition.hpp"
#include "reader.hpp"
#include "dubins.hpp"
#include "collision_detection.hpp"

#include "path.h"
#include "robot_project.h"

std::vector<cv::Vec3f> circles;
std::vector<cv::Point> orderedROI;

cv::Point gate_center = {0,0};

// Constructor taking as argument the command line parameters
RobotProject::RobotProject(int argc, char * argv[])
{

}

// Method invoked to preprocess the map (extrinsic calibration + reconstruction of layout)
bool RobotProject::preprocessMap(cv::Mat const & img)
{
	cv::Mat robotPlane, map;

	// calibrateCamera();

	//Image -> Map (borders only)
    if(!extractMap(img, map, robotPlane))
	{
		std::cerr << "(Critical) Failed to extract the map!" << std::endl;
		return false;
	}

	//Shapes
    circles = processImage(map , gate_center );

	//Digits
    if(!recognizeDigits(map, circles, orderedROI))
	{
		std::cerr << "(Critical) Failed to recognize the digits!" << std::endl;
		return false;
	}

    return true;

}

// Method invoked when a new path must be planned (detect initial robot position from img)
bool RobotProject::planPath(cv::Mat const & img, Path & path)
{
    std::vector<cv::Point> pointPath;
	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	
	cv::Mat map = readData("data/output/output.txt"); //First map acquisition
	
	cv::Mat original_map = map; 

    double robotX0 = 150, robotY0 = 450, robotTH0 = -M_PI/4, gateXf = 1440, gateYf = 225, gateTHf = 0; 
    double x0, y0, th0, xf, yf, thf; 
    double Kmax = 0.01;
    double radius = 50;
	int npts = 1000;
	double s1,s2,s3;

	DubinsCurve curve;

	for(int i=0;i<orderedROI.size();++i)
	{
		if(i==0)
		{
			x0 = robotX0;
			y0 = robotY0;
			th0 = robotTH0;
			xf = orderedROI[i].x;
			yf = orderedROI[i].y;
			thf = atan2(orderedROI[i].y-robotY0,orderedROI[i].x-robotX0);
		}
		else
		{
			x0 = orderedROI[i-1].x;
			y0 = orderedROI[i-1].y;
			th0 = thf;
			xf = orderedROI[i].x;
			yf = orderedROI[i].y;
			thf = atan2(orderedROI[i].y-orderedROI[i-1].y,orderedROI[i].x-orderedROI[i-1].x);
		}

    	curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 


		for(int j=0; j<npts;++j)
		{
			s1 = curve.arc1.L/npts*j;
			circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
			pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
			cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
		}
		for(int j=0; j<npts;++j)
		{
			s2 = curve.arc2.L/npts*j;
			circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
			pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
			cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
		}
		for(int j=0; j<npts;++j)
		{	
			s3 = curve.arc3.L/npts*j;
			circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
			pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
			cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
		}
	} 

	x0 = orderedROI[orderedROI.size()-1].x;
	y0 = orderedROI[orderedROI.size()-1].y;
	th0 = thf;
	xf = gateXf;
	yf = gateYf;
	thf = gateTHf;

	curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 

	for(int j=0; j<npts;++j)
	{
		s1 = curve.arc1.L/npts*j;
		circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
		pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{
		s2 = curve.arc2.L/npts*j;
		circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
		pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{	
		s3 = curve.arc3.L/npts*j;
		circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
		pointPath.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
        
    //INITIAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(robotX0,robotY0), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(robotX0,robotY0) , cv::Point(robotX0+radius*cos(robotTH0), robotY0+radius*sin(robotTH0)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
    //FINAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(gateXf,gateYf), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(gateXf,gateYf) , cv::Point(gateXf+radius*cos(gateTHf), gateYf+radius*sin(gateTHf)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
	
	showImage("Dubins path", out+map);
    
    //-----------------------------------------------------------------------------------------------------------------------------
    
    //NEW FUNCTION (INTRODUCE A NEW FUNCTION THAT CALLS POINT_PATH AND EXECUTE VBALUE BY REFEREnCE) 
    
    // int npts = 10000;
    
    int old_x=0; 
    int new_x=0;
    
    int old_y=0;
    int new_y=0;
    
    int remapper = 0;
    
    int collision_trigger = 0 ;
    
    bool stop_after_collision = false; // Developer mode .... false -> Don't stop after collisions
    
    int thres_safe = 40 ; //Safe value for first approximation

	int thres_limit = 0;
	
	int thres = thres_safe ;
	
	int collision_status = 0;
    
    int contour_idx ;
    
    cv::Point ini_collision;
    
    cv::Point fin_collision ;
    
    cv::Point collision_point;
    
    cv::Point contour_center(0,0) ; 
		
    int contour_radius = 0 ; 
    
    double desired_contour_area = 0;   //NEW
    
    bool contour_printer = true ;
    
    
    cv::Mat original_dubins = cv::Mat::zeros(out.size() , out.type());
    
    cv::Mat new_desired_contour = cv::Mat::zeros(out.size() , out.type());
    
    std::string name = "NAME OF THE WINDOW" ;
    
    cv::Mat expanded = cv::Mat::zeros(map.size() , map.type());
    
    cv::Mat isolated_contour = cv::Mat::zeros(out.size() , out.type());
 
    
    
    while(remapper < 4 )
    {
		
		collision_trigger = false;
		
		if 		(remapper == 0) thres = thres_safe;  
		
		else if (remapper == 1) {thres = thres_limit; remapper++;}
    
		else if (remapper == 2) remapper++;
		
		//COLLISION DETECTION
			
		for(int j=0 ; j<pointPath.size() ; ++j)
		{
			new_x = pointPath[j].x ;
			new_y = pointPath[j].y ;
			
			if ((new_x != old_x || new_y != old_y ))
			{
				collision_trigger = collision_detection(map, pointPath[j] , thres , remapper , collision_status);
					
				if (collision_trigger == false) cv::circle(out, cv::Point(pointPath[j]), 0.5 , cv::Scalar( 0 , 200 , 0 ), 5, 8, 0 ); //No collision
				
				else if(collision_trigger == true)
				{  
						 if (remapper == 0) remapper++;
					else if (remapper == 3 && collision_status == 1 ) { ini_collision = cv::Point(pointPath[j]); } 
					else if (remapper == 3 && collision_status == 2 ) { fin_collision = cv::Point(pointPath[j]); }
					
						 if (stop_after_collision == true) break;
				}
				
				old_x = new_x ;
				old_y = new_y ;
					
			}
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
		
		cv::Mat isolated_contour = out ;
		
		switch (remapper)
		{
			
			case 1 :  //To find the collision with the obstacle
				{
					expanded = expander(map);
				
					std::cout << "\n\n REMAPPING ___ FIRST ITERATION \n\n" << std::endl;
					map = expanded;
				
					break;
			
				}
			case 2 : //Remapping for the found contour
					
				{	
				
					isolated_contour = print_desired_contour(expanded , contour_idx , collision_point , desired_contour_area );
				
					if (contour_printer == true) { new_desired_contour = isolated_contour ; contour_printer = false; }
					
					break;
					
				}	
			case 3 : //To Draw the final trajectory
				{
					std::cout << "\n\n First Point : " << ini_collision << " ,  Last Point :  "  << fin_collision << "\n\n" << std::endl;
					
					//DELETING INITIAL POSITION AND CONFIGURATION
					cv::circle(out , cv::Point(171,292), 10 , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0 );
					cv::line(out , cv::Point(x0,y0) , cv::Point(x0+radius*cos(th0), y0+radius*sin(th0)) , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0);
	
					//DELETING FINAL POSITION AND CONFIGURATION
					cv::circle(out, cv::Point(xf,yf), 10 , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0 );
					cv::line(out, cv::Point(xf,yf) , cv::Point(xf+radius*cos(thf), yf+radius*sin(thf)) , cv::Scalar( 0 , 0 , 0 ), 5, 8, 0);
					
					original_dubins = out.clone();
				
					cv::Mat smoothed_contour_map = create_new_path ( original_dubins , new_desired_contour  , ini_collision ,  fin_collision  );
					
					
					//Analyze again the final smoothed trajectory for crashes
					
					//Save the vector in order to have it together with the others
					
					////// TO MIGRATE (END HERE) //////////////////////////////////
					
					
					name = " FInal Traj + Obstacles " ;
					cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
					cv::resizeWindow(name.c_str(), 512, 640);   		
					cv::imshow(name.c_str(), smoothed_contour_map); 
					cv::waitKey(0);
					cv::destroyWindow(name.c_str());
	
					
					remapper++;
					break;
				}
				
				
		}
	}

    return true;

}

// Method invoked periodically to determine the position of the robot within the map.
// The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const & img, 
            std::vector<double> & state)
{
    return true;
}
