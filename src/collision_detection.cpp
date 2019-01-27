#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "reader.hpp"
#include <math.h> 


using namespace std;
using namespace cv;

int polyflag = 0;


int output_trigger =0;



//NEW ADDED THE DESIRED_CONTOUR AREA
cv::Mat print_desired_contour(cv::Mat inputmap, int &contour_idx , cv::Point colliding_point  , double &desired_contour_area)
{
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY);
	
	cv::Mat isolated_map = Mat::zeros( BW_Map.size(), BW_Map.type() );  ;
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	int old_dist = 1000;
	
	for( int i = 0 ; i < contours.size() ; i++ ) // iterate through each contour. 
    {	
		int new_dist = (int)((cv::pointPolygonTest(contours[i] , colliding_point , true))*(-1));
		
		if (new_dist < old_dist ) { old_dist = new_dist ; contour_idx = i  ; } 
		
		std::cout << "   Area of Inflated Contour Nr." << i << " : " <<  cv::contourArea(contours[i]) << " \n  " << std::endl;
	}
	
	
	desired_contour_area = cv::contourArea(contours[contour_idx]);
	
	std::cout << "\n\n Detected " << contours.size() << " Contours ! \n  " << std::endl;
	
	
	//Writing the Intended Contour
	
	
	int idx = 0;
    
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    { 
        if (idx == contour_idx) drawContours( isolated_map, contours, idx , cv::Scalar(255,255 ,255) , 1 , 8, hierarchy);
           
    } 
   
	return isolated_map;
}


bool collision_detection( cv::Mat inputmap , cv::Point target , int thres , int remapper , int &collision_status )
{
	
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY); 
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	float dist;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	
	if (polyflag==0) {std::cout << "\n\n Detected " << contours.size() << " Contours ! \n  " << std::endl; polyflag++;}
	
	//std::cout << "\n For the Point : " << target << std::endl;
	int i = 0;
	
	for( i = 0 ; i < contours.size() ; i++ ) // iterate through each contour. 
    {
		
		dist = (int)((cv::pointPolygonTest(contours[i] , target , true))*(-1));
		
		if (dist <= thres) break;
		
		//else std::cout << " Distance from Contour " << i <<  " : "  << dist << std::endl;
		
	}
	
	if (dist <= thres) 
	{
		std::cout << " Distance from Contour " << i <<  " : "  << dist <<  "   <---- THRESHOLD REACHED !!!!!! \n\n   "  << std::endl;
	         if (dist == 0 && remapper == 3 && output_trigger == 0 ) {collision_status = 1; output_trigger = 1;  } //Initial Value
		else if (dist <  0 && remapper == 3 && output_trigger == 1 ) {collision_status = 1;                      } 
		else if (dist == 0 && remapper == 3 && output_trigger == 1)  {collision_status = 2; output_trigger = 2;  } //Final Value
		else if (dist >  0 && remapper == 3 && output_trigger == 2)  {collision_status = 0;                      } 
		return true;
	}
	
	else return false;

}


cv::Mat expander(cv::Mat input)
{
	cv::Mat BW_Map ;
	
	cv::cvtColor(input, BW_Map, CV_BGR2GRAY);
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	cv::Mat expanded = Mat::zeros( BW_Map.size(), CV_8UC3 );
	
	
	float pixels = 30; // Number of pixels expanded in respect to the original contour
	
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, Scalar(255,255,255), pixels * 2);
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, Scalar(255,255,255), CV_FILLED );
	

	return expanded;
	
}

//NEW Implementation in this function 

int distance( cv::Point P1 , cv::Point P2)

{
	int dist = sqrt(((P1.x- P2.x) * (P1.x-P2.x)) + ((P1.y - P2.y ) * (P1.y - P2.y)));
	
	return dist;
	
}

cv::Mat create_new_path( cv::Mat original_dubins , cv::Mat new_desired_contour  , cv::Point ini_collision , cv::Point fin_collision  )
{	
	vector<vector<Point>> contours;
	
	vector<Point> first_region ;
	
	vector<Point> second_region ;
	
	
	findContours(new_desired_contour , contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	
	new_desired_contour = cv::Mat::zeros(new_desired_contour.size() , new_desired_contour.type());
    
    
    drawContours(new_desired_contour, contours, -1, cv::Scalar(255,255,255), 5 , cv::LINE_AA);  //Draw the found contours 
    
    string name = " Printing Simple Contour " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    cv::imshow(name.c_str(), new_desired_contour ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	 name = "DUBINS  " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    cv::imshow(name.c_str(), original_dubins ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	
	
	cv::Mat test = cv::Mat::zeros(original_dubins.size() , original_dubins.type());
	
	cv::Point test_point = {0,0}; 
	
	
	
	int total_lenght = 0 ;
	
	int first_partial_lenght =0 ;
	
	int second_partial_lenght = 0;

	int counter = 0 ;
	
	
	int region = 0 ;
	
	//Region 0 : Any extreme point has been reached
	//Region 1 : One point has been reached
	//Region 2 : We touched the secon point so the path 2 and 0 must be added to form the second region
	
 	//we include a counter to discard false positives.
	
	//LATER FROM HERE WE CAN COMPUTE THE TWO APPROACHES AND GET ACTUALLY BOTH CHANCES < MEASERRE THE DISTANCES AND CHECK FOR COLLISIONS AND WE CAN CHOOSE THE BEST APPROACH
	
	//FOR JUST ONE APPROACH :
	
	for(int i= 0; i < contours.size(); i++)
	{
		
		for(int j= 0; j < contours[i].size();j++) 
		{	
			total_lenght++;
			
			test_point = {contours[i][j].x , contours[i][j].y};
			
			std::cout << " Distance From INI -> TEST : " << distance( ini_collision , test_point ) <<  " ,   Distance FINAL -> TEST : "  << distance (fin_collision , test_point ) << std::endl;

			if ( distance(test_point , ini_collision ) < 2 && counter == 0 ) {region ++ ; counter ++ ; }  
			
			if ( distance(test_point , fin_collision ) < 2 && counter == 0 ) {region ++ ; counter ++ ; }  

			
			if (counter > 0 && counter < 4) counter ++; 
			else  counter = 0 ; 
								 
				
			if (region == 0 || region == 2 ) {  first_region.push_back(test_point);  first_partial_lenght ++ ; }
	
			else if (region == 1 ) 			 { second_region.push_back(test_point); second_partial_lenght ++ ; }
		}
	}
	
	std::cout << " Contour's Total Points : " << total_lenght <<  " ,   First Partial Lenght : "  << first_region.size()  <<  " ,   Second Partial Lenght : "  << second_region.size() << std::endl;
	 
	if (first_region.size() <= second_region.size())
		{
			for ( int i=0 ; i < first_region.size() ; i++)
			{
				cv::circle(test , first_region[i] , 2 , cv::Scalar( 200 , 200 , 200 ), 5, 8, 0 ); //Printing First Region
			}
		}
	else if (first_region.size() > second_region.size())
		{
			for ( int i=0 ; i < second_region.size() ; i++)
			{
				cv::circle(test , second_region[i] , 2 , cv::Scalar( 200 , 200 , 200 ), 5, 8, 0 ); //Printing Second Region
			}
		}	
		
	
	
	
	name = "Test " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    
    cv::imshow(name.c_str(), test ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	
	cv::Mat complete_path = test + original_dubins;
	
	name = "Complete Test (RAW) " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640); 
    cv::imshow(name.c_str(), complete_path ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	//CHANGING TO BINARY
	cv::cvtColor(complete_path, complete_path , CV_BGR2GRAY); 				
	
	//For Closing////////////////////////////////////
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));				 
	cv::morphologyEx(complete_path , complete_path , cv::MORPH_CLOSE, kernel);
	
	name = "CLOSING " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640); 
    cv::imshow(name.c_str(), complete_path ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	//Applying Gaussian to Smooth Edges.
	
	GaussianBlur(complete_path , complete_path , Size(5, 5), 0);
	
	
	//Finding Contours again to Store the new Trajectory	
	
	
	
	vector<vector<Point>> final_trajectory_points;
	
	findContours(complete_path , final_trajectory_points , RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	cv::Mat new_trajectory_map = cv::Mat::zeros(complete_path.size() , complete_path.type());
    
    drawContours(new_trajectory_map , final_trajectory_points , -1, cv::Scalar(255,255,255), CV_FILLED );  //Draw the found contours 
    		
					
	name = " Printing Fin Contours Result " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
	cv::imshow(name.c_str(), new_trajectory_map ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	cv::Mat smoothed_contour_map = cv::Mat::zeros(new_trajectory_map.size() , new_trajectory_map.type());
	
	
	
	
	
	
	//Approximating to a better shape through ApproxPolyDP

	
	std::vector<cv::Point> approx_curve;

	std::vector<std::vector<cv::Point>> smoothed_final_contour;
	
	
	for (int i=0; i<final_trajectory_points.size(); ++i)
	
	{
		approxPolyDP(final_trajectory_points[i], approx_curve , 8 , true); //How Strict the Filter is to Aproximate the contour to a Polygon
		
		smoothed_final_contour = {approx_curve}; 
		
		drawContours(smoothed_contour_map, smoothed_final_contour , -1 , cv::Scalar(255,255,255), cv::FILLED); //draw Polygon approximations 

	}
	
		
	name = " SMOOTHING Contours Result " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
	cv::imshow(name.c_str(), smoothed_contour_map); 
	cv::waitKey(0);
	//cv::destroyWindow(name.c_str());
	
	//EXPERIMENTAL:
	
	
	//filter2D(smoothed_contour_map ,smoothed_contour_map , -1, kernel, Point(-1,-1), 2.0, BORDER_REPLICATE);
	
	
	//AFTER CHECKING THAT IS THE SHORTEST PATH AND THERE IS NO COLLISIONS AND THAT WE ARE NOT GETTING OUT OF THE PIXELS ON THE BORDER WE ARE GOOD TO GO
		
	return smoothed_contour_map;
	
}


 




