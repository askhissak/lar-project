#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h> 

#include "collision_detection.hpp"
using namespace std;
using namespace cv;

int polyflag = 0;


int output_trigger =0;

std::string name = "NAME OF THE WINDOW" ;

cv::RNG rng( 0xFFFFFFFF );
    
cv::Scalar random_color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));

bool first_collision_detection (cv::Mat original_img , 
								cv::Mat &original_map , 
								std::vector<cv::Point> pointPath , 
								int thres , 
								int &collisions_number , 
								std::vector<cv::Point> &ini_points , 
								std::vector<cv::Point> &fin_points , 
								int &contours_number , 
								int &contour_idx , 
								std::vector<cv::Point> &desired_contour_vector , 
								std::vector<int> &colliding_contours_idx , 
								std::vector<std::vector<cv::Point>> &original_contours )
{
	
	cv::Mat hsv_img;
	cv::cvtColor(original_map, hsv_img, cv::COLOR_BGR2HSV);
	cv::Mat red_mask_low, red_mask_high, red_mask;
	
	
	vector<Vec4i> hierarchy;
	
                               // 0  30 30             20
	cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(18, 255, 255), red_mask_low);
	                               // 150  30 30             180         
	cv::inRange(hsv_img, cv::Scalar(150, 0, 0), cv::Scalar(200, 255, 255), red_mask_high);
	  
	cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks
	  
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)); //5,5
	  
	cv::erode(red_mask , red_mask , kernel, cv::Point(-1, -1), 2);

	findContours(red_mask, original_contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	cv::Mat temp = cv::Mat::zeros( original_map.size(), original_map.type());
	
	
	
	drawContours(temp , original_contours, -1, cv::Scalar(255 , 255 ,255), CV_FILLED);  //Draw the found contours
	
	
	name = "Contours";
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 640, 512);
	cv::imshow(name.c_str(), temp);
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());        
	
	temp = cv::Mat::zeros(original_map.size(), original_map.type());
	
	
	float dist = 1000;
	
	
	for( int actual_contour = 0  ; actual_contour < original_contours.size() ; actual_contour++ )
	
	{
		
		int new_x = 0 ; 
		int new_y = 0 ;
		int old_x = 0 ;
		int old_y = 0 ;
		
		for(int actual_point = 0 ; actual_point < pointPath.size() ; actual_point++  )
		{
			new_x = pointPath[actual_point].x ;
			new_y = pointPath[actual_point].y ;
				
			if ((new_x != old_x || new_y != old_y ))
			{
				dist = (int)((cv::pointPolygonTest(original_contours[actual_contour] , cv::Point(new_x,new_y) , true))*(-1));
				
				if (dist < thres ) {colliding_contours_idx.push_back(actual_contour); break; }
			}
			
			else {  old_x = new_x ; old_y = new_y ;}
		}
	}
	
	std::cout << " \n\n The trajectory collide with " << colliding_contours_idx.size() <<  " obstacles ! " << std::endl;
	
	
	temp = cv::Mat::zeros(original_map.size() , original_map.type());
	
	
	for(int i=0 ; i < pointPath.size() ; i++ )
		{
			cv::circle( temp , pointPath[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
		}	
	
	
	for(int i = 0 ; i < colliding_contours_idx.size() ; i ++)
	{
		cv::polylines(temp , original_contours[colliding_contours_idx[i]] , true , cv::Scalar(255,255,255) , 1 , 8 ) ;
	}
	
	
	name = " COLLIDING OBSTACLES " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
	cv::imshow(name.c_str(), temp ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	original_map = temp ;
	
	if (colliding_contours_idx.size() > 0) return true;
	else return false;
	
}

//NEW ADDED THE DESIRED_CONTOUR AREA
cv::Mat inflate_contours( cv::Mat original_map , 
						  std::vector<std::vector<cv::Point>> original_contours , 
						  std::vector<std::vector<cv::Point>> &inflated_contours , 
						  std::vector<int> &colliding_contours_idx  ,  
						  std::vector<cv::Point> pointPath , 
						  int thres  , 
						  std::vector<cv::Point> &ini_points , 
						  std::vector<cv::Point> &fin_points )
{
	
	int old_x = 0; int new_x = 0;
	int old_y = 0; int new_y = 0;
	
	colliding_contours_idx.clear();
	
	ini_points.clear();
	
	fin_points.clear();
	
	cv::Mat expanded  = cv::Mat::zeros(original_map.size() , original_map.type() );
	
	cv::cvtColor(original_map , expanded , CV_BGR2GRAY);
	
	cv::Mat temp  = expanded.clone();
	
	cv::Mat original_map_temp  = cv::Mat::zeros(original_map.size() , expanded.type() );
	
	
	//dilate(BW_Map, BW_Map, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(471, 471)));
	//erode(BW_Map, BW_Map, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(471, 471)));

	
	for( int i = 0; i < original_contours.size(); i++ ) { drawContours(expanded, original_contours, i, cv::Scalar(255,255,255), 40 * 2); }
	
	for( int i = 0; i < original_contours.size(); i++ ) { drawContours(original_map_temp , original_contours, i, cv::Scalar(100,100,100), CV_FILLED); }
	
	
	GaussianBlur(expanded, expanded , cv::Size(203, 203), 181 );
	
	cv::threshold(expanded,expanded , 20 , 255 , THRESH_BINARY) ;
		
	string name = "EXPANDED AND MERGED MAP " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    cv::imshow(name.c_str(),  expanded - original_map_temp  ); //expanded
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	
	findContours( expanded , inflated_contours , RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	std::cout << " ---- \n Found " << inflated_contours.size() << " Inflated Contours \n" <<std::endl;
	
	for( int actual_contour = 0 ; actual_contour < inflated_contours.size() ; actual_contour++ ) // iterate through each new inflated contour. 
    {
		old_x = 0; new_x = 0;
		new_y = 0; new_y = 0;
		
		bool inside = false;
		
		int counter = 0;
		
		for(int actual_point =0 ; actual_point < pointPath.size() ; actual_point++ )
		
		{
			new_x = pointPath[actual_point].x ;
			new_y = pointPath[actual_point].y ;
				
			if ((new_x != old_x || new_y != old_y ))
			{
				int dist = (int)((cv::pointPolygonTest(inflated_contours[actual_contour] , cv::Point(new_x,new_y) , true))*(-1));
				
				if (dist < thres  && inside == false ) //Entering the Contour
				{
					ini_points.push_back(cv::Point(new_x,new_y));
					colliding_contours_idx.push_back(actual_contour); 
					inside = true;
				}
				
				else if(dist > thres  && inside == true ) //Getting out of the contour
				{
					fin_points.push_back(cv::Point(new_x,new_y));
					inside = false;
				}
			}
			
			else 
			{  
				old_x = new_x ; 
				old_y = new_y ;
			}	
		}	
	}
	
	std::cout << " \n\n The trajectory has " << colliding_contours_idx.size() <<  " different collisions ! " << std::endl;
	
	for (int i = 0 ; i < colliding_contours_idx.size() ; i++ )
	
	{
		random_color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		
		std::cout << "Collision Nr. " << i << " : In contour Nr. " << colliding_contours_idx[i] << " with initial point : " << ini_points[i] << " and final Point :" << fin_points[i] << std::endl;
		
		cv::circle( temp , ini_points[i] , 8 , random_color , 5, 8, 0 );
		cv::circle( temp , fin_points[i] , 8 , random_color , 5, 8, 0 );
		
	}
	
	for(int i=0 ; i< pointPath.size() ; i++ ) {cv::circle( temp , pointPath[i] , 2 , cv::Scalar(50,50,50) , 5, 8, 0 );}

	name = "COLLISIONS ON THE INFLATED MAP " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    cv::imshow(name.c_str(), expanded + temp ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	
	
	
	return expanded;
}

//NEW Implementation in this function 

int distance( cv::Point P1 , cv::Point P2)

{
	int dist = sqrt(((P1.x- P2.x) * (P1.x-P2.x)) + ((P1.y - P2.y ) * (P1.y - P2.y)));
	
	return dist;
	
}

	
cv::Mat create_new_path( std::vector<cv::Point> pointPath , 
						 std::vector<int> &colliding_contours_idx , 
						 std::vector<std::vector<cv::Point>> &inflated_contours , 
						 cv::Mat map , 
						 int thres, 
						 std::vector<cv::Point> &ini_points , 
						 std::vector<cv::Point> &fin_points, 
						 std::vector<cv::Point> &new_path )

{	
	//We have to iterate through here to find the collisions until we clear all the collisions with that contour and we iterate to verify if we have another collision in another contour   
	
	std::vector<cv::Point> first_region ;
	
	std::vector<cv::Point> second_region ;
	
	cv::Mat shorter_path  = cv::Mat::zeros(map.size() , map.type());
	
	cv::Mat complete_path = cv::Mat::zeros(map.size() , map.type());
	
	
	//DEBUGGING/////////////////////////////////////////////////////////
	cv::Mat test_map  = cv::Mat::zeros(map.size() , map.type());
	///////////////////////////////////////////////////////////////////
	
	int total_lenght = 0 ;
	
	int first_partial_lenght =0 ;
	
	int second_partial_lenght = 0;

	int counter = 0 ;
	
	int region = 0 ;
	
	int current_contour = 0 ;
	
	bool stop = false;
	
	bool start = false;
	
	string name;
	
	new_path.clear();
	
 	//we include a counter to discard false positives.
		
	 //Starting the new path 
	
	 // For every iteration through the number of collisions with this particular contour the algorithm :
	 
	 // - Takes the exit point of the previous trajectory or in case of the initial point initial robot's position
	 
	 // - Takes the initial entry and exit point to the desired contour stored by the corresponding index
	 
	 // - Stores the initial trajectory , new rounding trajectory until the final collision point and iterate to the 
	 //   next collision point to continue the process until no collision are detected (end of the index)
	 
	 // - Finally stores the remaining path until the gate's central point is reached   
	 
	
	for(current_contour = 0 ; current_contour < colliding_contours_idx.size() ; current_contour++ )
	{
		std::cout << "\n\n Solving the Collisions for the Contour Nr. " << colliding_contours_idx[current_contour] <<  "  , Initial Point : " << ini_points[current_contour] << "  , Final Point : " << fin_points[current_contour] << std::endl;
		
		stop = false;
		
		start = false;
		
		total_lenght = 0 ;
	
		first_partial_lenght =0 ;
		
		second_partial_lenght = 0;
		
		counter = 0 ;
		
		region = 0 ;
		
		first_region.clear();
	
		second_region.clear();
		
		////To save the first part of the initial trajectory 
		if (current_contour == 0) 
		{
			for(int i= 0 ; i < pointPath.size() ; i++ )
			{
				if ( pointPath[i] == ini_points[current_contour] ) stop = true ;  
				
				else if ( stop == false ) new_path.push_back(pointPath[i]);
			}
			
			
			//////////////////////////////////////////////////////////////////////
			////      Hardcoding Tool
			//////////////////////////////////////////////////////////////////////
			test_map  = cv::Mat::zeros(map.size() , map.type());
			
			random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
			
			for(int i=0 ; i < new_path.size() ; i++ )
			{
				cv::circle( test_map , new_path[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
			}	
			
			name = "FIRST SEGMENT" ;
			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
			cv::resizeWindow(name.c_str(), 512, 640);   		
		    
		    cv::imshow(name.c_str(), test_map ); 
			cv::waitKey(0);
			cv::destroyWindow(name.c_str());
			
			test_map  = cv::Mat::zeros(map.size() , map.type());
			
			/////////////////////////////////////////////////////////////////////
			
		}
		
		//ONCE THE PREVIOS COLLISION IS SOLVED IS TIME TO DRAW THE TRAJECTORY FROM 
		//THE END TO THE PREVIOS COLLISION TO THE BEGINNING OF THE NEW ONE
		
		////TO DRAW THE SEGMENT FROM THE END OF THE COLLISION TO THE BEGINNING OF THE NEW ONE
		
		else if ( current_contour > 0 )
		
		{
			start = false ;
			stop  = false ;
			
			for(int i=0 ; i < pointPath.size() ; i++ )
			{
				if ( start == false  && stop == false  && pointPath[i] == fin_points[current_contour-1 ] ) start = true ;  
				
				else if ( start == true && stop == false && pointPath[i] != ini_points[current_contour] ) new_path.push_back(pointPath[i]);
				
				else if ( start == true && stop == false && pointPath[i] == ini_points[current_contour] ) stop = true;
			}
			
			//////////////////////////////////////////////////////////////////////
			////      Hardcoding Tool
			//////////////////////////////////////////////////////////////////////
			test_map  = cv::Mat::zeros(map.size() , map.type());
			
			random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
			
			for(int i=0 ; i < new_path.size() ; i++ )
			{
				cv::circle( test_map , new_path[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
			}	
			
			name = "Accumulated Path after the collision Nr." + std::to_string( current_contour-1 ) ;
			cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
			cv::resizeWindow(name.c_str(), 512, 640);   		
		    
		    cv::imshow(name.c_str(), test_map ); 
			cv::waitKey(0);
			cv::destroyWindow(name.c_str());
			
			test_map  = cv::Mat::zeros(map.size() , map.type());
			
			/////////////////////////////////////////////////////////////////////
			
		}
		
		//Comparing lenghts and storing the partial lenght value
		
		for(int point_inside_current_contour = 0; point_inside_current_contour < inflated_contours[colliding_contours_idx[current_contour]].size(); point_inside_current_contour++) 
		{	
			total_lenght++; //for a later comparison
			
			//Before Colliding with the entry point the trajectory is in region 0 , and then after the traj. becomes 1 
			//until it collides again with the final point , where it becomes 2 until the loop finishes and we collide with the region 0 
			//The region 1 is the "below part" and the region 0 and 2 are the upper parts.
			
			//Region 0 : Any extreme point has been reached
			if ( distance(cv::Point( inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].x , inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].y ) , ini_points[current_contour]) < 2 && counter == 0 ) {region ++ ; counter ++ ; }  
			
			//Region 1 : One point has been reached
			else if ( distance(cv::Point(inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].x , inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].y ) , fin_points[current_contour] ) < 2 && counter == 0 ) {region ++ ; counter ++ ; }  
			
			//Counter to avoid counting twice (false positives primitive filter);
			if (counter > 0 && counter < 5) counter ++; 
			else  counter = 0 ; 
								 
			//Region 2 : We touched the secon point so the path 2 and 0 must be added to form the second region
			if (region == 0 || region == 2 ) 
			{  
				first_region.push_back(cv::Point(inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].x,
												 inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].y)) ; 
				
				first_partial_lenght ++ ; 
			}
	
			else if (region == 1 ) 			 
			{
				second_region.push_back(cv::Point (inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].x,
									    inflated_contours[colliding_contours_idx[current_contour]][point_inside_current_contour].y )) ; 
				second_partial_lenght ++ ; 
			
			}
			
			
			//Implement here a case to detect when a point gets out of the border with maybe comparing x and y to a maximum and minimum value
			
		}
		
		std::cout << " Contour's Total Points : " << total_lenght <<  " ,   First Partial Lenght : "  << first_region.size()  <<  " ,   Second Partial Lenght : "  << second_region.size() << std::endl;
		 
		if (first_region.size() <= second_region.size())
			{
				for ( int i=0 ; i < first_region.size() ; i++)
				{
					cv::circle( shorter_path , first_region[i] , 2 , cv::Scalar( 200 , 200 , 200 ), 5, 8, 0 ); //Printing First Region
					new_path.push_back(first_region[i]);
				}
			}
		else if (first_region.size() > second_region.size())
			{
				for ( int i=0 ; i < second_region.size() ; i++)
				{
					cv::circle( shorter_path , second_region[i] , 2 , cv::Scalar( 200 , 200 , 200 ), 5, 8, 0 ); //Printing Second Region
					new_path.push_back(second_region[i]);
					
				}
			}		
		
		//////////////////////////////////////////////////////////////////////
		////      Hardcoding Tool (ARCS)
		//////////////////////////////////////////////////////////////////////
		test_map  = cv::Mat::zeros(map.size() , map.type());
		
		random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
		
		for(int i=0 ; i < first_region.size() ; i++ )
		{
			cv::circle( test_map , first_region[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
		}
			
		name = "FIRST REGION " ;
		cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(name.c_str(), 512, 640);   		
	    
	    cv::imshow(name.c_str(), test_map ); 
		cv::waitKey(0);
		cv::destroyWindow(name.c_str());
		
		test_map  = cv::Mat::zeros(map.size() , map.type());
		
		 // -----------------------------------------------------------------
		
		test_map  = cv::Mat::zeros(map.size() , map.type());
		
		random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
		
		for(int i=0 ; i < second_region.size() ; i++ )
		{
			cv::circle( test_map , second_region[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
		}
			
		name = "SECOND REGION " ;
		cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(name.c_str(), 512, 640);   		
	    
	    cv::imshow(name.c_str(), test_map ); 
		cv::waitKey(0);
		cv::destroyWindow(name.c_str());
		
		test_map  = cv::Mat::zeros(map.size() , map.type());
		
		/////////////////////////////////////////////////////////////////////
		
		
		
		
	
	} // FOR LOOP (COLLISION IDX)
	
	//////FINAL PATH //////////////////////
	
	start = false ;
	
	for(int i=0 ; i < pointPath.size() ; i++ )
	{
		     if ( start == false  && pointPath[i] == fin_points[current_contour -1 ] ) start = true ;  
		
		else if ( start == true ) new_path.push_back(pointPath[i]);
	}
	
	////////////////////////////////////////////////////////////////////
	//      Hardcoding Tool (ARCS)
	////////////////////////////////////////////////////////////////////
	test_map  = cv::Mat::zeros(map.size() , map.type());
	
	random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
	
	
	for(int i=0 ; i < new_path.size() ; i++ )
	{
		cv::circle( test_map , new_path[i] , 2 , random_color , 5, 8, 0 ); //Printing Second Region
	}
		
	name = "Complete Path " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    
    cv::imshow(name.c_str(), test_map ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());
	
	
	///////////////////////////////////////////////////////////////////
	
	return test_map;
}	

	
bool collision_detection( std::vector<cv::Point> new_path , 
						  std::vector<std::vector<cv::Point>> contours , 
						  int thres , 
						  cv::Mat original_map )
{
	int new_x ; 
	int new_y ;
	int old_x ;
	int old_y ;
	
	float dist = 1000;
	
	bool collision_status = false;
	
	for(int current_contour = 0 ; current_contour < contours.size() ; current_contour++)
	{
		for(int current_point =0 ; current_point < new_path.size() ; current_point++)
		{
			new_x = new_path[current_point].x ;
			new_y = new_path[current_point].y ;
				
			if ((new_x != old_x || new_y != old_y ))
			{
				dist = (int)((cv::pointPolygonTest(contours[current_contour] , cv::Point(new_x,new_y) , true))*(-1));
			
				if (dist < thres )
				{
					collision_status = true ;
					std::cout << "\n \n \n " << cv::Point(new_x,new_y) << std::endl;
		
					break;
				}  
			}
			
			else 
			{  
				old_x = new_x ; 
				old_y = new_y ;
			}	
		}
		if (collision_status == true) break ;
	}
	
	if (collision_status == true)
	{
		std::cout << "\n\n ----------------- THERE IS STILL COLLISION(S) DETECTED IN THE NEW MAP , BACK TO NEW PATH CREATOR !!!  --------- \n\n" << std::endl;
		//DEBUGGING/////////////////////////////////////////////////////////
		cv::Mat test_map  = cv::Mat::zeros(original_map.size() , original_map.type());
		///////////////////////////////////////////////////////////////////
			
		////////////////////////////////////////////////////////////////////
		//      Hardcoding Tool (ARCS)
		////////////////////////////////////////////////////////////////////
		
		random_color = cv::Scalar(rand() % 255,rand() % 255,rand() % 255); 
		
		cv::circle( test_map , cv::Point(new_x , new_y ) , 10 , random_color , 5, 8, 0 ); //Printing Second Region
			
		string name = "Collision_Point " ;
		cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(name.c_str(), 512, 640);   		
	    
	    cv::imshow(name.c_str(), test_map ); 
		cv::waitKey(0);
		cv::destroyWindow(name.c_str());
	
	}
	
	return collision_status ;
}

bool run_collision_detection( cv::Mat map , 
							  std::vector<cv::Point> pointPath , 
							  std::vector<cv::Point> &new_path , 
							  std::vector<std::vector<cv::Point>> &inflated_contours )
{
	//-----------------------------------------------------------------------------------------------------------------------------
    
    cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));

    // int npts = 10000;
    
    cv::Mat original_map = map; 
	
    cv::Mat new_desired_contour = cv::Mat::zeros(out.size() , out.type());
    
    cv::Mat expanded = cv::Mat::zeros(map.size() , map.type());
    
    cv::Mat isolated_contour = cv::Mat::zeros(out.size() , out.type());
				
	std::vector<cv::Point> ini_points ;
	
	std::vector<cv::Point> fin_points ;
	
	cv::Scalar initial_color = cv::Scalar(50,50,50);
	
	cv::Scalar final_color = cv::Scalar(100,100,100);
	
	std::vector<int> colliding_contours_idx ;
    
    std::vector<std::vector<cv::Point>> original_contours;
    
    cv::Point ini_collision;
    
    cv::Point fin_collision ;
    
    cv::Point collision_point;
    
    string name ;
    
    int old_x=0; 
    int new_x=0;
    
    int old_y=0;
    int new_y=0;

    
   bool collision_trigger = false ;
    
    int thres ;
    
    int thres_safe = 40 ; //Safe value for first approximation (40)

	int thres_limit = 0;
	
	int collision_status = 0;
	
	int collisions_number = 0 ;
    
    int contour_idx ;
    
    bool first_loop = true ;
     
    
    double desired_contour_area = 0;   //NEW
    
    bool contour_printer = true ;
    
    int status = 0 ;
    
    
    
	
	int contours_number = 0 ;
	
	std::vector<cv::Point> desired_contour_vector;
	
	
	int current_collision = 0 ;
	
	collision_trigger = true ;
	
	//Working Image + 
	
	name = "Processed Image " ;
	cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
	cv::resizeWindow(name.c_str(), 512, 640);   		
    
    cv::imshow(name.c_str(), map ); 
	cv::waitKey(0);
	cv::destroyWindow(name.c_str());

	 
    while( collision_trigger == true )
    {
		
		//status = 0 // Comparing Original Trajectory with Otiginal obstacle's map in a conservative threshold to detect if the ideal initial path has collisions
						
						//If there is no collisions the algorithm is finished and we retrieve this path as the assigned path
		 
		//status = 1 // Collision(s) Detected , we have to inflate the contours and generate a new path to avoid all the collisions
			
						//We have to iterate in this loop until all the collisions has been cleare
		
		if ( status == 0 )
		{
			
			if (first_collision_detection ( map , original_map , pointPath , thres_safe , collisions_number , ini_points , fin_points , contours_number , contour_idx , desired_contour_vector , colliding_contours_idx , original_contours ) == true )
			{
				std::cout << "\n\n ---------------- COLLISION(S) DETECTED IN THE ORIGINAL MAP --------------------------------- \n\n" << std::endl;
				collision_trigger = true; 
				status = 1;
			}
			
			else 
			{
				std::cout << "\n\n ----------------- NO COLLISION(S) DETECTED IN THE ORIGINAL MAP , THE TRAJECTORY IS SAFE --------- \n\n" << std::endl;
				collision_trigger = false ;
				
			}

		}
		
		else if (collision_trigger == true && status > 0 ) 
		{
				
			std::cout << "\n\n ---------------- COLLISION DETECTOR (INFLATED CONTOURS) --------------------------------- \n\n" << std::endl;
		
			thres = thres_limit; 
			
			if (status == 1 ) expanded = inflate_contours( original_map , original_contours , inflated_contours , colliding_contours_idx  ,  pointPath , thres , ini_points  , fin_points ) ;
			
			create_new_path( pointPath , colliding_contours_idx , inflated_contours , map , thres, ini_points , fin_points , new_path ) ;
			
			
			//Verify one more time that there is no collisions anymore
			
			if ((collision_detection( new_path , inflated_contours , thres , original_map )) == true ) //There is still collisions ..
			{
				std::cout << "\n\n ----------------- THERE IS STILL COLLISION(S) DETECTED IN THE NEW MAP , BACK TO NEW PATH CREATOR !!!  --------- \n\n" << std::endl;
			}
			
			else //No collisions in the new path 
			{
				std::cout << "\n\n ----------------- NO COLLISION(S) DETECTED IN THE NEW MAP , THE TRAJECTORY IS SAFE --------- \n\n" << std::endl;
				collision_trigger = false ;
			}	
			
			status ++ ;
			
			if (collision_trigger == true && status > 10 )
			{
				std::cout << "\n\n ----------------- ERRROR , CHECK THE PATH TO DEBUG !!!  --------- \n\n" << std::endl;
				collision_trigger = true ;
				break;
			}
		
		}
		
		if (collision_trigger == true && status > 10 ) break;
		
	}//While

		if (collision_trigger == true) return false ; 
		
		else 						   return true  ;
}



 
