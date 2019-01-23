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


cv::Mat print_desired_contour(cv::Mat inputmap, int contour_idx , cv::Point &contour_center , int &contour_radius )
{
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY);
	
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	//For getting the desired circle's information
	vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center_aux( contours.size() );
    vector<float>radius_aux( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	
	{ 
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		minEnclosingCircle( (Mat)contours_poly[i], center_aux[i], radius_aux[i] );
	}
	
	contour_center = center_aux[contour_idx];
		   
	contour_radius = (int)radius_aux[contour_idx];
	
	std::cout << "\n\n Detected " << contours.size() << " Contours ! \n  " << std::endl;
	
	//Writing the useful Contour and Deleting the useless ones
	
	int idx = 0;
    
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    { 
        if (idx == contour_idx) drawContours( BW_Map, contours, idx , cv::Scalar(255,255 ,255) , 1 , 8, hierarchy);
        
        else drawContours( BW_Map, contours, idx , cv::Scalar( 0 , 0 ,0) , CV_FILLED, 8, hierarchy);
           
    } 
    
   
	return BW_Map;
}


bool collision_detection( cv::Mat inputmap , cv::Point target , int thres , int remapper , int &contour_index )
{
	
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY); 
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	float dist;
	
	int i=0;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	
	if (polyflag==0) {std::cout << "\n\n Detected " << contours.size() << " Contours ! \n  " << std::endl; polyflag++;}
	
	//std::cout << "\n For the Point : " << target << std::endl;
	
	for( i = 0 ; i < contours.size() ; i++ ) // iterate through each contour. 
    {
		
		dist = (int)((cv::pointPolygonTest(contours[i] , target , true))*(-1));
		
		if (dist <= thres) break;
		
		//else std::cout << " Distance from Contour " << i <<  " : "  << dist << std::endl;
		
	}
	
	if (dist <= thres) 
	{
		std::cout << " Distance from Contour " << i <<  " : "  << dist <<  "   <---- THRESHOLD REACHED !!!!!! \n\n   "  << std::endl;
		if      (dist == 0 && remapper != 3 ) {contour_index = i; }
		else if (dist == 0 && remapper == 3 && output_trigger == 0 )   {contour_index = 100; output_trigger = 100;  } //Initial Value
		else if (dist <  0 && remapper == 3 && output_trigger == 100 ) {contour_index =0 ; } 
		else if (dist == 0 && remapper == 3 && output_trigger == 100)  {contour_index = 200; output_trigger = 200;  } //Final Value
		else if (dist == 0 && remapper == 3 && output_trigger == 200)  {contour_index = 0;} 
		return true;
	}
	
	else return false;

}


cv::Mat rounder(cv::Mat input)
{
	cv::Mat BW_Map ;
	
	cv::cvtColor(input, BW_Map, CV_BGR2GRAY);
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	float scale = 1.5; //50% bigger than the original bounding circle containing them
	
	int thresh = 100;
	int max_thresh = 255;
	RNG rng(12345);
	
	
	// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

	  for( int i = 0; i < contours.size(); i++ )
		 { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		   boundRect[i] = boundingRect( Mat(contours_poly[i]) );
		   minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
		 }


	  // Draw polygonal contour + bonding rects + circles
	  cv::Mat rounded = Mat::zeros( BW_Map.size(), CV_8UC3 );
	  for( int i = 0; i< contours.size(); i++ )
		 
		 {
		   //drawContours( rounded, contours_poly, i, cv::Scalar(255,0,0), 1, 8, vector<Vec4i>(), 0, Point() );
		   //rectangle( rounded, boundRect[i].tl(), boundRect[i].br(),cv::Scalar(255,255,255), 2, 8, 0 );
		   circle( rounded , center[i], (int)radius[i]*scale , cv::Scalar(255,255,255), 2, 8, 0 );
		 }

	/*
	int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( BW_Map, contours, idx, color, CV_FILLED, 8, hierarchy );
        
    }
    */
    
    
	return rounded;
	
}

cv::Mat crop_contour( cv::Mat isolated_contour , cv::Point ini_collision , cv::Point fin_collision , cv::Point contour_center , int contour_radius )
{
	cv::Mat cropped_contour = isolated_contour;
	
	
	//Calculating the Angle
	
	//For the initial collision
	//Line1 :
	
	
	cv::circle(cropped_contour , contour_center, contour_radius, cv::Scalar(0,0,0) , 1 , 8, 0);
	
	
	double Initial_Angle ( atan2 ( ini_collision.y-contour_center.y , ini_collision.x - contour_center.x ) * 180 / M_PI ) ;
	
	double Final_Angle ( atan2 ( fin_collision.y-contour_center.y , fin_collision.x - contour_center.x ) * 180 / M_PI ) ;
	
	
	std::cout << "   Initial_ Angle " << Initial_Angle << " \n  " << "   Final_ Angle " << Final_Angle << " \n  " << std::endl;
	
	
	//The first one (up) goes from 180 to 0;
	
	
	cv::ellipse(cropped_contour ,contour_center,cv::Size(contour_radius,contour_radius),0,Initial_Angle,Final_Angle,cv::Scalar(0,255,0),8,0);
	
	//The DownRight Red Halfcircle takes from 0 to 180;
	
	
	
	
	double startAngleDownright = 0;
	//cv::ellipse(cropped_contour,contour_center,cv::Size(contour_radius,contour_radius),0,startAngleDownright,startAngleDownright+180,cv::Scalar(0,255,0),8,0);
	
	return cropped_contour;
	
}



