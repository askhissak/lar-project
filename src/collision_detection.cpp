#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "reader.hpp"

using namespace std;
using namespace cv;

int thresh = 40 ;

int polyflag = 0;


cv::Mat print_contours(cv::Mat inputmap)
{
	
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY);
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( BW_Map, contours, idx, color, CV_FILLED, 8, hierarchy );
        
    }
	
	//std::string mapname = "Input for Contours";
	//cv::namedWindow(mapname.c_str(), CV_WINDOW_NORMAL);
	//cv::resizeWindow(mapname.c_str(), 512, 640);
	//cv::imshow(mapname.c_str(), BW_Map); 
	//cv::waitKey(0); 
	//cv::destroyWindow(mapname.c_str());
	
	return BW_Map;
}


bool collision_detection( cv::Mat inputmap , cv::Point target )
{
	
	cv::Mat BW_Map ;
	
	cv::cvtColor(inputmap, BW_Map, CV_BGR2GRAY); 
	
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	float dist;
	
	int i=0;
	
	findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	
	if (polyflag==0) {std::cout << "\n\n Detected " << contours.size() << " Contours ! \n  " << std::endl; polyflag++;}
	
	std::cout << "\n For the Point : " << target << std::endl;
	
	for( i = 0 ; i < contours.size() ; i++ ) // iterate through each contour. 
    {
		
		dist = (int)((cv::pointPolygonTest(contours[i] , target , true))*(-1));
		
		if (dist <= thresh) break;
		
		else std::cout << " Distance from Contour " << i <<  " : "  << dist << std::endl;
		
	}
	
	if (dist <= thresh) {std::cout << " Distance from Contour " << i <<  " : "  << dist <<  "   <---- THRESHOLD REACHED !!!!!! \n\n   "  << std::endl; return true;}
	else return false;

}
