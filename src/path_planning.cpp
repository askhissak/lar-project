// path_planning.cpp: PATH PLANNING
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "path_planning.hpp"
#include "dubins.hpp"
#include "map_construction.hpp"
#include "map_extraction.hpp"
// #include "polygon_offset.hpp"

#include "path.h"

bool obstacleOffset(cv::Mat input, std::vector<cv::Point> & dst, double offset)
{
	
	cv::Mat BW_Map ;
	
	cv::cvtColor(input, BW_Map, CV_BGR2GRAY);
	
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	
	cv::Mat expanded = cv::Mat::zeros( BW_Map.size(), CV_8UC3 );
	
	
	float pixels = offset; // Number of pixels expanded in respect to the original contour
	
	
	findContours(BW_Map , contours, hierarchy, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	for(int i=0;i<contours.size();++i)
	{
		std::cout<<"Offset corner "<<contours[i][0]<<std::endl;
	}
	
	// for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, cv::Scalar(255,255,255), pixels * 2);
	
	// findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
	// for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, Scalar(255,255,255), CV_FILLED );
	

	return true;
	
}

bool planDubins(Map & map_object, Path & path, std::vector<int> & order)
{
    std::vector<cv::Point> points_path;
	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	
	cv::Mat map = map_object.showMap(); //First map acquisition
	
	cv::Mat original_map = map; 

	cv::Point direction = map_object.robot.getDirectionSidePoint();

    double robot_x0 = map_object.robot.getCenter().x, robot_y0 = map_object.robot.getCenter().y, robot_th0 = getOrientation(direction, map_object.robot.getCenter()), 
			gate_xf = map_object.gate.getCenter().x, gate_yf = map_object.gate.getCenter().y, gate_thf = 0; 
    double x0, y0, th0, xf, yf, thf; 
    double Kmax = 0.01;
    double radius = 50;
	int npts = 1000;
	double s1,s2,s3;

	DubinsCurve curve;
	// int next = 1;

	// for(int j=0;j<map_object.victims.size();++j)
	// {
		for(int i=0;i<order.size();++i)
		{
			// if(order[i] == next)
			// {
				if(i==0)
				{
					x0 = robot_x0;
					y0 = robot_y0;
					th0 = robot_th0;
					xf = map_object.victims[order[i]].center.x;
					yf = map_object.victims[order[i]].center.y;
					thf = getOrientation(map_object.victims[order[i]].center,map_object.robot.getCenter());
				}
				else
				{
					x0 = xf;
					y0 = yf;
					th0 = thf;
					xf = map_object.victims[order[i]].center.x;
					yf = map_object.victims[order[i]].center.y;
					thf = getOrientation(map_object.victims[order[i]].center, cv::Point(x0,y0));
				}

				curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 


				for(int j=0; j<npts;++j)
				{
					s1 = curve.arc1.L/npts*j;
					circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
				}
				for(int j=0; j<npts;++j)
				{
					s2 = curve.arc2.L/npts*j;
					circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
				}
				for(int j=0; j<npts;++j)
				{	
					s3 = curve.arc3.L/npts*j;
					circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
				}
			// }
		}
	// 	next++;
	// } 

	x0 = xf;
	y0 = yf;
	th0 = thf;
	xf = gate_xf;
	yf = gate_yf;
	thf = gate_thf;

	curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 

	for(int j=0; j<npts;++j)
	{
		s1 = curve.arc1.L/npts*j;
		circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{
		s2 = curve.arc2.L/npts*j;
		circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{	
		s3 = curve.arc3.L/npts*j;
		circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
        
    //INITIAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(robot_x0,robot_y0), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(robot_x0,robot_y0) , cv::Point(robot_x0+radius*cos(robot_th0), robot_y0+radius*sin(robot_th0)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
    //FINAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(gate_xf,gate_yf), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(gate_xf,gate_yf) , cv::Point(gate_xf+radius*cos(gate_thf), gate_yf+radius*sin(gate_thf)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
	
	showImage("Dubins path", out+map);

    return true;
                
}

bool planMission(cv::Mat const & map, Map & map_object, Path & path, std::vector<int> & order)
{
	if(!planDubins(map_object, path, order))
	{
		std::cerr << "(Critical) Failed to plan a path using Dubins curve algorithm!" << std::endl;
		return false;
	}

	// std::vector<cv::Point> dst;

	// for(int i=0;i<map_object.obstacles.size();++i)
	// {
	// 	if(!obstacleOffset(map, dst, 10))
	// 	{
	// 		std::cerr << "(Critical) Failed to plan a path using Dubins curve algorithm!" << std::endl;
	// 		return false;
	// 	}
	// 	for(int i=0;i<contours.size();++i)
	// 	{
	// 		std::cout<<"Offset corner "<<contours[i][0]<<std::endl;
	// 	}

	// }
	

	// if(!polygonOffset(map, map_object))
	// {
	// 	std::cerr << "(Critical) Failed to plan a path using Dubins curve algorithm!" << std::endl;
	// 	return false;
	// }

	return true;
}