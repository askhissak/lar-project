// robot_project.cpp:
#include <opencv2/opencv.hpp>
#include <vector>

#include "camera_calibration.hpp"
#include "map_extraction.hpp"
#include "map_construction.hpp"
#include "shape_detection.hpp"
#include "digit_recognition.hpp"
#include "collision_detection.hpp"
#include "path_planning.hpp"

#include "path.h"
#include "robot_project.h"
#include <chrono>

using namespace std::chrono;


bool RP_developer_session = false ; // if true  -> Retrieves desired debugging and log content 
									// if false -> Process everything without graphical output 
									
int Path_Planning_Mode = 2 ;  //if 1 -> The approximation is made throught the Voronoi Diagram Implementation
							  //if 2 -> The approximation is made through a Collision Detection Implementation				
									
Map map_object;
cv::Mat robot_plane, map;
std::vector<int> order;

std::vector<cv::Point> points_path;
std::vector<cv::Point> new_path;
std::vector<std::vector<cv::Point>> inflated_contours;

// Constructor taking as argument the command line parameters
RobotProject::RobotProject(int argc, char * argv[])
{

}

// Method invoked to preprocess the map (extrinsic calibration + reconstruction of layout)
bool RobotProject::preprocessMap(cv::Mat const & img)
{
	map_object = Map(cv::Point(0,0), MAP_LENGTH, MAP_WIDTH);

	// calibrateCamera();

	//Image -> Map (borders only)
    if(!extractMap(img, map, robot_plane, map_object))
	{
		std::cerr << "(Critical) Failed to extract the map!" << std::endl;
		return false;
	}

	//Shapes
    // circles = processImage(map , gate_center );

	if(!buildMap(map, robot_plane, map_object))
	{
		std::cerr << "(Critical) Failed to construct the map!" << std::endl;
		return false;
	}

	//Digits
    if(!recognizeDigits(map, map_object.victims, order))
	{
		std::cerr << "(Critical) Failed to recognize the digits!" << std::endl;
		return false;
	}

    return true;

}

// Method invoked when a new path must be planned (detect initial robot position from img)
bool RobotProject::planPath(cv::Mat const & img, Path & path)
{
	if (Path_Planning_Mode == 1)
	{
		if(!planMission(map, map_object, path, order))
		{
			std::cerr << "(Critical) Failed to plan a path!" << std::endl;
			return false;
		}

		printPosePath(path);
	
	}
	
	else if (Path_Planning_Mode == 2)
	{	
		if(!run_collision_detection( map , points_path , new_path , inflated_contours ))
		{
			std::cerr << "(Critical) Failed to create a new path!" << std::endl;
			return false;
		}
		
	}

    return true;

}

// Method invoked periodically to determine the position of the robot within the map.
// The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const & img, 
            std::vector<double> & state)
{
	
	
	high_resolution_clock::time_point t1 = high_resolution_clock::now();  // <- initial Process Time Measurement 
   
	if(!extractMapLocalize(img, map, robot_plane, map_object))
	{
		std::cerr << "(Critical) Failed to extract the map!" << std::endl;
		return false;
	}

	if(!findRobot(map, robot_plane, map_object))
	{
		std::cerr << "(Critical) Failed to find the robot!" << std::endl;
		return false;
	}

	state.push_back(map_object.robot.pose.x/1000.0);
	state.push_back(1.05-map_object.robot.pose.y/1000.0);
	state.push_back(-map_object.robot.pose.theta);
	std::cout<<"x = "<<map_object.robot.pose.x/1000.0<<" y = "<<1.05-map_object.robot.pose.y/1000.0<<" theta = "<<-map_object.robot.pose.theta<<std::endl;
    
    high_resolution_clock::time_point t2 = high_resolution_clock::now(); // <- Final Process Time Measurement 

    auto duration = duration_cast<microseconds>( t2 - t1 ).count(); // Process Time Calculation 

    std::cout << "\n\n PROCESSING TIME : " << duration/1000 << "MILISECONDS \n\n " ;
    
	return true;
}
