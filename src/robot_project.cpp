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

Map map_object;
cv::Mat robot_plane, map;
std::vector<int> order;

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

	if(!planMission(map, map_object, path, order))
	{
		std::cerr << "(Critical) Failed to plan a path!" << std::endl;
		return false;
	}

	printPosePath(path);

    return true;

}

// Method invoked periodically to determine the position of the robot within the map.
// The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const & img, 
            std::vector<double> & state)
{
	extractMapLocalize(img, map, robot_plane, map_object);

	findRobot(map, robot_plane, map_object);

	state.push_back(map_object.robot.pose.x);
	state.push_back(map_object.robot.pose.y);
	state.push_back(map_object.robot.pose.theta);
    
	return true;
}
