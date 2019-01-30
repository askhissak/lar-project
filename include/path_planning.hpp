#ifndef LAR_PATHPLANNING_HPP
#define LAR_PATHPLANNING_HPP

//Forward declared dependencies

//Included dependencies
#include "map_construction.hpp"
#include "path.h"

//Classes
//

//Function declarations
double timer();
bool obstacleOffset(cv::Mat input, std::vector<cv::Point> & dst);
bool planMissionOne(Map & map_object, Path & path);
bool planMissionTwo(Map & map_object, Path & path);
void detectCollisions();
bool planDubins(Map & map_object, Path & path, std::vector<int> & order);
bool planMission(cv::Mat const & map, Map & map_object, Path & path, std::vector<int> & order);

#endif