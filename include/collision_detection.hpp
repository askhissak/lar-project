#ifndef LAR_COLLISIONDETECTION_HPP
#define LAR_COLLISIONDETECTION_HPP

//Forward declared dependencies
//static int orderedArray[4] = {0};//hardcoded

//Included dependencies
//

//Classes
//

//Function declarations
cv::Mat print_contours(cv::Mat inputmap);
bool collision_detection(cv::Mat inputmap , cv::Point target);
#endif
