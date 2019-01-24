#ifndef LAR_COLLISIONDETECTION_HPP
#define LAR_COLLISIONDETECTION_HPP

//Forward declared dependencies
//static int orderedArray[4] = {0};//hardcoded

//Included dependencies
//

//Classes
//

//Function declarations
cv::Mat print_desired_contour(cv::Mat inputmap, int &contour_idx , cv::Point &contour_center , int &contour_radius , cv::Point colliding_point );
cv::Mat rounder(cv::Mat input);
bool collision_detection(cv::Mat inputmap , cv::Point target , int thres , int remapper , int &contour_index);
cv::Mat crop_contour( cv::Mat isolated_contour , cv::Point ini_collision , cv::Point fin_collision , cv::Point contour_center , int contour_radius );
#endif
