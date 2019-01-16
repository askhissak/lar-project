#ifndef LAR_COLLISIONDETECTION_HPP
#define LAR_COLLISIONDETECTION_HPP

//Forward declared dependencies
//static int orderedArray[4] = {0};//hardcoded

//Included dependencies
//

//Classes
//

//Function declarations
float collision_detection(cv::Point p);
void line_colliding( cv::Mat img , cv::Point ini_p , cv::Point fin_p , int &line_collision , int &x_out , int &y_out );

#endif
