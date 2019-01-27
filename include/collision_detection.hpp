#ifndef LAR_COLLISIONDETECTION_HPP
#define LAR_COLLISIONDETECTION_HPP

//Function declarations
cv::Mat print_desired_contour(cv::Mat inputmap, int &contour_idx , cv::Point colliding_point , double &desired_contour_area);
cv::Mat expander(cv::Mat input);
bool collision_detection(cv::Mat inputmap , cv::Point target , int thres , int remapper , int &collision_status);
cv::Mat create_new_path( cv::Mat original_dubins , cv::Mat new_desired_contour  , cv::Point ini_collision , cv::Point fin_collision  );
int distance(cv::Point P1 , cv::Point P2);

#endif
