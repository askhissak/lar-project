#ifndef LAR_COLLISIONDETECTION_HPP
#define LAR_COLLISIONDETECTION_HPP

//Function declarations


bool run_collision_detection( cv::Mat map , std::vector<cv::Point> pointPath , std::vector<cv::Point> &new_path , std::vector<std::vector<cv::Point>> &inflated_contours );

bool first_collision_detection ( cv::Mat original_img ,cv::Mat &original_map , std::vector<cv::Point> pointPath , int thres , int &collisions_number , std::vector<cv::Point> &ini_points , std::vector<cv::Point> &fin_points , int &contours_number , int &contour_idx , std::vector<cv::Point> &desired_contour_vector , std::vector<int> &colliding_contours_idx , std::vector<std::vector<cv::Point>> &original_contours );


cv::Mat inflate_contours( cv::Mat original_map , std::vector<std::vector<cv::Point>> original_contours , std::vector<std::vector<cv::Point>> &inflated_contours , std::vector<int> &colliding_contours_idx  ,  std::vector<cv::Point> pointPath , int thres , std::vector<cv::Point> &ini_points , std::vector<cv::Point> &fin_points );


bool collision_detection( std::vector<cv::Point> new_path , std::vector<std::vector<cv::Point>> contours , int thres , cv::Mat original_map );


cv::Mat create_new_path( std::vector<cv::Point> pointPath , std::vector<int> &colliding_contours_idx , std::vector<std::vector<cv::Point>> &inflated_contours , cv::Mat map , int thres, std::vector<cv::Point> &ini_points, std::vector<cv::Point> &fin_points, std::vector<cv::Point> &new_path );


int distance(cv::Point P1 , cv::Point P2);

#endif
