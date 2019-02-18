#ifndef LAR_MAPEXTRACTION_HPP
#define LAR_MAPEXTRACTION_HPP

//Forward declared dependencies
const double MIN_MAP_AREA_SIZE = 100000;
const int MAP_LENGTH = 1510; //Check dimensions!
const int MAP_WIDTH = 1050;  //Check dimensions!
// std::string outputFile = "data/output/corrected.jpg";

//Included dependencies
#include "map_construction.hpp"

//Classes
//

//Function declarations
void loadCoefficients(const std::string& filename,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs);
void showImage(const std::string& window_name, cv::Mat const & img);                      
bool compareX (cv::Point pt1, cv::Point pt2);
cv::Mat assignCorners(std::vector<cv::Point> sort_x);
cv::Mat calculateTransform(cv::Mat calib_image, int length,int width,double& pixel_scale);

std::vector<cv::Point>  detectMapCorners(const cv::Mat& img);
std::vector<cv::Point>  detectRobotPlaneCorners(const cv::Mat& img);

cv::Mat findTransform(cv::Mat const & calib_image,
                  double& pixel_scale, Map & map_object);
void storeAllParameters(const std::string& filename,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        double pixel_scale,
                        const cv::Mat& persp_transf);

bool extractMapLocalize(cv::Mat const & img, cv::Mat &map, cv::Mat &robot_plane, Map & map_object);
bool extractMap(cv::Mat const & img, cv::Mat &map, cv::Mat &robot_plane, Map & map_object);

void Calibrate_HSV(cv::Mat original_img , cv::Mat hsv_img );


#endif
