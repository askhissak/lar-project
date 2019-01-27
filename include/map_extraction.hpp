#ifndef LAR_MAPEXTRACTION_HPP
#define LAR_MAPEXTRACTION_HPP

//Forward declared dependencies
const double MIN_MAP_AREA_SIZE = 100000;
const int MAP_LENGTH = 1510; //Check dimensions!
const int MAP_WIDTH = 1050;  //Check dimensions!
// std::string outputFile = "data/output/corrected.jpg";

//Included dependencies
//

//Classes
//

//Function declarations
void loadCoefficients(const std::string& filename,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs);
void showImage(const std::string& windowName, cv::Mat const & img);                      
bool compareX (cv::Point pt1, cv::Point pt2);
cv::Mat assignCorners(std::vector<cv::Point> sortX);
cv::Mat calculateTransform(cv::Mat calib_image, int length,int width,double& pixel_scale);

std::vector<cv::Point>  detectMapCorners(const cv::Mat& img);
std::vector<cv::Point>  detectRobotPlaneCorners(const cv::Mat& img);

cv::Mat findTransform(cv::Mat const & calib_image,
                  double& pixel_scale);
void storeAllParameters(const std::string& filename,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        double pixel_scale,
                        const cv::Mat& persp_transf);

bool extractMap(cv::Mat const & img, cv::Mat &map, cv::Mat &robotPlane);

#endif