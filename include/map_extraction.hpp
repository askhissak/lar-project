#ifndef LAR_MAPEXTRACTION_HPP
#define LAR_MAPEXTRACTION_HPP

//Forward declared dependencies
const double MIN_MAP_AREA_SIZE = 100000;
// std::string outputFile = "data/output/corrected.jpg";

//Included dependencies
//

//Classes
//

//Function declarations
bool compareX (cv::Point pt1, cv::Point pt2);
void loadCoefficients(const std::string& filename,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs);
cv::Mat detectMapCorners(const cv::Mat& img);
cv::Mat pickOrigin();
cv::Mat rotate(cv::Mat src, double angle);
cv::Mat findTransform(const std::string& calib_image_name,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs,
                  double& pixel_scale);
void storeAllParameters(const std::string& filename,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        double pixel_scale,
                        const cv::Mat& persp_transf);
std::string extractMap(const std::string filename);

#endif