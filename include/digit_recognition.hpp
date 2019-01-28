#ifndef LAR_DIGITRECOGNITION_HPP
#define LAR_DIGITRECOGNITION_HPP

//Forward declared dependencies
const double MIN_AREA_SIZE = 300;

//Included dependencies
//

//Classes
//

//Function declarations
cv::Mat rotate(cv::Mat src, double angle);
// void printAngle(cv::RotatedRect calculatedRect);
bool useTesseract(cv::Mat const & map, std::vector<cv::Vec3f> circles, int* index);
bool useTemplateMatching(cv::Mat const & map, std::vector<cv::Vec3f> circles, int* index);
bool recognizeDigits(cv::Mat const & map, std::vector<cv::Vec3f> circles, std::vector<cv::Point> &orderedROI);

#endif