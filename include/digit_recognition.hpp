#ifndef LAR_DIGITRECOGNITION_HPP
#define LAR_DIGITRECOGNITION_HPP

//Forward declared dependencies
const double MIN_AREA_SIZE = 300;

//Included dependencies
#include "map_construction.hpp"

//Classes
//

//Function declarations
cv::Mat rotate(cv::Mat src, double angle);
// void printAngle(cv::RotatedRect calculatedRect);
bool useTesseract(cv::Mat const & map, std::vector<Victim> victims, int* index);
bool useTemplateMatching(cv::Mat const & map, std::vector<Victim> victims, int* index);
bool recognizeDigits(cv::Mat const & map, std::vector<Victim> victims, std::vector<int> & order);

#endif