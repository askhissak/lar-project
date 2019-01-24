#ifndef LAR_DIGITRECOGNITION_HPP
#define LAR_DIGITRECOGNITION_HPP

//Forward declared dependencies
static int orderedArray[4] = {0};//hardcoded

//Included dependencies
//

//Classes
//

//Function declarations
int* recognizeDigits(cv::Mat const & img, std::vector<cv::Vec3f> circles);

#endif