// map_extraction.cpp:
#include <string>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "map_extraction.hpp"

cv::Mat result;

//IMAGE UNWARPING
bool compareX (cv::Point pt1, cv::Point pt2)
{
    return (pt1.x < pt2.x); 
}

//Load camera matrix and distortion coefficients values from intrinsic_calibration.xml
void loadCoefficients(const std::string& filename,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs)
{
    cv::FileStorage fs( filename, cv::FileStorage::READ );
    if (!fs.isOpened())
    {
        throw std::runtime_error("Could not open file " + filename);
    }
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();
}

//Picking reference points automatically
cv::Mat detectMapCorners(const cv::Mat& img)
{
    result = cv::Mat(4, 2, CV_32F);
    cv::Mat bg_img = img.clone();

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(bg_img, hsv_img, cv::COLOR_BGR2HSV);

    // Find black regions (filter on saturation and value)
    cv::Mat black_mask;
    cv::inRange(hsv_img, cv::Scalar(0, 5, 5), cv::Scalar(180, 255, 40), black_mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(black_mask, black_mask, kernel);	
    cv::erode(black_mask, black_mask, kernel);

    //Show detected black regions
    std::string name = "Boundary";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 512, 640);
    cv::imshow(name.c_str(), black_mask);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());

    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve, sortX;
    cv::Mat contours_img;

    // Process black mask
    cv::findContours(black_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
    contours_img = img.clone();
    drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    for (int i=0; i<contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_MAP_AREA_SIZE) continue; // filter too small contours to remove false positives
        approxPolyDP(contours[i], approx_curve, 80, true); // fit a closed polygon (with less vertices) to the given contour
        
        if(approx_curve.size()!=4) continue;

        sortX = approx_curve;

        std::sort(sortX.begin(), sortX.end(), compareX);

        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
        name = "Main Perimeter (Black Boundary)";
        cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
        cv::resizeWindow(name.c_str(), 512, 640);
        cv::imshow(name.c_str(), contours_img);
        cv::waitKey(0);
        cv::destroyWindow(name.c_str());

    }

    //Calculate euclidian distance between points to determine rectangle's layout
    double euclidianDist1 = 0, euclidianDist2 = 0, euclidianDist3 = 0, euclidianDist4 = 0, euclidianDist5 = 0;
    euclidianDist1 = sqrt(pow((sortX[0].x-sortX[1].x),2)+pow((sortX[0].y-sortX[1].y),2));
    euclidianDist2 = sqrt(pow((sortX[0].x-sortX[2].x),2)+pow((sortX[0].y-sortX[2].y),2));
    euclidianDist3 = sqrt(pow((sortX[0].x-sortX[3].x),2)+pow((sortX[0].y-sortX[3].y),2));
    euclidianDist4 = sqrt(pow((sortX[1].x-sortX[2].x),2)+pow((sortX[1].y-sortX[2].y),2));
    euclidianDist5 = sqrt(pow((sortX[1].x-sortX[3].x),2)+pow((sortX[1].y-sortX[3].y),2));

    if(sortX[0].y<sortX[1].y)
    {
        if(euclidianDist2<euclidianDist3)
        {
            if(euclidianDist1<euclidianDist2)
            {
                result.at<float>(0, 0) = sortX[0].x;
                result.at<float>(0, 1) = sortX[0].y;
                result.at<float>(1, 0) = sortX[2].x;
                result.at<float>(1, 1) = sortX[2].y; 
                result.at<float>(2, 0) = sortX[3].x;
                result.at<float>(2, 1) = sortX[3].y;                
                result.at<float>(3, 0) = sortX[1].x;
                result.at<float>(3, 1) = sortX[1].y; 
            }
            else
            {
                result.at<float>(0, 0) = sortX[2].x;
                result.at<float>(0, 1) = sortX[2].y;
                result.at<float>(1, 0) = sortX[3].x;
                result.at<float>(1, 1) = sortX[3].y;                  
                result.at<float>(2, 0) = sortX[1].x;
                result.at<float>(2, 1) = sortX[1].y;                
                result.at<float>(3, 0) = sortX[0].x;
                result.at<float>(3, 1) = sortX[0].y; 
            }       
        }
        else
        {
            if(euclidianDist1<euclidianDist3)
            {
                result.at<float>(0, 0) = sortX[0].x;
                result.at<float>(0, 1) = sortX[0].y;
                result.at<float>(1, 0) = sortX[3].x;
                result.at<float>(1, 1) = sortX[3].y; 
                result.at<float>(2, 0) = sortX[2].x;
                result.at<float>(2, 1) = sortX[2].y;                
                result.at<float>(3, 0) = sortX[1].x;
                result.at<float>(3, 1) = sortX[1].y; 
            }
            else
            {
                result.at<float>(0, 0) = sortX[3].x;
                result.at<float>(0, 1) = sortX[3].y;
                result.at<float>(1, 0) = sortX[2].x;
                result.at<float>(1, 1) = sortX[2].y;                  
                result.at<float>(2, 0) = sortX[1].x;
                result.at<float>(2, 1) = sortX[1].y;                
                result.at<float>(3, 0) = sortX[0].x;
                result.at<float>(3, 1) = sortX[0].y; 
            }       
        }
    }    
    else
    {
        if(euclidianDist4<euclidianDist5)
        {
            if(euclidianDist1<euclidianDist4)
            {
                result.at<float>(0, 0) = sortX[1].x;
                result.at<float>(0, 1) = sortX[1].y;
                result.at<float>(1, 0) = sortX[2].x;
                result.at<float>(1, 1) = sortX[2].y; 
                result.at<float>(2, 0) = sortX[3].x;
                result.at<float>(2, 1) = sortX[3].y;                
                result.at<float>(3, 0) = sortX[0].x;
                result.at<float>(3, 1) = sortX[0].y; 
            }
            else
            {
                result.at<float>(0, 0) = sortX[2].x;
                result.at<float>(0, 1) = sortX[2].y;
                result.at<float>(1, 0) = sortX[3].x;
                result.at<float>(1, 1) = sortX[3].y;                  
                result.at<float>(2, 0) = sortX[0].x;
                result.at<float>(2, 1) = sortX[0].y;                
                result.at<float>(3, 0) = sortX[1].x;
                result.at<float>(3, 1) = sortX[1].y; 
            }       
        }
        else
        {
            if(euclidianDist1<euclidianDist5)
            {
                result.at<float>(0, 0) = sortX[1].x;
                result.at<float>(0, 1) = sortX[1].y;
                result.at<float>(1, 0) = sortX[3].x;
                result.at<float>(1, 1) = sortX[3].y; 
                result.at<float>(2, 0) = sortX[2].x;
                result.at<float>(2, 1) = sortX[2].y;                
                result.at<float>(3, 0) = sortX[0].x;
                result.at<float>(3, 1) = sortX[0].y; 
            }
            else
            {
                result.at<float>(0, 0) = sortX[3].x;
                result.at<float>(0, 1) = sortX[3].y;
                result.at<float>(1, 0) = sortX[2].x;
                result.at<float>(1, 1) = sortX[2].y;                  
                result.at<float>(2, 0) = sortX[0].x;
                result.at<float>(2, 1) = sortX[0].y;                
                result.at<float>(3, 0) = sortX[1].x;
                result.at<float>(3, 1) = sortX[1].y; 
            }       
        }
    }

    return result;
}

//Return destination matrix dimensions
cv::Mat pickOrigin()
{
    result = cv::Mat(2, 2, CV_32F);

    result.at<float>(0, 0) = 0;
    result.at<float>(0, 1) = 0;
    result.at<float>(1, 0) = 1470;
    result.at<float>(1, 1) = 970;
    
    return result;
}

//Properly rotate an image
cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point2f pt(src.cols/2., src.rows/2.);  
 
    cv::Mat r = cv::getRotationMatrix2D(pt, -angle, 1.0);
    
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();

    r.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    r.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;
       
    warpAffine(src, dst, r, bbox.size());
    return dst;
}

//Calculate and return perspective transformation matrix
cv::Mat findTransform(const std::string& calib_image_name,
                  const cv::Mat& camera_matrix,
                  const cv::Mat& dist_coeffs,
                  double& pixel_scale)
{
    cv::Mat calib_image, original_image = cv::imread(calib_image_name);

    if (original_image.empty())
    {
        throw std::runtime_error("Could not open image " + calib_image_name);
    }

    undistort(original_image, calib_image, camera_matrix, dist_coeffs);

    std::string name = "Undistorted image";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 512, 640);
    cv::imshow(name.c_str(), calib_image);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());

    cv::Mat corner_pixels = detectMapCorners(calib_image);

    cv::Mat screen_pixels = cv::Mat(calib_image.rows, calib_image.cols, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat dst_pixels = pickOrigin();

    float origin_x = dst_pixels.at<float>(0, 0),
            origin_y = dst_pixels.at<float>(0, 1);

    float delta_x = dst_pixels.at<float>(1, 0)-dst_pixels.at<float>(0, 0);
    float delta_y = dst_pixels.at<float>(1, 1)-dst_pixels.at<float>(0, 1);

    float delta_x_mm = 1470;
    float delta_y_mm = 970;

    float scale_x = delta_x/delta_x_mm;
    float scale_y = delta_y/delta_y_mm;
    float scale = std::min(scale_x, scale_y);

    pixel_scale = 1./scale;
    std::cout<<"Delta x: "<<delta_x<<std::endl;
    std::cout<<"Delta y: "<<delta_y<<std::endl;

    cv::Mat transf_pixels = (cv::Mat_<float>(4,2) << origin_x, origin_y,
                                                    origin_x+delta_x, origin_y,
                                                    origin_x+delta_x, origin_y+delta_y,
                                                    origin_x, origin_y+delta_y);

    cv::Mat transf = cv::getPerspectiveTransform(corner_pixels, transf_pixels);
    cv::Mat unwarped_frame, rotated_frame;

    cv::warpPerspective(calib_image, unwarped_frame, transf, cv::Size(delta_x,delta_y));

    cv::Mat hsv_img;
    cv::cvtColor(unwarped_frame, hsv_img, cv::COLOR_BGR2HSV);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> approx_curve;
    cv::Mat blue_mask;
    cv::inRange(hsv_img, cv::Scalar(90, 55, 55), cv::Scalar(130, 255, 200), blue_mask);
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (int i=0; i<contours.size(); ++i)
    {
        if (cv::contourArea(contours[i])>100)
        {        
            approxPolyDP(contours[i], approx_curve, 10, true);
        }
    }

    if(std::abs(result.at<float>(0, 1)-approx_curve[0].y)<500)
    {
        rotated_frame = rotate(unwarped_frame, 270);
    }
    else
    {
        rotated_frame = rotate(unwarped_frame, 90);
    }

    cv::imwrite("data/output/corrected.jpg", rotated_frame);

    std::string wind2; 

    wind2 = "Unwarping";
    cv::namedWindow(wind2.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(wind2.c_str(), 512, 640);
    imshow(wind2.c_str(), rotated_frame);
    cv::waitKey(0);
    cv::destroyWindow(wind2);

    return transf;
}

//Store all parameters
void storeAllParameters(const std::string& filename,
                        const cv::Mat& camera_matrix,
                        const cv::Mat& dist_coeffs,
                        double pixel_scale,
                        const cv::Mat& persp_transf)
{
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );
    fs << "camera_matrix" << camera_matrix
        << "dist_coeffs" << dist_coeffs
        << "pixel_scale" << pixel_scale
        << "persp_transf" << persp_transf;
    fs.release();
}

//Unwarp the image using saved distortion coefficients and store all parameters (plus perspective transformation matrix)
std::string extractMap(const std::string filename) 
{
    cv::Mat camera_matrix, dist_coeffs, persp_transf;
    loadCoefficients("config/intrinsic_calibration.xml", camera_matrix, dist_coeffs);

    double pixel_scale;
    persp_transf = findTransform(filename, camera_matrix, dist_coeffs, pixel_scale);
    std::cout << "Pixel Scale: " << pixel_scale << "mm" << std::endl;

    storeAllParameters("config/fullCalibration.yml", camera_matrix, dist_coeffs, pixel_scale, persp_transf);
    
    return "data/output/corrected.jpg";
}