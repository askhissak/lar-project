// map_extraction.cpp: Image processing to extract map and robot planes
#include <string>
#include <vector>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>

#include "map_extraction.hpp"

cv::Mat result, rbPlane;

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

//Compare x coordinate of 2 points
bool compareX (cv::Point pt1, cv::Point pt2)
{
    return (pt1.x < pt2.x); 
}

//Properly rotate an image
cv::Mat rotate(cv::Mat src, double angle)
{
    cv::Mat dst;
    cv::Point pt(src.cols/2., src.rows/2.);  
 
    cv::Mat r = cv::getRotationMatrix2D(pt, -angle, 1.0);
    
    cv::Rect2f bbox = cv::RotatedRect(cv::Point(), src.size(), angle).boundingRect2f();

    r.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    r.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;
       
    warpAffine(src, dst, r, bbox.size());
    return dst;
}

cv::Mat assignCorners(std::vector<cv::Point> sortX)
{
    if(sortX[0].y<sortX[1].y)
    {
        if(sortX[2].y<sortX[3].y)
        {
            result.at<float>(0, 0) = sortX[0].x;
            result.at<float>(0, 1) = sortX[0].y;
            result.at<float>(1, 0) = sortX[2].x;
            result.at<float>(1, 1) = sortX[2].y; 
            result.at<float>(2, 0) = sortX[3].x;
            result.at<float>(2, 1) = sortX[3].y;                
            result.at<float>(3, 0) = sortX[1].x;
            result.at<float>(3, 1) = sortX[1].y; //+
        }
        else
        {
            result.at<float>(0, 0) = sortX[0].x;
            result.at<float>(0, 1) = sortX[0].y;
            result.at<float>(1, 0) = sortX[3].x;
            result.at<float>(1, 1) = sortX[3].y; 
            result.at<float>(2, 0) = sortX[2].x;
            result.at<float>(2, 1) = sortX[2].y;                
            result.at<float>(3, 0) = sortX[1].x;
            result.at<float>(3, 1) = sortX[1].y; //+   
        }
    }    
    else
    {
        if(sortX[2].y<sortX[3].y)
        {
            result.at<float>(0, 0) = sortX[1].x;
            result.at<float>(0, 1) = sortX[1].y;
            result.at<float>(1, 0) = sortX[2].x;
            result.at<float>(1, 1) = sortX[2].y; 
            result.at<float>(2, 0) = sortX[3].x;
            result.at<float>(2, 1) = sortX[3].y;                
            result.at<float>(3, 0) = sortX[0].x;
            result.at<float>(3, 1) = sortX[0].y; //+      
        }
        else
        {
            result.at<float>(0, 0) = sortX[1].x;
            result.at<float>(0, 1) = sortX[1].y;
            result.at<float>(1, 0) = sortX[3].x;
            result.at<float>(1, 1) = sortX[3].y; 
            result.at<float>(2, 0) = sortX[2].x;
            result.at<float>(2, 1) = sortX[2].y;                
            result.at<float>(3, 0) = sortX[0].x;
            result.at<float>(3, 1) = sortX[0].y; //+   
        }
    }

    return result;
}

cv::Mat calculateTransform(cv::Mat calib_image, int length,int width,double& pixel_scale)
{
    cv::Mat screen_pixels = cv::Mat(calib_image.rows, calib_image.cols, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat dst_pixels = (cv::Mat_<float>(2,2) << 0,0,length,width);

    float origin_x = dst_pixels.at<float>(0, 0),
            origin_y = dst_pixels.at<float>(0, 1);

    float delta_x = dst_pixels.at<float>(1, 0)-dst_pixels.at<float>(0, 0);
    float delta_y = dst_pixels.at<float>(1, 1)-dst_pixels.at<float>(0, 1);

    float delta_x_mm = length; 
    float delta_y_mm = width; 

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

    return transf_pixels;
}

//Picking reference points for map plane automatically
std::vector<cv::Point> detectMapCorners(const cv::Mat& img)
{
    result = cv::Mat(4, 2, CV_32F);
    cv::Mat bg_img = img.clone();

    // Convert color space from BGR to grayscale
    cv::Mat gray_img;
    cv::cvtColor(bg_img, gray_img, cv::COLOR_BGR2GRAY);

    // Find image filtered by adaptive threshold 
    cv::Mat adaptive_mask;
    cv::adaptiveThreshold(gray_img, adaptive_mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(adaptive_mask, adaptive_mask, kernel);

    //Show filtered image
    std::string name = "Boundary";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 640, 512);
    cv::imshow(name.c_str(), adaptive_mask);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());

    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve, sortX;
    cv::Mat contours_img;

    // Process black mask
    cv::findContours(adaptive_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
    std::cout<<"Countours: "<<contours[0]<<std::endl;
    std::cout<<"Countours: "<<contours[1]<<std::endl;
    std::cout<<"Countours: "<<contours[2]<<std::endl;
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
        cv::resizeWindow(name.c_str(), 640, 512);
        cv::imshow(name.c_str(), contours_img);
        cv::waitKey(0);
        cv::destroyWindow(name.c_str());

    }

    return sortX;
}

//Picking reference points for robot plane automatically
std::vector<cv::Point> detectRobotPlaneCorners(const cv::Mat& img)
{
    result = cv::Mat(4, 2, CV_32F);
    cv::Mat bg_img = img.clone();
    
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(bg_img, hsv_img, cv::COLOR_BGR2HSV);

    // Find white regions
    cv::Mat white_mask;
    cv::inRange(hsv_img, cv::Scalar(60, 10, 180), cv::Scalar(160, 100, 255), white_mask);

    std::string name = "White circles";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 640, 512);
    cv::imshow(name.c_str(), white_mask);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());     
    
    std::vector<cv::Point> sortHighX;
    std::vector<cv::Vec3f> cornerCircles;
    HoughCircles( white_mask , cornerCircles, cv::HOUGH_GRADIENT, 1, white_mask.rows/10, 40, 20, 15, 27);

    if(cornerCircles.size()==4)
    {
        cv::Mat circles_img = img.clone();

        for( int i = 0; i < cornerCircles.size(); ++i )
        {
            
            cv::Point center(cvRound(cornerCircles[i][0]), cvRound(cornerCircles[i][1]));
            std::cout<<"Circle Center point: "<<center<<" and radius "<<cornerCircles[i][2]<<std::endl;
                    
            // circle center
            circle( circles_img, center, cornerCircles[i][2], cv::Scalar(0, 250 ,255), -1, 8, 0 );        

            name = "Circles Detected";
            cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
            cv::resizeWindow(name.c_str(), 640, 512);
            cv::imshow(name.c_str(), circles_img);
            cv::waitKey(0);
            cv::destroyWindow(name.c_str());   

            sortHighX.push_back(center);

            if(i==cornerCircles.size()-1)
            {
                cv::line(circles_img, cv::Point(cornerCircles[i][0],cornerCircles[i][1]) , cv::Point(cornerCircles[0][0],cornerCircles[0][1]) , cv::Scalar( 0 , 250 , 255 ), 5, 8, 0);
            }else
            {
                cv::line(circles_img, cv::Point(cornerCircles[i][0],cornerCircles[i][1]) , cv::Point(cornerCircles[i+1][0],cornerCircles[i+1][1]) , cv::Scalar( 0 , 250 , 255 ), 5, 8, 0);
            }
            
            name = "Higher plane";
            cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
            cv::resizeWindow(name.c_str(), 640, 512);
            cv::imshow(name.c_str(), circles_img);
            cv::waitKey(0);
            cv::destroyWindow(name.c_str());

        }

    }
    else
    {
        std::cout << "Detected some noise! Leaving..." << std::endl;
        return sortHighX;
    }

    std::sort(sortHighX.begin(), sortHighX.end(), compareX);

    return sortHighX;
}

//Calculate and return perspective transformation matrix
cv::Mat findTransform(cv::Mat const & calib_image,
                  double& pixel_scale)
{
    std::string name = "Undistorted image";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 640, 512);
    cv::imshow(name.c_str(), calib_image);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());


    std::vector<cv::Point> sortHighX = detectRobotPlaneCorners(calib_image);
    cv::Mat high_plane_corner_pixels = assignCorners(sortHighX);

    std::vector<cv::Point> sortX = detectMapCorners(calib_image);
    cv::Mat corner_pixels = assignCorners(sortX);

    corner_pixels.at<float>(0, 0) = corner_pixels.at<float>(0, 0)-15;
    corner_pixels.at<float>(0, 1) = corner_pixels.at<float>(0, 1)-15;
    corner_pixels.at<float>(1, 0) = corner_pixels.at<float>(1, 0)+15;
    corner_pixels.at<float>(1, 1) = corner_pixels.at<float>(1, 1)-15; 
    corner_pixels.at<float>(2, 0) = corner_pixels.at<float>(2, 0)+15;
    corner_pixels.at<float>(2, 1) = corner_pixels.at<float>(2, 1)+15;                
    corner_pixels.at<float>(3, 0) = corner_pixels.at<float>(3, 0)-15;
    corner_pixels.at<float>(3, 1) = corner_pixels.at<float>(3, 1)+15;

    // cv::Mat h = cv::findHomography(sortHighX,sortX);

    // std::cout<<"Matrix h "<<h<<std::endl;

    // cv::warpPerspective(high_plane_corner_pixels, corner_pixels, h, cv::Size(MAP_LENGTH,MAP_WIDTH));

    cv::Mat transf_rbplane_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);
    cv::Mat transf_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);

    cv::Mat transf_rb = cv::getPerspectiveTransform(high_plane_corner_pixels, transf_rbplane_pixels);
    cv::Mat unwarped_rb_frame;    
    
    cv::Mat transf = cv::getPerspectiveTransform(corner_pixels, transf_pixels);
    cv::Mat unwarped_frame;

    cv::warpPerspective(calib_image, unwarped_rb_frame, transf_rb, cv::Size(MAP_LENGTH,MAP_WIDTH));
    cv::warpPerspective(calib_image, unwarped_frame, transf, cv::Size(MAP_LENGTH,MAP_WIDTH));

    std::string wind2; 

    name = "Unwarped robot plane";
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 640, 512);
    imshow(name.c_str(), unwarped_rb_frame);

    wind2 = "Unwarped map plane";
    cv::namedWindow(wind2.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(wind2.c_str(), 640, 512);
    imshow(wind2.c_str(), unwarped_frame);

    cv::waitKey(0);
    cv::destroyWindow(name);
    cv::destroyWindow(wind2);

    rbPlane = unwarped_rb_frame;

    return unwarped_frame;
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

//Process the image using perspective transformation matrix
cv::Mat extractMap(cv::Mat const & img, cv::Mat &robotPlane) 
{
    cv::Mat camera_matrix, dist_coeffs, persp_transf, calib_image;
    loadCoefficients("config/intrinsic_calibration.xml", camera_matrix, dist_coeffs);

    undistort(img, calib_image, camera_matrix, dist_coeffs);

    double pixel_scale;
    persp_transf = findTransform(calib_image, pixel_scale);

    storeAllParameters("config/fullCalibration.yml", camera_matrix, dist_coeffs, pixel_scale, persp_transf);

    robotPlane = rbPlane;
    
    return persp_transf;
}