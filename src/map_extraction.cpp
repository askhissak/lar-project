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
#include "map_construction.hpp"

cv::Mat result, rb_plane;

bool ME_developer_session = true ; // if true  -> Retrieves desired debugging and log content and show the Color Calibrating Trackbars
								 // if false -> Process everything without graphical outputs 


int filtering_mode = 2 ;    // if 1 -> Filtering Mode set to Adaptive Threshold
							// if 2 -> Filtering Mode set to Black Mask Filtering 

bool undistort_flag = false;              

const int max_value_H = 360/2;
const int max_value = 255;
int low_h = 0, low_s = 0, low_v = 0;
int track_low_h = 0, track_low_s = 0, track_low_v = 0;
int high_h = max_value_H, high_s = max_value, high_v = max_value;
int track_high_h = max_value_H, track_high_s = max_value, track_high_v = max_value;

cv::String source_image = "Original Image";
cv::String masked_image = "Filtered Image";

int Black_h_Low  = 0; 	int Black_h_High = 180;

int Black_s_Low  = 5; 	int Black_s_High = 255;

int Black_v_Low  = 5; 	int Black_v_High = 40;


static void on_low_h_thresh_trackbar(int, void *)
{
    low_h = cv::min(high_h-1, low_h);
    setTrackbarPos("Low H", masked_image, low_h);
}
static void on_high_h_thresh_trackbar(int, void *)
{
    high_h = cv::max(high_h, low_h+1);
    setTrackbarPos("High H", masked_image, high_h);
}
static void on_low_s_thresh_trackbar(int, void *)
{
    low_s = cv::min(high_s-1, low_s);
    setTrackbarPos("Low S", masked_image, low_s);
}
static void on_high_s_thresh_trackbar(int, void *)
{
    high_s = cv::max(high_s, low_s+1);
    setTrackbarPos("High S", masked_image, high_s);
}
static void on_low_v_thresh_trackbar(int, void *)
{
    low_v = cv::min(high_v-1, low_v);
    setTrackbarPos("Low V",masked_image, low_v);
}
static void on_high_v_thresh_trackbar(int, void *)
{
    high_v = cv::max(high_v, low_v+1);
    setTrackbarPos("High V", masked_image, high_v);
}

void Calibrate_HSV(cv::Mat original_img , cv::Mat hsv_img )
{
    
    cv::resize(original_img, original_img , cv::Size(640, 430));
    cv::resize(hsv_img, hsv_img , cv::Size(640, 430));
    
    std::cout <<"\n Adjust the Values and press 'q' to set the Values and finish the Calibration \n " << std::endl ;
    
    low_h = 0, low_s = 0, low_v = 0;
	high_h = max_value_H, high_s = max_value, high_v = max_value;
	
		
	cv::namedWindow(source_image);
	cv::namedWindow(masked_image);
	// Trackbars to set thresholds for HSV values

	
	cv::Mat hsv_filtered;
	
	bool first_values = true ;
	
    while(1)
    {
		if(first_values == true)
		{	
			low_h = Black_h_Low ;
			low_s = Black_s_Low ;
			low_v = Black_v_Low ;
	
			high_h = Black_h_High;
			high_s = Black_s_High;
			high_v = Black_v_High;
			
			cv::namedWindow(source_image, CV_WINDOW_NORMAL);
			cv::namedWindow(masked_image, CV_WINDOW_NORMAL);
			
			cv::moveWindow(masked_image, 700,0);
			
			createTrackbar("Low H", masked_image, &low_h, max_value_H, on_low_h_thresh_trackbar);
			createTrackbar("Low S", masked_image, &low_s, max_value, on_low_s_thresh_trackbar);
			createTrackbar("Low V", masked_image, &low_v, max_value, on_low_v_thresh_trackbar);
	
			createTrackbar("High H", masked_image, &high_h, max_value_H, on_high_h_thresh_trackbar);
			createTrackbar("High S", masked_image, &high_s, max_value, on_high_s_thresh_trackbar);
			createTrackbar("High V", masked_image, &high_v, max_value, on_high_v_thresh_trackbar);
				
			first_values = false ;
		}
		
		inRange(hsv_img, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v), hsv_filtered);

		
		cv::imshow(source_image, original_img);
				
		cv::imshow(masked_image, hsv_filtered);
		
		
		char key = (char) cv::waitKey(30);
		
		if (key == 'q' || key == 27)
		{
			Black_h_Low  = low_h ;  Black_h_High = high_h;
	
			Black_s_Low  = low_s ;  Black_s_High = high_s;
	
			Black_v_Low  = low_v ;  Black_v_High = high_v;
			
			cv::destroyWindow(source_image);
			cv::destroyWindow(masked_image);
			
			std::cout <<"\n Values of Filters for the Robot : " << " H_LOW: " << Black_h_Low <<  " H_HIGH : " << Black_h_High << std::endl ;
			std::cout <<"                                 " << " S_LOW: " << Black_s_Low <<  " S_HIGH : " << Black_s_High << std::endl ;
			std::cout <<"                                 " << " V_LOW: " << Black_v_Low <<  " V_HIGH : " << Black_v_High << std::endl ;
			
			break;
				
		}
	}
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

//Show an image in proper window
void showImage(const std::string& window_name, cv::Mat const & img)
{
    std::string name = window_name;
    cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
    cv::resizeWindow(name.c_str(), 640, 512);
    cv::imshow(name.c_str(), img);
    cv::waitKey(0);
    cv::destroyWindow(name.c_str());
}

//Compare x coordinate of 2 points
bool compareX (cv::Point pt1, cv::Point pt2)
{
    return (pt1.x < pt2.x); 
}

cv::Mat assignCorners(std::vector<cv::Point> sort_x)
{
    if(sort_x[0].y<sort_x[1].y)
    {
        if(sort_x[2].y<sort_x[3].y)
        {
            result.at<float>(0, 0) = sort_x[0].x;
            result.at<float>(0, 1) = sort_x[0].y;
            result.at<float>(1, 0) = sort_x[2].x;
            result.at<float>(1, 1) = sort_x[2].y; 
            result.at<float>(2, 0) = sort_x[3].x;
            result.at<float>(2, 1) = sort_x[3].y;                
            result.at<float>(3, 0) = sort_x[1].x;
            result.at<float>(3, 1) = sort_x[1].y; //+
        }
        else
        {
            result.at<float>(0, 0) = sort_x[0].x;
            result.at<float>(0, 1) = sort_x[0].y;
            result.at<float>(1, 0) = sort_x[3].x;
            result.at<float>(1, 1) = sort_x[3].y; 
            result.at<float>(2, 0) = sort_x[2].x;
            result.at<float>(2, 1) = sort_x[2].y;                
            result.at<float>(3, 0) = sort_x[1].x;
            result.at<float>(3, 1) = sort_x[1].y; //+   
        }
    }    
    else
    {
        if(sort_x[2].y<sort_x[3].y)
        {
            result.at<float>(0, 0) = sort_x[1].x;
            result.at<float>(0, 1) = sort_x[1].y;
            result.at<float>(1, 0) = sort_x[2].x;
            result.at<float>(1, 1) = sort_x[2].y; 
            result.at<float>(2, 0) = sort_x[3].x;
            result.at<float>(2, 1) = sort_x[3].y;                
            result.at<float>(3, 0) = sort_x[0].x;
            result.at<float>(3, 1) = sort_x[0].y; //+      
        }
        else
        {
            result.at<float>(0, 0) = sort_x[1].x;
            result.at<float>(0, 1) = sort_x[1].y;
            result.at<float>(1, 0) = sort_x[3].x;
            result.at<float>(1, 1) = sort_x[3].y; 
            result.at<float>(2, 0) = sort_x[2].x;
            result.at<float>(2, 1) = sort_x[2].y;                
            result.at<float>(3, 0) = sort_x[0].x;
            result.at<float>(3, 1) = sort_x[0].y; //+   
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
    // std::cout<<"Delta x: "<<delta_x<<std::endl;
    // std::cout<<"Delta y: "<<delta_y<<std::endl;

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
    
    
    cv::Mat filtered_mask;
    cv::Mat gray_img;

    // ----- Variable for the First Filtering mode  -------------
	cv::Mat bg_img = img.clone();

	// ----- Variables for the Second Filtering mode ------------ 
	cv::Mat hsv_img;
    
    
	if (filtering_mode == 1) // <- Adaptive Threshold
	{
		//Converting the Image to Grayscale to work on the filtering
		cv::cvtColor(bg_img, gray_img, cv::COLOR_BGR2GRAY);
		
		// Find image filtered by adaptive threshold 	
		
		cv::adaptiveThreshold(gray_img, filtered_mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31, 0);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
		cv::dilate(filtered_mask, filtered_mask, kernel);
		
		//Show filtered image
		if(ME_developer_session == true) showImage("Adaptive mask", filtered_mask);
		
	}
	
	else if( filtering_mode == 2)
	{
		//Converting the Image to HSV Color Space to work with the respective color filtered masks
		cv::cvtColor(bg_img, hsv_img, cv::COLOR_BGR2HSV);
		
		
		if (ME_developer_session == true) Calibrate_HSV(bg_img , hsv_img);
		
		
		// Find black regions (filter on saturation and value)
		cv::inRange(hsv_img, cv::Scalar(Black_h_Low, Black_s_Low, Black_v_Low), cv::Scalar(Black_h_High, Black_s_High, Black_v_High), filtered_mask);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
		cv::dilate(filtered_mask, filtered_mask, kernel);	
		cv::erode(filtered_mask, filtered_mask, kernel);
		
	} 
	
    // Find contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve, sort_x;
    cv::Mat contours_img;

    // Process black mask
    cv::findContours(filtered_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob

    contours_img = img.clone();
    drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    for (int i=0; i<contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_MAP_AREA_SIZE) continue; // filter too small contours to remove false positives
        approxPolyDP(contours[i], approx_curve, 80, true); // fit a closed polygon (with less vertices) to the given contour
        
        if(approx_curve.size()!=4) continue;

        sort_x = approx_curve;

        std::sort(sort_x.begin(), sort_x.end(), compareX);

        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);

        if(ME_developer_session == true) showImage("Map Boundary", contours_img);

    }

    return sort_x;
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

    if(ME_developer_session == true) showImage("White mask", white_mask);   
    
    std::vector<cv::Point> sort_high_x;
    std::vector<cv::Vec3f> corner_circles;
    HoughCircles( white_mask , corner_circles, cv::HOUGH_GRADIENT, 1, white_mask.rows/10, 40, 20, 15, 27);

    if(corner_circles.size()==4)
    {
        cv::Mat circles_img = img.clone();

        for( int i = 0; i < corner_circles.size(); ++i )
        {
            
            cv::Point center(cvRound(corner_circles[i][0]), cvRound(corner_circles[i][1]));
            // std::cout<<"Circle Center point: "<<center<<" and radius "<<cornerCircles[i][2]<<std::endl;
                    
            // circle center
            circle( circles_img, center, corner_circles[i][2], cv::Scalar(0, 250 ,255), -1, 8, 0 );        

            // showImage("Circles Detected", circles_img);

            sort_high_x.push_back(center);

            if(i==corner_circles.size()-1)
            {
                cv::line(circles_img, cv::Point(corner_circles[i][0],corner_circles[i][1]) , cv::Point(corner_circles[0][0],corner_circles[0][1]) , cv::Scalar( 0 , 250 , 255 ), 5, 8, 0);
            }else
            {
                cv::line(circles_img, cv::Point(corner_circles[i][0],corner_circles[i][1]) , cv::Point(corner_circles[i+1][0],corner_circles[i+1][1]) , cv::Scalar( 0 , 250 , 255 ), 5, 8, 0);
            }
            
        }

        if(ME_developer_session == true) showImage("Higher plane", circles_img);

    }
    else
    {
        std::cout << "Detected some noise! Leaving..." << std::endl;
        return sort_high_x;
    }

    std::sort(sort_high_x.begin(), sort_high_x.end(), compareX);

    return sort_high_x;
}

//Calculate and return perspective transformation matrix
cv::Mat findTransform(cv::Mat const & calib_image,
                  double& pixel_scale, Map & map_object)
{
   
    if(ME_developer_session == true) showImage("Undistorted image", calib_image);


    std::vector<cv::Point> sort_high_x = detectRobotPlaneCorners(calib_image);
    cv::Mat high_plane_corner_pixels = assignCorners(sort_high_x);

    std::vector<cv::Point> sort_x = detectMapCorners(calib_image);
    cv::Mat corner_pixels = assignCorners(sort_x);

    corner_pixels.at<float>(0, 0) = corner_pixels.at<float>(0, 0)-15;
    corner_pixels.at<float>(0, 1) = corner_pixels.at<float>(0, 1)-15;
    corner_pixels.at<float>(1, 0) = corner_pixels.at<float>(1, 0)+15;
    corner_pixels.at<float>(1, 1) = corner_pixels.at<float>(1, 1)-15; 
    corner_pixels.at<float>(2, 0) = corner_pixels.at<float>(2, 0)+15;
    corner_pixels.at<float>(2, 1) = corner_pixels.at<float>(2, 1)+15;                
    corner_pixels.at<float>(3, 0) = corner_pixels.at<float>(3, 0)-15;
    corner_pixels.at<float>(3, 1) = corner_pixels.at<float>(3, 1)+15;

    map_object.corners = corner_pixels;
    map_object.robot_corners = high_plane_corner_pixels;
    map_object.toMap = cv::findHomography(high_plane_corner_pixels,corner_pixels);

    cv::Mat transf_rbplane_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);
    cv::Mat transf_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);

    cv::Mat transf_rb = cv::getPerspectiveTransform(high_plane_corner_pixels, transf_rbplane_pixels);
    cv::Mat unwarped_rb_frame;    
    
    cv::Mat transf = cv::getPerspectiveTransform(corner_pixels, transf_pixels);
    cv::Mat unwarped_frame;

    cv::warpPerspective(calib_image, unwarped_rb_frame, transf_rb, cv::Size(MAP_LENGTH,MAP_WIDTH));
    cv::warpPerspective(calib_image, unwarped_frame, transf, cv::Size(MAP_LENGTH,MAP_WIDTH));

    std::string name, wind2; 
	
	if(ME_developer_session == true) 
    {
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
	}
    rb_plane = unwarped_rb_frame;

    return unwarped_frame;
}

bool extractMapLocalize(cv::Mat const & img, cv::Mat &map, cv::Mat &robot_plane, Map & map_object)
{
    cv::Mat calib_image;
    cv::Mat camera_matrix, dist_coeffs, persp_transf;
    if(undistort_flag)
    {
        loadCoefficients("config/intrinsic_calibration.xml", camera_matrix, dist_coeffs);

        calib_image = img;
        undistort(img, calib_image, camera_matrix, dist_coeffs);
    }
    else
    {
        calib_image = img;
    }

    if(ME_developer_session == true) showImage("Undistorted image", calib_image);

    double pixel_scale;
    cv::Mat transf_rbplane_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);
    cv::Mat transf_pixels = calculateTransform(calib_image, MAP_LENGTH,MAP_WIDTH,pixel_scale);

    cv::Mat transf_rb = cv::getPerspectiveTransform(map_object.robot_corners, transf_rbplane_pixels);
    cv::Mat unwarped_rb_frame;    
    
    cv::Mat transf = cv::getPerspectiveTransform(map_object.corners, transf_pixels);
    cv::Mat unwarped_frame;

    cv::warpPerspective(calib_image, unwarped_rb_frame, transf_rb, cv::Size(MAP_LENGTH,MAP_WIDTH));
    cv::warpPerspective(calib_image, unwarped_frame, transf, cv::Size(MAP_LENGTH,MAP_WIDTH));

    std::string name, wind2; 
	
	if(ME_developer_session == true)
	{
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

	}
    robot_plane = unwarped_rb_frame;
    map = unwarped_frame;

    if(undistort_flag)
    {
        storeAllParameters("config/fullCalibration.yml", camera_matrix, dist_coeffs, pixel_scale, persp_transf);
    }

    if(map.empty())
    {
        std::cout<<"Map is empty!"<<std::endl;
        return false;
    }
    
    return true;
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
bool extractMap(cv::Mat const & img, cv::Mat &map, cv::Mat &robot_plane, Map & map_object) 
{
    cv::Mat calib_image;
    cv::Mat camera_matrix, dist_coeffs, persp_transf;
    if(undistort_flag)
    {
        loadCoefficients("config/intrinsic_calibration.xml", camera_matrix, dist_coeffs);

        calib_image = img;
        undistort(img, calib_image, camera_matrix, dist_coeffs);
    }
    else
    {
        calib_image = img;
    }

    double pixel_scale;
    map = findTransform(calib_image, pixel_scale, map_object);

    if(undistort_flag)
    {
        storeAllParameters("config/fullCalibration.yml", camera_matrix, dist_coeffs, pixel_scale, persp_transf);        
    }

    robot_plane = rb_plane;

    if(map.empty())
    {
        std::cout<<"Map is empty!"<<std::endl;
        return false;
    }
    
    return true;
}

