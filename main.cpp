//main.cpp
#include <iostream>
#include <opencv2/opencv.hpp>

#include "robot_project.h"


int main(int argc, char* argv[])
{
    //MENU
    // std::cout << "Please choose an option: " << std::endl;
    
    if (argc != 2) 
    {
        std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
        return 0;
    }

    std::string filename = argv[1];

    cv::Mat img = cv::imread(filename);

    if (filename.empty())
    {
        throw std::runtime_error("Could not open image " + filename);
    }
   
    RobotProject rp(argc, argv);
    if (!rp.preprocessMap(img))
    {
        std::cerr << "(Critical) Failed to preprocess map" << std::endl;
        return false;
    }

    Path path;
    if (!rp.planPath(img, path))
    {
        std::cerr << "(Critical) Failed to plan path" << std::endl;
        return false;
    }
    
    int count = 0;

    while (count<10)
    {

        std::vector<double> state;
        if (!rp.localize(img, state))
        {
        std::cerr << "(Warning) Failed localization" << std::endl;
        continue;
        }

        count++;        

    }
	
	return 0;

}

	
