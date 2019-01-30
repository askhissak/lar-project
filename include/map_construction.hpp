#ifndef LAR_MAPCONSTRUCTION_HPP
#define LAR_MAPCONSTRUCTION_HPP

//Forward declared dependencies
//

//Included dependencies
#include <iostream>
#include <vector>

#include "path.h"

//Classes
// A general convex shape represented by corner coordinates
class Polygon 
{
private:
//   int corner_number;
  // cv::Scalar color;
//   cv::Point center;

public:
  std::vector<cv::Point> corners;

  Polygon(std::vector<cv::Point> corners):
    corners(corners)
  {}

  Polygon(): 
    Polygon ({cv::Point(0,0)})
  {}

  cv::Point getCenter()
  {
    cv::Moments mo = cv::moments(corners);
	  cv::Point center = cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00);
    return center;
  }

  std::vector<cv::Point> getContours(cv::Mat map);

//   double getLengthOfSide(cv::Point start_corner, cv::Point end_corner)
//   {
//     return cv::norm(end_corner-start_corner);
//   }
//   void setCorners(std::vector<cv::Point> Corners)
//   {
//       corners = Corners;
//   }
};

// A type of polygon
class Obstacle : public Polygon
{

private:

public:
  Obstacle(std::vector<cv::Point> corners):
    Polygon(corners)
  {}

  Obstacle(): 
    Obstacle ({cv::Point(0,0)})
  {}
//   void setCorners(std::vector<cv::Point> Corners)
//   {
//       corners = Corners;
//   }
  
};

// A configuration of the robot along the path, represented by x, y, orientation and curvature
class Gate : public Polygon
{
  
private:

public:
  Gate(std::vector<cv::Point> corners):
    Polygon(corners)
  {}

  Gate(): 
    Gate({cv::Point(0,0)})
  {}

};

// A configuration of the robot along the path, represented by x, y, orientation and curvature
class Robot : public Polygon
{
  
private:

public:
  Pose pose;

  Robot(std::vector<cv::Point> corners):
    Polygon(corners)
  {}

  Robot(): 
    Robot({cv::Point(0,0)})
  {}

  void setPose(Pose Pose)
  {
      pose = Pose;
  }
  cv::Point getDirectionSidePoint();

};

class Circle
{

private:

public:
  cv::Point center;
  double radius;

  Circle(cv::Point center, double radius):
    center(center), radius(radius)
  {}

  Circle(): 
    Circle (cv::Point(0,0), 0.0)
  {}

};

// A configuration of the robot along the path, represented by x, y, orientation and curvature
class Victim : public Circle
{
  
private:

public:
  Pose pose;
  int index;

  Victim(cv::Point center, double radius):
    Circle(center, radius)
  {}

  Victim(): 
    Victim (cv::Point(0,0), 0.0)
  {}
  void setPose(Pose Pose)
  {
      pose = Pose;
  }
  int getIndex()
  {
    return index;
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
class Map 
{

private:
  cv::Point origin;
  int length;
  int width;

public: 
  std::vector<Obstacle> obstacles;
  std::vector<Victim> victims;
  Gate gate;
  Robot robot;

  Map(cv::Point origin, int length, int width):
    origin(origin), length(length), width(width)
  {}

  Map():
    Map(cv::Point(0,0), 1500, 1000)
  {}

//   void addObstacles(Obstacle obstacle)
//   {
//       this->obstacles.push_back(obstacle);
//   }
  void printAll();
  cv::Mat showMap();
  
//   void printObstacles()
//   {
//     for(int i = 0; i<obstacles.size();++i)
//     {
//         for(int j = 0; j<obstacles[i].corners.size();++j)
//         {   
//             std::cout <<"Obstacle "<<i<<" "<<this->obstacles[i].corners[j]<< std::endl;
//         }
//     }
//   }

//   void printVictims()
//   {
//     for(int i = 0; i<victims.size();++i)
//     {
//         std::cout <<"Victim "<<i<<" "<<this->victims[i].center<< std::endl;
//     }
//   }

};

//Function declarations
bool findObstacles(cv::Mat const & map, Map & map_object);
bool findROI(cv::Mat const & map, Map & map_object);
bool findGate(cv::Mat const & map, Map & map_object);
bool findRobot(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object);
double getOrientation(cv::Point end_point, cv::Point start_point); 
bool buildMap(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object);

#endif
