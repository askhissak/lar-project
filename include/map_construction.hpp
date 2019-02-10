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
struct Polygon 
{
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
struct Obstacle : Polygon
{
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

// A type of polygon
struct Border : Polygon
{

  Border(std::vector<cv::Point> corners):
    Polygon(corners)
  {}

  Border(): 
    Border ({cv::Point(0,0)})
  {}
  
};

// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Gate : Polygon
{
  Gate(std::vector<cv::Point> corners):
    Polygon(corners)
  {}

  Gate(): 
    Gate({cv::Point(0,0)})
  {}

};

// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Robot : Polygon
{
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

struct Circle
{
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
struct Victim : Circle
{
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
  Border border;
  std::vector<Victim> victims;
  Gate gate;
  Robot robot;

  Map(cv::Point origin, int length, int width):
    origin(origin), length(length), width(width)
  {}

  Map():
    Map(cv::Point(0,0), 1500, 1000)
  {}

  void printAll();
  cv::Mat showMap();

};

//Function declarations
bool findObstacles(cv::Mat const & map, Map & map_object);
bool findBorders(cv::Mat const & map, Map & map_object);
bool findROI(cv::Mat const & map, Map & map_object);
bool findGate(cv::Mat const & map, Map & map_object);
bool findRobot(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object);
double getOrientation(cv::Point end_point, cv::Point start_point); 
bool buildMap(cv::Mat const & map, cv::Mat const & robot_plane, Map & map_object);

#endif
