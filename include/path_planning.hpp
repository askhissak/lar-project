#ifndef LAR_PATHPLANNING_HPP
#define LAR_PATHPLANNING_HPP

//Forward declared dependencies
const int COLLISION_THRESHOLD = 100;
const double KMAX = 0.01;

//Included dependencies
#include "map_construction.hpp"
#include "path.h"
#include "dubins.hpp"

//Classes
struct Neighbor
{
    cv::Point neighbor;
    double weight;

    Neighbor(cv::Point p, double w):
        neighbor(p), weight(w)
    {}

    Neighbor():
        Neighbor(cv::Point(0,0),0)
    {}

};

struct CustomCompare
{
    bool operator()(const Neighbor lhs, const Neighbor rhs) const
    {
        return lhs.weight > rhs.weight;
    }
};

//Classes
struct NeighborVertices
{
    cv::Point vertex;
    std::vector<Neighbor> neighbors;

    NeighborVertices(cv::Point p, std::vector<Neighbor> Neighbors):
        vertex(p), neighbors(Neighbors)
    {}

    NeighborVertices(): 
        NeighborVertices (cv::Point(0,0),{})
    {}

    NeighborVertices(cv::Point p): 
        vertex(p)
    {}

    void printNeigbors()
    {
        std::cout<<"Neigbors of vertex "<<this->vertex<<" are ";
        for(int i=0;i<this->neighbors.size();++i)
        {
            std::cout<<this->neighbors[i].neighbor;
            std::cout<<" with weight "<<this->neighbors[i].weight<<std::endl;
        }
    }
    
    // std::vector<cv::Point> getNeighbors(cv::Point p)
    // {
        
    // }
};

//Function declarations

//Dijkstra file
// void planLine(std::vector<cv::Point> & points_path, cv::Point end, cv::Point start, cv::Mat &out);
void planDubins(Map & map_object, std::vector<DubinsArc> & arcs, 
                double x0, double y0, double th0, double xf, double yf, double thf, double Kmax, cv::Mat &out);
bool restorePath(std::vector<NeighborVertices> graph, std::vector<cv::Point> came_from,
								    std::vector<cv::Point> & sequence, cv::Point start, cv::Point end);
bool equalPoints(cv::Point p1, cv::Point p2);
int containsPoint(std::vector<NeighborVertices> adjList, cv::Point p);
bool pointCollision(Map map, cv::Point p, int threshold);
// bool lineCollision(Map map, cv::Point p1, cv::Point p2, cv::Point2d &intersection);
std::vector<NeighborVertices> formAdjList(std::vector<std::vector<cv::Point>> src, Map & map_object);
// std::vector<double> shortestPath(std::vector<NeighborVertices> &adjList, cv::Point start, cv::Point end, std::vector<cv::Point> &point_path);
bool shortestPath(std::vector<NeighborVertices> &adjList, cv::Point start, cv::Point end,     
								std::vector<double> &cost_so_far, std::vector<cv::Point> &came_from);
void addVertex(std::vector<NeighborVertices> & adjList, cv::Point start);
void printShortestPath(std::vector<double> &dist, cv::Point &start, std::vector<cv::Point> came_from);
void printPath(std::vector<NeighborVertices> adjList, std::vector<cv::Point> came_from);
void printPosePath(Path path);
bool convertToPosePath(std::vector<DubinsArc> arcs, Path & path);
bool planMissionOne(Map & map_object, std::vector<NeighborVertices> & adjList, Path & path, std::vector<int> & order);


//Voronoi file
bool linesIntersect(cv::Point a1, cv::Point a2, cv::Point b1, cv::Point b2, cv::Point & int_point);
void removeDuplicates(std::vector<cv::Point> & src, std::vector<cv::Point> & dst);
static void drawPoint( cv::Mat& img, cv::Point2f fp, cv::Scalar color );
static void drawDelaunay( cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color );
static void drawVoronoi( cv::Mat& img, cv::Subdiv2D& subdiv, std::vector<std::vector<cv::Point> > & roadmap );
bool constructRoadmap(cv::Mat const &map, Map & map_object, std::vector<NeighborVertices> & adjList);

double timer();
// bool planMissionOne(Map & map_object, Path & path);
bool planMissionTwo(Map & map_object, Path & path);
// bool planDubins(Map & map_object, std::vector<cv::Point> & points_path, Path & path, std::vector<int> & order);
bool planMission(cv::Mat const & map, Map & map_object, Path & path, std::vector<int> & order);

#endif