// path_planning.cpp: PATH PLANNING
#include <string>
#include <vector>
#include <queue>
#include <unordered_set>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "path_planning.hpp"
#include "dubins.hpp"
#include "map_construction.hpp"
#include "map_extraction.hpp"
// #include "polygon_offset.hpp"
// #include "voronoi.hpp"

#include "path.h"


bool PP_developer_session = true ; // if true  -> Retrieves desired debugging and log content 
								    // if false -> Process everything without graphical output 

bool PP_demo_session      = false ;  // if true  -> Retrieves desired demo real time visualization of desired output 
								    // if false -> Process everything without graphical output 


std::vector<NeighborVertices> roadmap;
cv::Mat local_map;

bool pointCollision(Map map, cv::Point p, int threshold)
{
    double obs_test, bord_test;
    int count = 0;
    for(int k=0;k<map.obstacles.size();++k)
    {
        obs_test = cv::pointPolygonTest(map.obstacles[k].corners, p, true);
        if(obs_test<-threshold)
        {
            count++;            
        }
        else 
        {
            break;
        }
    }

    // bord_test = cv::pointPolygonTest(map.border.corners, p, true);
    // std::cout<<"-------------------------------------------------------------Test for border collision "<<bord_test<<std::endl;

    if(count==map.obstacles.size())
    {
        return false;
    }
    else
    {
        return true;
    }

    // if(bord_test<-threshold)
    // {
    //     return false;
    // }
    // else
    // {
    //     return true;
    // }

}

bool closeDistance(cv::Point p1, cv::Point p2)
{  
    double dist = sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
    if(dist<20)
    {
        return true;
    }
    else
    {
        return false;
    }
  
}

bool equalPoints(cv::Point p1, cv::Point p2)
{  
  if(p1.x==p2.x && p1.y==p2.y)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

bool comparePoint(cv::Point p1, cv::Point p2)
{
	if (p1.x != p2.x)
		return p1.x > p2.x;
	else
		return p1.y > p2.y;
}

int containsPoint(std::vector<NeighborVertices> adjList, cv::Point p)
{
  int count = 0, index = 0;
  for(int i=0;i<adjList.size();++i)
  {
    if(equalPoints(adjList[i].vertex,p))
    {
      index = i;
      count++;
    }
  }
  if(count>0)
  {
    return index;
  }
  else
  {
    index = -1;
    return index;
  }
  
}

bool containsPointInNeighbors(std::vector<Neighbor> neigbors, cv::Point p)
{
  int count = 0;
  for(int i=0;i<neigbors.size();++i)
  {
    if(equalPoints(neigbors[i].neighbor,p))
    {
      count++;
    }
  }
  if(count>0)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}

void connectGraph(Map map_object, std::vector<NeighborVertices> & graph, cv::Mat & out)
{
    bool collision_flag, inside_flag;
    for(int i=0;i<graph.size();++i)
    {
        for(int j=0;j<graph.size();++j)
        {
            if(i!=j)
            {
                cv::Point intersection;

                cv::LineIterator it(local_map, graph[i].vertex, graph[j].vertex, 8);
                std::vector<cv::Vec3b> buf(it.count);
                std::vector<cv::Point> points(it.count);

                for(int k = 0; k < it.count; k++, ++it)
                {
                    buf[k] = (const cv::Vec3b)*it;
                    points[k] = it.pos();    
                    if(pointCollision(map_object, points[k], COLLISION_THRESHOLD))
                    {
                        collision_flag = true;
                        intersection = points[k];
                        break;
                    }else
                    {
                        collision_flag = false;
                    }
                    
                }

                inside_flag = containsPointInNeighbors(graph[i].neighbors, graph[j].vertex);
                if(!inside_flag && !collision_flag)
                {
                    cv::line(out, graph[i].vertex , graph[j].vertex , cv::Scalar( 0 , 0 ,150), 2);
                    double dist = sqrt(pow((graph[j].vertex.x-graph[i].vertex.x),2)+pow((graph[j].vertex.y-graph[i].vertex.y),2));
                    if(dist<1000)
                    {
                        std::cout<<"Adding vertex "<<graph[j].vertex<<" to the list of neighbors of vertex "<<graph[i].vertex<<std::endl;
                        Neighbor n = Neighbor(graph[j].vertex, dist);
                        graph[i].neighbors.push_back(n);
                    }
                }
                else if(collision_flag)
                {
                    cv::circle(out, intersection, 10, cv::Scalar(0,0,250),10,8,0);                    
                }
            }            
            
        }
    }


}

// An adjacency list. Each adjList[i] holds a all the friends of node i.
// The first int is the vertex of the friend, the second int is the edge weight.
std::vector<NeighborVertices> formAdjList(std::vector<std::vector<cv::Point>> src, Map & map_object)
{
    std::vector<NeighborVertices> adjList;
    NeighborVertices vert;
    bool collision_flag, inside_flag;
    int idx = -2, previous, next, insertIndex;
    double dist;
    std::vector<cv::Point> upd_roadmap;
    cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));

    for(int i=0;i<src.size();++i)
    {
        for(int j=0;j<src[i].size();++j)
        {
            std::cout<<"Facet "<<i<<", element "<<j<<". Vertex "<<src[i][j]<<std::endl;
            collision_flag = pointCollision(map_object, src[i][j], COLLISION_THRESHOLD);
            // std::cout<<"Collision flag "<<collision_flag<<std::endl;
            if(!collision_flag)
            {
                idx = containsPoint(adjList, src[i][j]);
                // std::cout<<"Already in the Adjacency list flag "<<idx<<std::endl;
                // std::cout<<"Facet "<<src[i]<<" contains point "<<src[i][j]<<std::endl;
                if(adjList.empty() && src[i][j].x>0 && src[i][j].y>0 && src[i][j].x<1495 && src[i][j].y<1035)
                {
                    // std::cout<<"Adding the first element to the Adjacency list..."<<std::endl;
                    // std::cout<<"Facet "<<i<<", element "<<j<<std::endl;
                    vert = NeighborVertices(src[i][j]);
                    adjList.push_back(vert);
                    insertIndex = 0;
                    std::cout<<"Added "<<adjList[insertIndex].vertex<<" to the Adjacency list."<<std::endl;
                    if(j==0)
                    {
                        previous = src[i].size()-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                    else if(j==(src[i].size()-1))
                    {
                        previous = j-1;
                        next = 0;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                    else
                    {
                        previous = j-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Middle facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Middle facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                }
                else if(idx==-1 && src[i][j].x>0 && src[i][j].y>0 && src[i][j].x<1495 && src[i][j].y<1035)
                {
                    // std::cout<<"Adding a new element to the Adjacency list..."<<std::endl;
                    // std::cout<<"Facet "<<i<<", element "<<j<<std::endl;
                    vert = NeighborVertices(src[i][j]);
                    adjList.push_back(vert);
                    insertIndex = adjList.size()-1;
                    std::cout<<"Added "<<adjList[insertIndex].vertex<<" to the Adjacency list."<<std::endl;
                    if(j==0)
                    {
                        previous = src[i].size()-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                    else if(j==(src[i].size()-1))
                    {
                        previous = j-1;
                        next = 0;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                    else
                    {
                        previous = j-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Middle facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[insertIndex].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Miidle facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[insertIndex].neighbors.push_back(n);
                        }
                    }
                }
                else if(idx>=0)
                {
                    std::cout<<"Vertex "<<src[i][j]<<" is already in the Adjacency list!"<<std::endl;
                    std::cout<<"Adding its neighbors to an existing element in the Adjacency list..."<<std::endl;
                    // std::cout<<"Facet "<<i<<", element "<<j<<std::endl;
                    if(j==0)
                    {
                        previous = src[i].size()-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"First facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                    }
                    else if(j==(src[i].size()-1))
                    {
                        previous = j-1;
                        next = 0;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Last facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                    }
                    else
                    {
                        previous = j-1;
                        next = j+1;
                        collision_flag = pointCollision(map_object, src[i][previous], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][previous]);
                        if(!collision_flag && !inside_flag && src[i][previous].x>0 && src[i][previous].y>0 && src[i][previous].x<1495 && src[i][previous].y<1035)
                        {
                            // std::cout<<"Middle facet element "<<src[i][j]<<". Adding the previous element "<<src[i][previous]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][previous].x-src[i][j].x),2)+pow((src[i][previous].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][previous], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                        collision_flag = pointCollision(map_object, src[i][next], COLLISION_THRESHOLD);
                        inside_flag = containsPointInNeighbors(adjList[idx].neighbors, src[i][next]);
                        if(!collision_flag && !inside_flag && src[i][next].x>0 && src[i][next].y>0 && src[i][next].x<1495 && src[i][next].y<1035)
                        {
                            // std::cout<<"Middle facet element "<<src[i][j]<<". Adding the next element "<<src[i][next]<<" in the facet to the neighbors..."<<std::endl;
                            dist = sqrt(pow((src[i][next].x-src[i][j].x),2)+pow((src[i][next].y-src[i][j].y),2));
                            Neighbor n = Neighbor(src[i][next], dist);
                            adjList[idx].neighbors.push_back(n);
                        }
                    }
                }
            }
            else 
            {
                std::cout<<"Collision detected! Discarding the vertex "<<src[i][j]<<std::endl;
                continue;
            }
        }
            

        cv::polylines(out, src[i], true, cv::Scalar(250,0,0), 1, CV_AA, 0);
    }

    connectGraph(map_object, adjList, out);

    // for(int i=0;i<adjList.size();++i)
    // {
    //     for(int j=0;j<adjList.size();++j)
    //     {
    //         if(i!=j)
    //         {
    //             cv::Point intersection;

    //             cv::LineIterator it(local_map, adjList[i].vertex, adjList[j].vertex, 8);
    //             std::vector<cv::Vec3b> buf(it.count);
    //             std::vector<cv::Point> points(it.count);

    //             for(int k = 0; k < it.count; k++, ++it)
    //             {
    //                 buf[k] = (const cv::Vec3b)*it;
    //                 points[k] = it.pos();    
    //                 if(pointCollision(map_object, points[k], COLLISION_THRESHOLD))
    //                 {
    //                     collision_flag = true;
    //                     intersection = points[k];
    //                     break;
    //                 }else
    //                 {
    //                     collision_flag = false;
    //                 }
                    
    //             }

    //             inside_flag = containsPointInNeighbors(adjList[i].neighbors, adjList[j].vertex);
    //             if(!inside_flag && !collision_flag)
    //             {
    //                 cv::line(out, adjList[i].vertex , adjList[j].vertex , cv::Scalar( 0 , 0 ,150), 2);
    //                 dist = sqrt(pow((adjList[j].vertex.x-adjList[i].vertex.x),2)+pow((adjList[j].vertex.y-adjList[i].vertex.y),2));
    //                 if(dist<1000)
    //                 {
    //                     std::cout<<"Adding vertex "<<adjList[j].vertex<<" to the list of neighbors of vertex "<<adjList[i].vertex<<std::endl;
    //                     Neighbor n = Neighbor(adjList[j].vertex, dist);
    //                     adjList[i].neighbors.push_back(n);
    //                 }
    //             }
    //             else if(collision_flag)
    //             {
    //                 cv::circle(out, intersection, 10, cv::Scalar(0,0,250),10,8,0);                    
    //             }
    //         }            
            
    //     }
    // }

    for(int i=0;i<adjList.size();++i)
    {
        cv::circle(out, adjList[i].vertex, 5, cv::Scalar(0,250,0),5,8,0);
        // for(int j=0;j<adjList[i].neighbors.size();++j)
        // {
        //     cv::circle(out, adjList[i].neighbors[j].neighbor, 10, cv::Scalar(0,0,250),10,8,0);
        // }

    }

	if(PP_developer_session == true) showImage("Roadmap", local_map+out);

    // Our graph is now represented as an adjacency list. Return it.
    return adjList; 
}

// Given an Adjacency List, find all shortest paths from "start" to all other vertices.
bool shortestPath(std::vector<NeighborVertices> &adjList, cv::Point start, cv::Point end,     
								std::vector<double> &cost_so_far, std::vector<cv::Point> &came_from)
{
    std::cout << "\nGetting the shortest path from " << start << " to "<<end<<std::endl;

	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	std::string win_dijkstra = "Dijkstra path";

    int current_graph_index, next_graph_index;
    double priority;


    std::priority_queue<Neighbor, std::vector<Neighbor>, CustomCompare> frontier;
    frontier.push(Neighbor(start, 0));

    cost_so_far.clear();
    came_from.clear();

    int n = adjList.size();
    for(int i = 0; i < n; i++)
    {
        cost_so_far.push_back(1000000007);
        came_from.push_back(cv::Point(0,0));
        // std::cout<<"Cost so far "<<cost_so_far[i]<<std::endl;
    }

    for(int k=0;k<adjList.size();++k)
    {
        if(equalPoints(adjList[k].vertex, start))
        {
            current_graph_index = k;
        }
    }

    cost_so_far[current_graph_index] = 0;
    came_from[current_graph_index] = start;        

    while(!frontier.empty())
    {
        cv::Point current_vertex = frontier.top().neighbor;
        double current_weight = frontier.top().weight;
        frontier.pop();

        if(equalPoints(current_vertex,end))
        {
			break;
        }

        for(int k=0;k<adjList.size();++k)
        {
            if(equalPoints(adjList[k].vertex, current_vertex))
            {
                current_graph_index = k;
            }
        }

        for(int i = 0; i < adjList[current_graph_index].neighbors.size(); ++i)
        {
            for(int j=0;j<adjList.size();++j)
            {
                if(equalPoints(adjList[j].vertex, adjList[current_graph_index].neighbors[i].neighbor))
                {
                    next_graph_index = j;
                }
            }            
            
            cv::Point next_vertex = adjList[current_graph_index].neighbors[i].neighbor;
            double weight = adjList[current_graph_index].neighbors[i].weight;

            double heuristic = sqrt(pow((end.x-next_vertex.x),2)+pow((end.y-next_vertex.y),2));
            
            double new_cost = current_weight + weight + heuristic;

            // std::cout<<"Current index "<<current_graph_index<<std::endl;
            std::cout<<"Current vertex "<<current_vertex<<std::endl;

            // for(int k=0;k<adjList.size();++k)
            // {
            //     if(equalPoints(adjList[k].vertex, current_vertex))
            //     {                  
            //         for(int j=0; j<adjList[k].neighbors.size();++j)
            //         {
            //             std::cout<<"Neighbors of current vertex "<<adjList[k].neighbors[j].neighbor<<std::endl;
            //         }
            //     }
            // }
            // std::cout<<"Current frontier cost "<<current_weight<<std::endl;
            std::cout<<"Cost of current "<<cost_so_far[current_graph_index]<<std::endl;            

            // std::cout<<"Next index "<<next_graph_index<<std::endl;
            std::cout<<"Next vertex "<<next_vertex<<std::endl;
            std::cout<<"New cost "<<new_cost<<std::endl;
            std::cout<<"Cost of next "<<cost_so_far[next_graph_index]<<std::endl;

            if(new_cost < cost_so_far[next_graph_index])
            {
                // std::cout << "Test\n";
                cost_so_far[next_graph_index] = new_cost;
            	priority = new_cost;
				// std::cout<<"Heuristics "<<heuristics<<std::endl;
				std::cout<<"Priority order of frontier "<<priority<<std::endl<<std::endl;
                frontier.push(Neighbor(next_vertex, priority));
                came_from[next_graph_index] = current_vertex;
            	// std::cout<<"Current vertex "<<current_vertex<<std::endl;
				cv::circle(out, current_vertex, 10 , cv::Scalar( 0,0,220 ), 5, 8, 0 );
            }
            
        }
		
		if(PP_developer_session == true)
		{
			cv::namedWindow(win_dijkstra.c_str(), CV_WINDOW_NORMAL);
			cv::resizeWindow(win_dijkstra.c_str(), 640, 512);
			cv::imshow(win_dijkstra, local_map+out);
			cv::waitKey(1);
		}
    }

    return true;
}

void addVertex(Map map_object, std::vector<NeighborVertices> & adjList, cv::Point start)
{
	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
    double min_dist = 3000, dist;
    int index;
    for(int i = 0; i < adjList.size(); i++)
    {
        if(equalPoints(adjList[i].vertex, start))
        {
            std::cout<<"Graph already contains this vertex "<<start<<std::endl;
            return;
        }
        dist = sqrt(pow((adjList[i].vertex.x-start.x),2)+pow((adjList[i].vertex.y-start.y),2));
        if(dist<min_dist)
        {
            min_dist = dist;
            index = i;
        }
    }

    Neighbor new_neighbor = Neighbor(start, min_dist);
    adjList[index].neighbors.push_back(new_neighbor);
    Neighbor n = Neighbor(adjList[index].vertex, min_dist);
    std::vector<Neighbor> nl;
    nl.push_back(n);
    NeighborVertices new_vertex = NeighborVertices(start, nl);
    adjList.push_back(new_vertex);

    connectGraph(map_object, adjList, out);

	cv::circle(out, adjList[index].vertex, 10 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	if(PP_developer_session == true) showImage("Added vertex", local_map+out);

}
    
void printShortestPath(std::vector<double> &dist, cv::Point &start, std::vector<cv::Point> came_from)
{
    std::cout << "\nPrinting the shortest paths for node " << start << ".\n";
    for(int i = 0; i < dist.size(); i++)
    {
     std::cout << "The distance from node " << start << " to node " << came_from[i] << " is: " << dist[i] << std::endl;
    }
}

void printPath(std::vector<NeighborVertices> adjList, std::vector<cv::Point> came_from)
{
    for(int i = 0; i < adjList.size(); i++)
    {
     std::cout << "We came to the node " << adjList[i].vertex << " from node " << came_from[i] << std::endl;
    }
}

void printPosePath(Path path)
{
    for(int i = 0; i < path.size(); i++)
    {
        std::cout << "Pose " <<i<< " of the path has properties: s = " << path.points[i].s;
        std::cout << ", x = " <<path.points[i].x << ", y = " << path.points[i].y;
        std::cout << ", theta = " <<path.points[i].theta<< std::endl;
    }
}

void planDubins(Map & map_object, std::vector<cv::Point> & points_path, double x0, double y0, double th0, double xf, double yf, double thf, cv::Mat &out)
{
	// cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	// cv::Mat map = map_object.showMap(); //First map acquisition
	std::string win_dubins = "Dubins segment path";
	// cv::Mat original_map = map; 

	int npts = 20;
	double s1,s2,s3;

	DubinsCurve curve;

	curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, KMAX); 

	for(int j=0; j<npts;++j)
	{
		s1 = curve.arc1.L/npts*j;
		circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{
		s2 = curve.arc2.L/npts*j;
		circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}
	for(int j=0; j<npts;++j)
	{	
		s3 = curve.arc3.L/npts*j;
		circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
	}

	if(PP_developer_session == true)
	{
		cv::namedWindow(win_dubins.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(win_dubins.c_str(), 640, 512);
		cv::imshow(win_dubins, local_map+out);
		cv::waitKey(0);
	}               
}

bool restorePath(std::vector<NeighborVertices> graph, std::vector<cv::Point> came_from,
								    std::vector<cv::Point> & sequence, cv::Point start, cv::Point end)
{
    int graph_index = 0; 
    cv::Point graph_vertex = end;
    sequence.push_back(end);

    while(!equalPoints(graph_vertex, start))
    {
        for(int k=0;k<graph.size();++k)
        {
            if(equalPoints(graph_vertex, graph[k].vertex))
            {
                graph_index = k;
            }
        }
        for(int i = 0;i<came_from.size();++i)
        {
            if(graph_index == i)
            {
                sequence.push_back(came_from[i]);
                graph_vertex = came_from[i];
                break;	
            }
            else if(graph_index!=i && i==(came_from.size()-1))
            {
                std::cout<<"Could not retrieve a path!"<<std::endl;
                return false;
            }
	    }   
    }

	return true;

}

bool convertToPosePath(std::vector<cv::Point> & point_path, Path & path)
{
    std::vector<Pose> new_path;

    if(point_path.empty())
    {
        return false;
    }

    for(int i=0;i<point_path.size();++i)
    {
        double s, x0, y0, theta;
        if(i==0)
        {
            s = 0;
            x0 = point_path[i].x/1000;
            y0 = -point_path[i].y/1000-1.05;
            theta = -getOrientation(point_path[i+1], point_path[i]);
        }
        else if(i==(point_path.size()-1))
        {
            s = sqrt(pow((point_path[i].x/1000-point_path[i-1].x/1000),2)+pow((point_path[i].y/1000-point_path[i-1].y/1000),2));
            x0 = point_path[i].x/1000;
            y0 = -point_path[i].y/1000-1.05;
            theta = -getOrientation(point_path[i], point_path[i-1]);
        }
        else
        {
            s = sqrt(pow((point_path[i].x/1000-point_path[i-1].x/1000),2)+pow((point_path[i].y/1000-point_path[i-1].y/1000),2));
            x0 = point_path[i].x/1000;
            y0 = -point_path[i].y/1000-1.05;
            theta = -getOrientation(point_path[i+1], point_path[i]);
        }
        
        Pose pose = Pose(s, x0, y0, theta, KMAX);
        new_path.push_back(pose);
    }

    path.setPoints(new_path);
    return true;
}

bool planMissionOne(Map & map_object, std::vector<NeighborVertices> & adjList, std::vector<cv::Point> & points_path, Path & path, std::vector<int> & order)
{
    cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	std::vector<double> dist;
	bool animate = true;
	std::string win_dubins = "Dubins path";

	std::vector<double> cost_so_far;
	std::vector<cv::Point> came_from;
    std::vector<cv::Point> path_vertices;

    // adjList = formAdjList(src, map_object, map);

    cv::Point start = map_object.robot.getCenter();
    cv::Point end = map_object.gate.getCenter();
	cv::Point direction = map_object.robot.getDirectionSidePoint();

	double robot_x0 = start.x, robot_y0 = start.y, robot_th0 = getOrientation(direction, start), 
			gate_xf = end.x, gate_yf = end.y, gate_thf; 

    if(map_object.gate.getCenter().x<70)
    {
        gate_thf = M_PI;
    }
    else if(map_object.gate.getCenter().y>980)
    {
        gate_thf = 3*M_PI/2;        
    }
    else if(map_object.gate.getCenter().x>1440)
    {
        gate_thf = 0;        
    }
    else if(map_object.gate.getCenter().y<70)
    {
        gate_thf = M_PI/2;        
    }

	double robot_dist = sqrt(pow((end.x-robot_x0),2)+pow((end.y-robot_y0),2));
	// double previous_distance, current_distance;

    double x0,y0,th0,xf,yf,thf;
	int radius = 10;
	int graph_index;

    addVertex(map_object, adjList, start);

    for(int k=0;k<order.size();++k)
    {
		if(k==0)
		{
			addVertex(map_object, adjList, map_object.victims[order[k]].center);
			std::cout<<"Added point "<<adjList[adjList.size()-1].vertex<<std::endl;

            // x0 = robot_x0;
            // y0 = robot_y0;
            // th0 = robot_th0;
            // xf = map_object.victims[order[k]].center.x;
            // yf = map_object.victims[order[k]].center.y;
            // thf = getOrientation(map_object.victims[order[k+1]].center, map_object.victims[order[k]].center);

            // planDubins(map_object, points_path, x0, y0, th0, xf, yf, thf, out);
            // for(int j=0;j<points_path.size();++j)
            // {
            //     bool collision_flag = pointCollision(map_object, points_path[j], COLLISION_THRESHOLD);
            //     if(!collision_flag)
            // }
            // if(/*dubins*/)
            // {
            //     //plan
            // }
			if(!shortestPath(adjList, start, map_object.victims[order[k]].center, cost_so_far, came_from))
			{
				std::cout<<"Can't find a path!"<<std::endl;
			}
			else
			{
                std::vector<cv::Point> path_vertices_temp;
                std::cout<<"Found a path!"<<std::endl;
                printPath(adjList, came_from);
                if(restorePath(adjList, came_from, path_vertices_temp, start, map_object.victims[order[k]].center))
                {
                    for(int i=(path_vertices_temp.size()-1);i>=0;--i)
                    {
                        cv::circle(out, path_vertices_temp[i], 15 , cv::Scalar( 255 , 255 , 0 ), 10, 8, 0 );
                        path_vertices.push_back(path_vertices_temp[i]);
                    }                    
                }
			}
			
		}
		else
		{
			addVertex(map_object, adjList, map_object.victims[order[k]].center);
			std::cout<<"Added point "<<adjList[adjList.size()-1].vertex<<std::endl;
			if(!shortestPath(adjList, map_object.victims[order[k-1]].center, map_object.victims[order[k]].center, cost_so_far, came_from))
			{
				std::cout<<"Can't find a path!"<<std::endl;
			}
			else
			{
                std::vector<cv::Point> path_vertices_temp;
                std::cout<<"Found a path!"<<std::endl;
                printPath(adjList, came_from);
                if(restorePath(adjList, came_from, path_vertices_temp, map_object.victims[order[k-1]].center, map_object.victims[order[k]].center))
                {
                    for(int i=(path_vertices_temp.size()-2);i>=0;--i)
                    {
                        int idx;
                        cv::circle(out, path_vertices_temp[i], 15 , cv::Scalar( 255 , 255 , 0 ), 10, 8, 0 );
                        path_vertices.push_back(path_vertices_temp[i]);                                                  
                    
                    }
                }
			}		
		}
		// 	// if (animate)
		// 	// {
		// 	// 	// cv::Mat img_copy = map.clone();
		// 	// 	cv::namedWindow(win_dubins.c_str(), CV_WINDOW_NORMAL);
		// 	// 	cv::resizeWindow(win_dubins.c_str(), 640, 512);
		// 	// 	cv::imshow(win_dubins, map+out);
		// 	// 	cv::waitKey(0);
		// 	// }
        // }

        // printShortestPath(dist, start, came_from);

    }

    addVertex(map_object, adjList, cv::Point(gate_xf, gate_yf));
    std::cout<<"Added gate "<<adjList[adjList.size()-1].vertex<<std::endl;
    if(!shortestPath(adjList, map_object.victims[order[order.size()-1]].center, cv::Point(gate_xf,gate_yf), cost_so_far, came_from))
    {
        std::cout<<"Can't find a path!"<<std::endl;
    }
    else
    {
        std::vector<cv::Point> path_vertices_temp;
        std::cout<<"Found a path!"<<std::endl;
        printPath(adjList, came_from);
        if(restorePath(adjList, came_from, path_vertices_temp, map_object.victims[order[order.size()-1]].center, cv::Point(gate_xf,gate_yf)))
        {
            for(int i=(path_vertices_temp.size()-2);i>=0;--i)
            {
                int idx;
                cv::circle(out, path_vertices_temp[i], 15 , cv::Scalar( 255 , 255 , 0 ), 10, 8, 0 );
                path_vertices.push_back(path_vertices_temp[i]);                                                  
            }
        }
    }

    for(int j=0;j<path_vertices.size();++j)
    {
        if(j==0)
        {
            xf = robot_x0;
            yf = robot_y0;
            thf = robot_th0;
        }
        else if(j==(path_vertices.size()-1))
        {
            x0 = xf;
            y0 = yf;
            th0 = thf;
            xf = gate_xf;
            yf = gate_yf;
            thf = gate_thf;

            planDubins(map_object, points_path, x0,y0,th0,xf,yf,thf, out);
        }
        else
        {
            x0 = xf;
            y0 = yf;
            th0 = thf;
            xf = path_vertices[j].x;
            yf = path_vertices[j].y;
            thf = getOrientation(path_vertices[j+1], path_vertices[j]);

            planDubins(map_object, points_path, x0,y0,th0,xf,yf,thf, out);
        }
    }     

	//INITIAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(robot_x0,robot_y0), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(robot_x0,robot_y0) , cv::Point(robot_x0+radius*cos(robot_th0), robot_y0+radius*sin(robot_th0)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
    //FINAL POSITION AND CONFIGURATION
    cv::circle(out, cv::Point(gate_xf,gate_yf), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
    cv::line(out, cv::Point(gate_xf,gate_yf) , cv::Point(gate_xf+radius*cos(gate_thf), gate_yf+radius*sin(gate_thf)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);

    std::string name = "Dubins path with roadmap vertices";
    
   
    if(PP_developer_session == true || PP_demo_session == true ) 
    
	{
		cv::namedWindow(name.c_str(), CV_WINDOW_NORMAL);
		cv::resizeWindow(name.c_str(), 640, 512);
		cv::imshow(name.c_str(), out+local_map);
		cv::waitKey(0);
		cv::destroyWindow(name.c_str());		
	}

    if(!convertToPosePath(points_path, path))
    {
        std::cout<<"Cannot convert to Pose path!"<<std::endl;
        return false;
    }

    // for(int i=0;i<path.size();++i)
    // {
    //     // cv::circle(out, cv::Point(path.points[i].x, path.points[i].y), 2, cv::Scalar(255,0,0), 5,8,0 );
    //     // if(path.points[i].x==211 && path.points[i].y==503)
    //     // {
    //         cv::line(out, cv::Point(path.points[i].x, path.points[i].y) , cv::Point(path.points[i].x+radius*cos(path.points[i].theta), path.points[i].y+radius*sin(path.points[i].theta)) , cv::Scalar( 255 , 255 , 255 ), 10, 8, 0);
    //     // }

    // }

    // showImage("Pose path", out+local_map);

    return true;
}

 
// static void drawPoint( Mat& img, Point2f fp, Scalar color )
// {
//     circle( img, fp, 2, color, CV_FILLED, CV_AA, 0 );
// }
 
static void drawDelaunay( cv::Mat& img, cv::Subdiv2D& subdiv, cv::Scalar delaunay_color )
{
 
    std::vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);
    std::vector<cv::Point> pt(3);
    cv::Size size = img.size();
    cv::Rect rect(0,0, size.width, size.height);
 
    for( size_t i = 0; i < triangleList.size(); i++ )
    {
        cv::Vec6f t = triangleList[i];
        pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));
         
        // Draw rectangles completely inside the image.
        if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
}
 
//Draw voronoi diagram
static void drawVoronoi( cv::Mat& img, cv::Subdiv2D& subdiv, std::vector<std::vector<cv::Point>> & roadmap)
{
    std::vector<std::vector<cv::Point2f> > facets;
    std::vector<cv::Point2f> centers;
    subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);
 
    std::vector<cv::Point> ifacet;
    std::vector<std::vector<cv::Point> > ifacets(1);
 
    for( size_t i = 0; i < facets.size(); i++ )
    {
		std::vector<cv::Point> v;
		ifacet.resize(facets[i].size());
        for( size_t j = 0; j < facets[i].size(); j++ )
		{
			cv::Point p = cv::Point((int)facets[i][j].x,(int)facets[i][j].y);
			v.push_back(p);
            ifacet[j] = facets[i][j];
            // std::cout<<"Roadmap point "<<p<<std::endl;
            // std::cout<<"Facets point "<<facets[i][j]<<std::endl;
		}
 
		roadmap.push_back(v);

        cv::Scalar color;
        color[0] = rand() & 255;
        color[1] = rand() & 255;
        color[2] = rand() & 255;
        cv::fillConvexPoly(img, ifacet, color, 8, 0);
 
        ifacets[0] = ifacet;
        polylines(img, ifacets, true, cv::Scalar(), 1, CV_AA, 0);
        circle(img, centers[i], 3, cv::Scalar(), CV_FILLED, CV_AA, 0);

    }
}
 
 
bool constructRoadmap(Map & map_object, std::vector<NeighborVertices> & adjList)
{
	cv::Mat img_orig = local_map.clone();
 
    std::string win_delaunay = "Delaunay Triangulation";
    std::string win_voronoi = "Voronoi Diagram";
    bool animate = true;
    cv::Scalar delaunay_color(255,255,255), points_color(0, 0, 255);     
    cv::Size size = local_map.size();
    cv::Rect rect(0, 0, size.width, size.height); 
    cv::Subdiv2D subdiv(rect);     
    std::vector<cv::Point2f> points;

 
    cv::Point2f p;
	for(int j=0;j<map_object.obstacles.size();++j)
	{
		for(int i=0;i<map_object.obstacles[j].corners.size();++i)
		{
			p = cv::Point2f((float)map_object.obstacles[j].corners[i].x,(float)map_object.obstacles[j].corners[i].y);
			points.push_back(p);
		}
	}

    if(map_object.gate.getCenter().x<70 && map_object.gate.getCenter().y<525)
    {
        // points.push_back(cv::Point2f(15, 125));
        // points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x<70 && map_object.gate.getCenter().y>=525)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        // points.push_back(cv::Point2f(15, 625));
        // points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x<755 && map_object.gate.getCenter().y>980)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        // points.push_back(cv::Point2f(200, 1035));
        // points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x>=755 && map_object.gate.getCenter().y>980)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        // points.push_back(cv::Point2f(940, 1035));
        // points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x>1440 && map_object.gate.getCenter().y<525)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        // points.push_back(cv::Point2f(1495, 125));
        // points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x>1440 && map_object.gate.getCenter().y>=525)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        // points.push_back(cv::Point2f(1495, 625));
        // points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x<755 && map_object.gate.getCenter().y<70)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        // points.push_back(cv::Point2f(200, 15));
        // points.push_back(cv::Point2f(570, 15));
        points.push_back(cv::Point2f(940, 15));
        points.push_back(cv::Point2f(1310, 15));
    }
    else if(map_object.gate.getCenter().x>=755 && map_object.gate.getCenter().y<70)
    {
        points.push_back(cv::Point2f(15, 125));
        points.push_back(cv::Point2f(15, 375));
        points.push_back(cv::Point2f(15, 625));
        points.push_back(cv::Point2f(15, 875));
        points.push_back(cv::Point2f(200, 1035));
        points.push_back(cv::Point2f(570, 1035));
        points.push_back(cv::Point2f(940, 1035));
        points.push_back(cv::Point2f(1310, 1035));
        points.push_back(cv::Point2f(1495, 125));
        points.push_back(cv::Point2f(1495, 375));
        points.push_back(cv::Point2f(1495, 625));
        points.push_back(cv::Point2f(1495, 875));
        points.push_back(cv::Point2f(200, 15));
        points.push_back(cv::Point2f(570, 15));
        // points.push_back(cv::Point2f(940, 15));
        // points.push_back(cv::Point2f(1310, 15));
    }

    // points.push_back(cv::Point2f(15, 125));
    // points.push_back(cv::Point2f(15, 375));
    // points.push_back(cv::Point2f(15, 625));
    // points.push_back(cv::Point2f(15, 875));
    // points.push_back(cv::Point2f(200, 1035));
    // points.push_back(cv::Point2f(570, 1035));
    // points.push_back(cv::Point2f(940, 1035));
    // points.push_back(cv::Point2f(1310, 1035));
    // // points.push_back(cv::Point2f(1495, 125));
    // // points.push_back(cv::Point2f(1495, 375));
    // points.push_back(cv::Point2f(1495, 625));
    // points.push_back(cv::Point2f(1495, 875));
    // points.push_back(cv::Point2f(200, 15));
    // points.push_back(cv::Point2f(570, 15));
    // points.push_back(cv::Point2f(940, 15));
    // points.push_back(cv::Point2f(1150, 15));
    // points.push_back(cv::Point2f(1310, 15));
    
    for( std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); it++)
    {
        subdiv.insert(*it);
        // if (animate)
        // {
        //     cv::Mat img_copy = img_orig.clone();
        //     drawDelaunay( img_copy, subdiv, delaunay_color );
		// 	cv::namedWindow(win_delaunay.c_str(), CV_WINDOW_NORMAL);
    	// 	cv::resizeWindow(win_delaunay.c_str(), 640, 512);
        //     cv::imshow(win_delaunay, img_copy);
        //     cv::waitKey(5);
        // }
         
    }
     
    // drawDelaunay(img_orig, subdiv, delaunay_color);
 
    // for( vector<Point2f>::iterator it = points.begin(); it != points.end(); it++)
    // {
    //     drawPoint(img_orig, *it, points_color);
    // }
     
    cv::Mat img_voronoi = cv::Mat::zeros(img_orig.rows, img_orig.cols, CV_8UC3);
	std::vector<std::vector<cv::Point>> facets;
      
    drawVoronoi( img_voronoi, subdiv, facets );
	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));

	adjList = formAdjList(facets, map_object);
	
    for(int i=0;i<facets.size();++i)
    {
        cv::polylines(out, facets[i], true, cv::Scalar(250,0,0), 1, CV_AA, 0);
    }

	// showImage("Roadmap", map+out);

    // showImage( win_delaunay, img_orig);
    if(PP_developer_session == true) showImage( win_voronoi, img_voronoi);

    // cv::namedWindow(win_voronoi.c_str(), CV_WINDOW_NORMAL);
    // cv::resizeWindow(win_voronoi.c_str(), 640, 512);
    // imshow(win_voronoi, img_voronoi);

	if(PP_developer_session == true) showImage("Roadmap", out+local_map);

    return true;
}

// bool obstacleOffset(cv::Mat input, std::vector<cv::Point> & dst, double offset)
// {
	
// 	cv::Mat BW_Map ;
	
// 	cv::cvtColor(input, BW_Map, CV_BGR2GRAY);
	
// 	std::vector<std::vector<cv::Point>> contours;
// 	std::vector<cv::Vec4i> hierarchy;
	
// 	cv::Mat expanded = cv::Mat::zeros( BW_Map.size(), CV_8UC3 );
	
	
// 	float pixels = offset; // Number of pixels expanded in respect to the original contour
	
	
// 	findContours(BW_Map , contours, hierarchy, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

// 	for(int i=0;i<contours.size();++i)
// 	{
// 		std::cout<<"Offset corner "<<contours[i][0]<<std::endl;
// 	}
	
// 	// for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, cv::Scalar(255,255,255), pixels * 2);
	
// 	// findContours(BW_Map , contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	
// 	// for( int i = 0; i< contours.size(); i++ ) drawContours(expanded, contours, i, Scalar(255,255,255), CV_FILLED );
	

// 	return true;
	
// }

// bool planDubins(Map & map_object, std::vector<cv::Point> & points_path, Path & path, std::vector<int> & order)
// {
//     // std::vector<cv::Point> points_path;
// 	cv::Mat out(1050, 1510, CV_8UC3, cv::Scalar(0,0,0));
	
// 	cv::Mat map = map_object.showMap(); //First map acquisition
	
// 	cv::Mat original_map = map; 

// 	cv::Point direction = map_object.robot.getDirectionSidePoint();

//     double robot_x0 = map_object.robot.getCenter().x, robot_y0 = map_object.robot.getCenter().y, robot_th0 = getOrientation(direction, map_object.robot.getCenter()), 
// 			gate_xf = map_object.gate.getCenter().x, gate_yf = map_object.gate.getCenter().y, gate_thf = 0; 
//     double x0, y0, th0, xf, yf, thf; 
//     double Kmax = 0.01;
//     double radius = 50;
// 	int npts = 1000;
// 	double s1,s2,s3;

// 	DubinsCurve curve;
// 	// int next = 1;

// 	// for(int j=0;j<map_object.victims.size();++j)
// 	// {
// 		for(int i=0;i<order.size();++i)
// 		{
// 			// if(order[i] == next)
// 			// {
// 				if(i==0)
// 				{
// 					x0 = robot_x0;
// 					y0 = robot_y0;
// 					th0 = robot_th0;
// 					xf = map_object.victims[order[i]].center.x;
// 					yf = map_object.victims[order[i]].center.y;
// 					thf = getOrientation(map_object.victims[order[i]].center,map_object.robot.getCenter());
// 				}
// 				else
// 				{
// 					x0 = xf;
// 					y0 = yf;
// 					th0 = thf;
// 					xf = map_object.victims[order[i]].center.x;
// 					yf = map_object.victims[order[i]].center.y;
// 					thf = getOrientation(map_object.victims[order[i]].center, cv::Point(x0,y0));
// 				}

// 				curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 


// 				for(int j=0; j<npts;++j)
// 				{
// 					s1 = curve.arc1.L/npts*j;
// 					circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
// 					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 				}
// 				for(int j=0; j<npts;++j)
// 				{
// 					s2 = curve.arc2.L/npts*j;
// 					circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
// 					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 				}
// 				for(int j=0; j<npts;++j)
// 				{	
// 					s3 = curve.arc3.L/npts*j;
// 					circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
// 					points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 					cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 				}
// 			// }
// 		}
// 	// 	next++;
// 	// } 

// 	x0 = xf;
// 	y0 = yf;
// 	th0 = thf;
// 	xf = gate_xf;
// 	yf = gate_yf;
// 	thf = gate_thf;

// 	curve = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax); 

// 	for(int j=0; j<npts;++j)
// 	{
// 		s1 = curve.arc1.L/npts*j;
// 		circline(s1,curve.arc1.x0,curve.arc1.y0,curve.arc1.th0,curve.arc1.k);
// 		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 	}
// 	for(int j=0; j<npts;++j)
// 	{
// 		s2 = curve.arc2.L/npts*j;
// 		circline(s2,curve.arc2.x0,curve.arc2.y0,curve.arc2.th0,curve.arc2.k);
// 		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 	}
// 	for(int j=0; j<npts;++j)
// 	{	
// 		s3 = curve.arc3.L/npts*j;
// 		circline(s3,curve.arc3.x0,curve.arc3.y0,curve.arc3.th0,curve.arc3.k);
// 		points_path.push_back(cv::Point((int)cline[0],(int)cline[1]));
// 		cv::circle(out, cv::Point(cline[0],cline[1]), 0.5 , cv::Scalar( 0,170,220 ), 5, 8, 0 );
// 	}
        
//     //INITIAL POSITION AND CONFIGURATION
//     cv::circle(out, cv::Point(robot_x0,robot_y0), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
//     cv::line(out, cv::Point(robot_x0,robot_y0) , cv::Point(robot_x0+radius*cos(robot_th0), robot_y0+radius*sin(robot_th0)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
//     //FINAL POSITION AND CONFIGURATION
//     cv::circle(out, cv::Point(gate_xf,gate_yf), 10 , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0 );
//     cv::line(out, cv::Point(gate_xf,gate_yf) , cv::Point(gate_xf+radius*cos(gate_thf), gate_yf+radius*sin(gate_thf)) , cv::Scalar( 255 , 255 , 255 ), 5, 8, 0);
    
	
// 	showImage("Dubins path", out+map);

//     return true;
                
// }

bool planMission(cv::Mat const & map, Map & map_object, std::vector<cv::Point> & points_path, Path & path, std::vector<int> & order)
{
	std::vector<NeighborVertices> roadmap;
	local_map = map;
    map_object.robot.pose.kappa = KMAX;

	if(!constructRoadmap(map_object, roadmap))
	{
		std::cerr << "(Critical) Failed to construct a Graph!" << std::endl;
		return false;
	}

	if(!planMissionOne(map_object, roadmap, points_path, path, order))
	{
		std::cerr << "(Critical) Failed to plan a mission one using Dijkstra and Dubins algorithms!" << std::endl;
		return false;
	}


    


	return true;
}
