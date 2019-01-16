
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <tuple> 

#include "collision_detection.hpp"

// Define Infinite (Using INT_MAX caused overflow problems) 
#define INF 10000 

using namespace std;
using namespace cv;


//Debugging Example
//Point p = {420,470};

bool collision = false;


// Given three colinear points p, q, r, the function checks if 
// point q lies on line segment 'pr' 
bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&  q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
        return true; 
    else
		return false; 
} 

// To find orientation of ordered triplet (p, q, r). 
// The function returns following values 
// 0 --> p, q and r are colinear 
// 1 --> Clockwise 
// 2 --> Counterclockwise 
int orientation(Point p, Point q, Point r) 
{ 
    int val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 
  
// The function that returns true if line segment 'p1q1' 
// and 'p2q2' intersect. 
bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
} 

// Returns true if the point p lies inside the polygon[] with n vertices 
bool isInside(Point polygon[], int n, Point p) 
{ 
    // There must be at least 3 vertices in polygon[] 
    if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinite 
    Point extreme = {INF, p.y}; 
  
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    do
    { 
        int next = (i+1)%n; 
  
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0) 
               return onSegment(polygon[i], p, polygon[next]); 
  
            count++; 
        } 
        i = next; 
    } while (i != 0); 
  
    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
} 
  

//Check if a line segment collides with the Obstacles in the map
//Using the LineIteration Function to loop through all the points contained in the Line  
   

void line_colliding( cv::Mat img , cv::Point ini_p , cv::Point fin_p , int &line_collision , int &x_out , int &y_out ) 
{
	
	std::cout << "ANALYZING THE LINEAR SEGMENT FROM: (" << ini_p.x << " , " << ini_p.y << ") TO (" << fin_p.x << " , " << fin_p.y << " )" << std::endl;  
	
	cv::LineIterator it( img , ini_p , fin_p , 8 );
	
	for(int i = 0; i < it.count; i++, it++)
	{
        Point actual_p = it.pos();
        
        //std::cout << "The actual point is (" << it.pos().x << " ,  " << it.pos().y << ")" << std::endl;
        
        if (collision_detection(it.pos()) == 1) 
        { 
			line_collision = 1;
			x_out = it.pos().x;
			y_out = it.pos().y;
			break;
		}
		
    }
}


//Point Comparison:
			
			// 1) Draw a horizontal line to the right of each point and extend it to infinity

			// 1) Count the number of times the line intersects with polygon edges.

			// 2) A point is inside the polygon if either count of intersections is odd or
			//    point lies on an edge of polygon.  If none of the conditions is true, then 
			//    point lies outside
			

float collision_detection( cv::Point p )
{
    
    int type;
    int id;
    int corners;
    
    int PolID = 0;
    
    ifstream file;
    file.open("data/output/output.txt");
    
    
    std::cout << "For the Point : ( " << p.x << " , " << p.y << ") " << endl ;
    
    
    while (!file.eof() && collision == false)
	{
		std::vector<cv::Point> Obstacle;
		
		file >> type >> id >> corners ;
		
		if (type == 2){ file.ignore(1000,'\n'); }
		
		else if (corners < 3) { file.ignore(1000,'\n'); std::cout << " ERROR : NOT ENOUGH CORNERS TO FORM A SHAPE " << endl; } 
		
		else if (collision == false && corners >=3 )
		{
		    std::cout << " Obstacle N. " << PolID << " Corners : " << corners ;
		    
			for( int i= 0 ; i < corners ; i++ )
		
			{	
				int x , y ;
			
				file >> x >> y ;
				
				cv::Point corner(x,y);   
				 
				Obstacle.push_back(corner);
				
				//Save the different obstacles with their own points
				
				if (collision == 1) break;
			
				if (i==0) std::cout <<  " Corners:  P" << i << ": (" << x  <<  "," << y << ")" ;	
				else      std::cout << ", P" << i << ": (" << x  <<  "," << y << ")" ;
			}
				
			if (Obstacle.size() == 3)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! " : cout << " ---No Collision ---";
			}
			else if (Obstacle.size() == 4)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y} ,  {Obstacle[3].x , Obstacle[3].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! "  : cout << " ---No Collision ---";
			}
			else if (Obstacle.size() == 5)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y} ,  {Obstacle[3].x , Obstacle[3].y}, {Obstacle[4].x , Obstacle[4].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! ": cout << " ---No Collision ---";
			}
			else if (Obstacle.size() == 6)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y} ,  {Obstacle[3].x , Obstacle[3].y}, {Obstacle[4].x , Obstacle[4].y}, {Obstacle[5].x , Obstacle[5].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! ": cout << " ---No Collision ---";
			}
			else if (Obstacle.size() == 7)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y} ,  {Obstacle[3].x , Obstacle[3].y}, {Obstacle[4].x , Obstacle[4].y}, {Obstacle[5].x , Obstacle[5].y}, {Obstacle[6].x , Obstacle[6].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! ": cout << " ---No Collision ---";
			}
			else if (Obstacle.size() == 8)
			{
				Point polygon[] = {{Obstacle[0].x, Obstacle[0].y}, {Obstacle[1].x , Obstacle[1].y},  {Obstacle[2].x , Obstacle[2].y} ,  {Obstacle[3].x , Obstacle[3].y}, {Obstacle[4].x , Obstacle[4].y}, {Obstacle[5].x , Obstacle[5].y}, {Obstacle[6].x , Obstacle[6].y}, {Obstacle[7].x , Obstacle[7].y}};
				isInside(polygon , Obstacle.size() , p)? collision =true , cout << "   Collision! ": cout << " ---No Collision ---";
			}
			
			std::cout << endl ;
			
			if (collision == true) break;
			
			else PolID++; //next Polygon 
		}
		if (collision == true) break;	
	} 
	
	cout << endl ;
	
	if(collision == true) {cout << "\n Collision Detected in Obstacle " << PolID << " , Coordinate (" << p.x << "," <<  p.y <<  ") \n\n" <<endl ; return 1;} //Add the Return for giving the Coordinates of the collision for Remapping purposes 
	else                  {return 0;}
}


