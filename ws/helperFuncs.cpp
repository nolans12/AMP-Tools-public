#include "helperFuncs.h"

/// @brief takes in two vectors and outputs the direction between the two
/// @param start 
/// @param end 
/// @return 
Eigen::Vector2d getDirectionVector(Eigen::Vector2d start, Eigen::Vector2d end){
    // now take find the direction vector between the two points
    // create a vector to store the direction vector    
    Eigen::Vector2d directionVector;
    // find the direction vector
    directionVector = (end - start).normalized();
    // return the direction vector
    return directionVector;
}

// /// @brief This function takes in obstacles and a position, if the position is in any obstacles, return that obstacle index, else return -1
// /// @param obstacles 
// /// @param position 
// /// @return 
int obstacleIndex(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point){
    std::vector<int> hitIndices; // vector to store the indices of the obstacles that the point is in
    // loop through all obstacles
    for (int i = 0; i < obstacles.size(); i++){ 
        int count = 0;
        int extra = 0;
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle

        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
            if ((vertices[(j+1)%vertices.size()] - vertices[j]).norm() == 0){ // this is to fix for randomized workspace where there are duplicate vertices, skips them
                extra++;
            }else{
                // check the normal angle of a line between each point
                Eigen::ParametrizedLine<double, 2> line(vertices[j], vertices[(j+1)%vertices.size()] - vertices[j]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

                if ((normal.dot(point - vertices[j]) <= 0)){ // method for finding if a point is inside polygon
                    count++;
                }
            }
        }
        // at the end, check the count to see if it number of counts is = number of vertices
        if (count == (vertices.size()-extra)){;
            //return i;
            // store in a vector
            hitIndices.push_back(i);
        }
    }
    if (hitIndices.size() > 0){
        int random = rand() % hitIndices.size();
        return hitIndices[random];
    }
    return -1; // if no collision, return -1
}

/// @brief This function takes in 4 vertices and returns the intersection of the two lines
/// @param  
/// @param  
/// @param  
/// @param  
/// @return 
Eigen::Vector2d intersectionPt(Eigen::Vector2d curr, Eigen::Vector2d next, Eigen::Vector2d right, Eigen::Vector2d left){
    Eigen::Vector2d intersectionPoint;
    Eigen::Hyperplane<double,2> currPlane;
    Eigen::Hyperplane<double,2> vertPlane;  

    if (next == right || next == left){ // checks if one of the verts is the nextPos
        return next;
    }

    // create the planes
    currPlane = Eigen::Hyperplane<double,2>::Through(curr, next);
    vertPlane = Eigen::Hyperplane<double,2>::Through(right, left);
    intersectionPoint = currPlane.intersection(vertPlane); // find the intersection point of the two planes
    
    return intersectionPoint;
}