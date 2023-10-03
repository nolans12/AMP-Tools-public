#include "helperFuncs.h"

/// @brief This function takes in a theta rotation and a translation vector and returns the transformation matrix
Eigen::MatrixXd Tmatrix(double theta, double a){
    Eigen::MatrixXd T(3,3);
    T << cos(theta), -sin(theta), a,
              sin(theta), cos(theta), 0,
              0, 0, 1;

    return T;
}

/// @brief This function sorts the vertices of the polygon by polar angle, aka ccw order, from farthest left point
std::vector<Eigen::Vector2d> sort_verts(amp::Polygon poly){
    // get the vertices of the polygon
    std::vector<Eigen::Vector2d> vertices = poly.verticesCCW();
    // find the mean x and y of the polygon
    double meanX = 0;
    double meanY = 0;
    for (int i = 0; i < vertices.size(); i++){
        meanX += vertices[i].x();
        meanY += vertices[i].y();
    }
    meanX = meanX/vertices.size();
    meanY = meanY/vertices.size();

    // create a vector to store the polar angles
    std::vector<double> polarAngles;

    // loop through all vertices
    for (int i = 0; i < vertices.size(); i++){
        // get the polar angle
        double angle = atan2(vertices[i].y() - meanY, vertices[i].x() - meanX);
        // store the polar angle
        polarAngles.push_back(angle);
    }

    // sort the polar angles
    std::sort(polarAngles.begin(), polarAngles.end());

    // create a vector to store the sorted vertices
    std::vector<Eigen::Vector2d> sortedVertices;
    // loop through all polar angles
    for (int i = 0; i < polarAngles.size(); i++){
        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
            // get the current polar angle of the vertices
            double angle = atan2(vertices[j].y() - meanY, vertices[j].x() - meanX);
            // check if the polar angle is equal to the sorted polar angle
            if (angle == polarAngles[i]){
                // add the vertex to the sorted vertices
                sortedVertices.push_back(vertices[j]);
            }
        }
    }
    // return the sorted vertices
    return sortedVertices;
}

/// @brief Calculates the cross product of two vectors
double cross(Eigen::Vector2d a, Eigen::Vector2d b){
    return a.x()*b.y() - a.y()*b.x();
}

/// @brief Calculates and returns the minkowski sum of the two polygons
amp::Polygon minkowski(amp::Polygon robot, amp::Polygon obstacle){
    std::vector<Eigen::Vector2d> minkowVerts;
    // sort by polar angle
    std::vector<Eigen::Vector2d> p1 = sort_verts(robot);
    std::vector<Eigen::Vector2d> p2 = sort_verts(obstacle); 

    int i = 0;
    int j = 0;
    double crossProd;
    while (i < p1.size() || j < p2.size()){ 
        minkowVerts.push_back(p1[i%p1.size()] + p2[j%p2.size()]);
        crossProd = cross(p1[(i+1)%p1.size()] - p1[i%p1.size()], p2[(j+1)%p2.size()] - p2[j%p2.size()]);
        // find lowest polar angle
        if (crossProd >= 0){
            i++;
        }
        if (crossProd <= 0){
            j++;
        }
    }
    amp::Polygon minkowPoly(minkowVerts);
    return minkowPoly;
}

/// @brief This funciton takes in an obstacle and returns the negative of the vertices
amp::Polygon negativeVert(amp::Polygon posObj, Eigen::Vector2d refPoint){
    // declare empty obstacle
    amp::Obstacle2D negObj;
    // get the vertices of the obstacle
    std::vector<Eigen::Vector2d> vertices = posObj.verticesCCW();
    // loop through all vertices
    for (int i = 0; i < vertices.size(); i++){
        // negate the vertices
        if (vertices[i] != refPoint){
            vertices[i] = -(vertices[i]-refPoint) + refPoint; // - w/ respect to ref point
        }
    }
    // set the vertices of the negative obstacle
    negObj = amp::Polygon(vertices);
    // return the negative obstacle
    return negObj;
}

/// @brief This funciton rotates a polygon by an angle theta, about reference point refPoint
amp::Polygon rotatePolygon(amp::Polygon polyOrig, Eigen::Vector2d refPoint, double angle){
    // first, subtract ref point from all vertices in polygon
    std::vector<Eigen::Vector2d> vertices = polyOrig.verticesCCW();
    for (int i = 0; i < vertices.size(); i++){
        vertices[i] = vertices[i] - refPoint;
    }
    // now, rotate all vertices by angle theta
    for (int i = 0; i < vertices.size(); i++){
        vertices[i] = Eigen::Rotation2D(angle)*vertices[i];
    }
    // now add the ref point back to all vertices
    for (int i = 0; i < vertices.size(); i++){
        vertices[i] = vertices[i] + refPoint;
    }
    // return the resulting polygon
    return amp::Polygon(vertices);
}

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