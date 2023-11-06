#include "newHelper.h"

/// @brief Returns true if two line sections intersect any obstacle in problem
bool lineFullCollision(Eigen::VectorXd p1, Eigen::VectorXd p2, amp::Problem2D problem){
    // Convert to Eigen::Vector2d
    Eigen::Vector2d p1_2d(p1[0], p1[1]);
    Eigen::Vector2d p2_2d(p2[0], p2[1]);

    // get all the obstacles
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
    // loop through all obstacles
    for (int i = 0; i < obstacles.size(); i++){
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
            // check if the line collides with any of the vertices
            if (lineCollision(p1_2d, p2_2d, vertices[j%vertices.size()], vertices[(j+1)%vertices.size()])){
                return true;
            }
        }
    }
    return false;
}

/// @brief Returns true if two line sections intersect
bool lineCollision(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2){
    double x1 = p1.x();
    double y1 = p1.y();
    double x2 = q1.x();
    double y2 = q1.y();
    double x3 = p2.x();
    double y3 = p2.y();
    double x4 = q2.x();
    double y4 = q2.y();
    // Check if none of the lines are of length 0
    if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4)) {
        return false;
    }
    double denom = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    // // Lines are parallel
    if (denom == 0) {
        return false;
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/denom;
    // is the intersection along the segments
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return false;
    }
    return true; // is intersection
}

/// @brief Returns true if a point is in collision with the environment
bool pointCollision(amp::Environment2D environment, Eigen::Vector2d point){
    std::vector<amp::Obstacle2D> obstacles = environment.obstacles;
    // loop through all obstacles
    for (int i = 0; i < obstacles.size(); i++){
        int count = 0;
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
                // check the normal angle of a line between each point
                Eigen::ParametrizedLine<double, 2> line(vertices[j], vertices[(j+1)%vertices.size()] - vertices[j]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

                if ((normal.dot(point - vertices[j]) <= 0)){ // method for finding if a point is inside polygon
                    count++;
                }
        }
        // at the end, if count is equal to the vertices size, then in collision
        if (count == vertices.size()){
            return true;
        }
    }
    return false;
}

double randomNum(double min, double max){
    // Generate a random double number between min and max, with random seeding
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(min, max);
    return distr(eng);
}

amp::Path2D pathSmooth(amp::Path2D currPath, amp::Problem2D problem){
    // amp::Visualizer::makeFigure(problem, currPath);
   
    // Create a new path
    amp::Path2D newPath;
    // set the newPath to have the same values as currPath
    newPath.waypoints = currPath.waypoints;

    double n = 100;
    // for 100 times, randomly sample i and j from 0 to n
    for (int count = 0; count < n; count++){
        double size = newPath.waypoints.size();
        int i = std::floor(randomNum(0, size));
        int j = std::floor(randomNum(0, size));
        // if i < j, swap them thus i > j
        if (i == j){
            continue;
        }
        if (i < j){
            int temp = i;
            i = j;
            j = temp;
        }
        // check if the line between i and j is in collision
        if (!lineFullCollision(newPath.waypoints[i], newPath.waypoints[j], problem)){
            // if no collision, remove inbetween nodes from newPath
            newPath.waypoints.erase(newPath.waypoints.begin() + j + 1, newPath.waypoints.begin() + i);
        }
    }
    // amp::Visualizer::makeFigure(problem, newPath);
    return newPath;
}
  
