#include "GDAlgo.h"

amp::Path2D GDAlgo::plan(const amp::Problem2D& problem){
    // initiate the path
    amp::Path2D path;

    // now calculate the path using gradient decent
    // first get the start and goal
    Eigen::Vector2d start = problem.q_init;
    Eigen::Vector2d goal = problem.q_goal;
        // output the start and goal vertices
        std::cout << "Start point: x: " << start.x() << ", y: " << start.y() << std::endl;
        std::cout << "Goal point: x: " << goal.x() << ", y: " << goal.y() << std::endl;

    // now calculate the path
    path.waypoints.push_back(start); // start point
    int count = 0;
    while (count < maxStep){ // iterate until goal or max steps reached

    // get current point
        Eigen::Vector2d curr = path.waypoints.back();
            // std::cout << "Point: x: " << curr.x() << ", y: " << curr.y() << std::endl;

    // check if goal reached
        if ((curr - goal).norm() <= epsilon){ // if the current point is within epsilon of the goal
            path.waypoints.push_back(goal);
            return path;
        }

    //  calculate current gradient decent vector
        Eigen::Vector2d grad = attract(curr, goal) + repulse(curr, problem);

            // std::cout << "Direction is: x: " << -grad.x() << ", y: " << -grad.y() << std::endl;

    //  calculate next point
        Eigen::Vector2d next = curr - step * grad; // gradient decent goes '-' gradient

    //  store in path
        path.waypoints.push_back(next);
        count++;
    }
    return path;
}

Eigen::Vector2d GDAlgo::attract(Eigen::Vector2d curr, Eigen::Vector2d goal){
    // first calculate the distance between the current position and the goal position
    double dist = (curr - goal).norm();

    if (dist <= d_star){ // if the distance is less than d_star, very close to goal
        return zeta * (curr - goal);
    }
    else{ // if the distance is greater than d_star, far from goal
        return (d_star * zeta * (curr - goal)) / dist;
    }
}

Eigen::Vector2d GDAlgo::repulse(Eigen::Vector2d curr, const amp::Problem2D& problem){
    // first calculate the distance to the closest obstacle

    Eigen::Vector2d closestVec = rayDetect(problem, curr) - curr;
    double dist = closestVec.norm();

    if (dist <= q_star){ // if the distance is less than q_star, very close to obstacle
        return eta * (1 / q_star - 1 / dist) * (1 / (dist * dist)) * -closestVec;
    }
    else{ // if the distance is greater than q_star, far from obstacle
        return Eigen::Vector2d(0, 0);
    }
}

Eigen::Vector2d GDAlgo::rayDetect(const amp::Problem2D& problem, const Eigen::Vector2d& point){
    // use ray casting to detect the closest obstacle
    
    std::vector<amp::Polygon> obstacles = problem.obstacles;
    Eigen::Vector2d currMin(INT16_MAX, INT16_MAX); 

    for (int i = 0; i < obstacles.size(); i++){
        // get the vertices of the obstacle
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW();

        // now iterate through the vertices
        for (int j = 0; j < vertices.size(); j++){
            // get the current vertex and the next vertex
            Eigen::Vector2d curr = vertices[j];
            Eigen::Vector2d next = vertices[(j + 1) % vertices.size()];

            for (int k = 0; k < numRay; k++){
                // angle of ray
                double angle = (2 * M_PI * k) / numRay;
                // cast a ray from point in direction of angle, cast to 1000 units
                Eigen::Vector2d ray = point + 1000 * Eigen::Vector2d(cos(angle), sin(angle));

                // now check if the point-ray intersects the line segement
                Eigen::Vector2d intersection = intersect(point, ray, curr, next);

                // now check if the intersection is within the line segment
                if (intersection.x() != INT16_MAX && intersection.y() != INT16_MAX){ // if the intersection is true
                    if ((intersection-point).norm() < (currMin-point).norm()){
                        currMin = intersection;
                    }
                }
            }
        }
    }
    return currMin;
}

Eigen::Vector2d GDAlgo::intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2){
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
        return Eigen::Vector2d(INT16_MAX, INT16_MAX);
    }
    double denom = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    // Lines are parallel
    if (denom == 0) {
        return Eigen::Vector2d(INT16_MAX, INT16_MAX);
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/denom;
    // is the intersection along the segments
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return Eigen::Vector2d(INT16_MAX, INT16_MAX);
    }
    return Eigen::Vector2d(x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)); // is intersection, return location
}