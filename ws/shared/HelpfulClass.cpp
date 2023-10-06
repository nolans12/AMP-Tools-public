#include "HelpfulClass.h"
#include "helperFuncs.h"


/// @brief This function computes the grid c space given an environment and linkLengths for a 2 dof manipulator
grid computeGrid(amp::Environment2D environment, std::vector<double> linkLengths){
    double x0_min = environment.x_min;
    double x0_max = environment.x_max;
    double x1_min = environment.y_min;
    double x1_max = environment.y_max;

    double density = 100; // hardset, will be density x density grid.

// create manipulator
    Link2d robot = Link2d(Eigen::Vector2d(0,0),linkLengths); // base of (0,0) assumed
    // for plotting, assuming 0 joint angles
    amp::ManipulatorState state;
    state.push_back(0);
    state.push_back(0);

    // plot the workspace environment
    amp::Visualizer::makeFigure(environment, robot, state);
     
    // create c-space
    double x0_min_c = 0;
    double x0_max_c = 2*M_PI;
    double x1_min_c = 0;
    double x1_max_c = 2*M_PI;
    grid c(robot, environment, density, density, x0_min_c, x0_max_c, x1_min_c, x1_max_c);

    amp::DenseArray2D collisions(density,density);

    // now we need to check for collisions, loop through all cells
    for (int i = 0; i < density; i++){
        for (int j = 0; j < density; j++){
            // get the current t0 and t1 values
            double t0 = x0_min_c + i*(x0_max_c-x0_min_c)/density;
            double t1 = x1_min_c + j*(x1_max_c-x1_min_c)/density;

            // check if in collision with any one of the 3 vertices, or between!
            bool collision = c.inCollision(t0,t1);
            if (collision == true){
                c.operator()(i,j) = true;
            }
        }
    }
    return c;
}

bool grid::inCollision(double x0, double x1) const{ 
        amp::ManipulatorState state; // joint angles
        state.push_back(x0);
        state.push_back(x1);

        // get foward kinematics of all vertices, then check if collide
        Eigen::Vector2d v0 = robot.getJointLocation(state,0);
        Eigen::Vector2d v1 = robot.getJointLocation(state,1);
        Eigen::Vector2d v2 = robot.getJointLocation(state,2);

        // now, find if the line between v0 and v1 and v1 and v2 collides with any of the obstacles
        for (int i = 0; i < environment.obstacles.size(); i++){
            amp::Polygon currObs = environment.obstacles[i];
            // get vertices of the obstacle
            std::vector<Eigen::Vector2d> obsVertices = currObs.verticesCCW();
            // now, for each vertice, check if it collides with the line between v0 and v1 and v1 and v2
            for (int j = 0; j < obsVertices.size(); j++){
                // check if the line between v0 and v1 and v1 and v2 collides with the line between obsVertices[j] and obsVertices[j+1]
                if (intersect(v0,v1,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()]) || intersect(v1,v2,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()])){
                    return true; // if happens at all, return true
                }
            }
        }
        return false;
}

bool intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2){
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
    // Lines are parallel
    if (denom == 0) {
        return false;
    }
    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))/denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))/denom;
    // is the intersection along the segments
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
        return false;
    }
    return true; // isnt intersection
}

Eigen::Vector2d Link2d::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const{
    // get link lengths
    std::vector<double> link_lens = getLinkLengths();
    
    // get base location
    Eigen::Vector2d base_loc = getBaseLocation();

    // perform forward kinematics
    Eigen::MatrixXd curr(3,3); // start with identity
    curr << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

    int i = 0;
    while (i < joint_index+1){
        if (i == 0){
            curr = curr*Tmatrix(state[i], 0); // first condition
        }
        else if(i == joint_index){
            curr = curr*Tmatrix(0, link_lens[i-1]); // last condition
        }
        else{
            curr = curr*Tmatrix(state[i], link_lens[i-1]); // algorithm
        }
        i++;
    }
    // now compute final position
    Eigen::MatrixXd final = curr*Eigen::Vector3d(base_loc.x(), base_loc.y(), 1);

    return Eigen::Vector2d(final(0),final(1));
}

amp::ManipulatorState Link2d::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    // get link lengths
    std::vector<double> link_lens = getLinkLengths();
    // get base location
    Eigen::Vector2d base_loc = getBaseLocation();
    Eigen::Vector2d endLoc(end_effector_location);
    std::vector<double> link_angles;

    Eigen::Vector2d newEndLoc;
    //if we have 2 links, we can solve for the angles
    //thus, set first angle to point first link in direction of end_effector_location, so 3 links, constrains to 2 dof.
    if (link_lens.size() > 2){
        // find angle it should point too:
        double angle = atan2(endLoc.y(), endLoc.x());
        // now we solve for new end effector position we are aiming for, with 2 dof
        newEndLoc = endLoc - Eigen::Vector2d(link_lens[0]*cos(angle), link_lens[0]*sin(angle));
        // increment so that we skip the first link later
        // now perform inverse kinematics on 2dof
        double a1 = link_lens[1];
        double a2 = link_lens[2];
        double c2 = (newEndLoc.x()*newEndLoc.x() + newEndLoc.y()*newEndLoc.y() - a1*a1 - a2*a2)/(2*a1*a2);
        double s2 = sqrt(1-c2*c2);

        // check if reachable
        //if (!std::isnan(s2) && !std::isnan(c2)){
            double t1 = atan2(newEndLoc.y(), newEndLoc.x()) - atan2(a2*s2, a1+a2*c2);
            double t2 = atan2(s2, c2);
            link_angles.push_back(angle);
            link_angles.push_back(t1);
            link_angles.push_back(t2);
            return link_angles;
       // }
    }

    // for(double angle = 0; angle <= 2*M_PI; angle += 0.005){
    //     // compute new end location
    //     Eigen::Vector2d newEndLoc = endLoc - Eigen::Vector2d(link_lens[0]*cos(angle), link_lens[0]*sin(angle));
        
    //     // now perform inverse kinematics on 2dof
    //     double a1 = link_lens[1];
    //     double a2 = link_lens[2];

    //     double c2 = (newEndLoc.x()*newEndLoc.x() + newEndLoc.y()*newEndLoc.y() - a1*a1 - a2*a2)/(2*a1*a2);
    //     double s2 = sqrt(1-c2*c2);

    //     // check if reachable
    //     if (!std::isnan(s2) && !std::isnan(c2)){
    //         double t1 = atan2(newEndLoc.y(), newEndLoc.x()) - atan2(a2*s2, a1+a2*c2);
    //         double t2 = atan2(s2, c2);
    //         link_angles.push_back(angle);
    //         link_angles.push_back(t1);
    //         link_angles.push_back(t2);
    //         return link_angles;
    //     }
    // }

    // double a1 = link_lens[0+count];
    // double a2 = link_lens[1+count];

    // double c2 = (endLoc.x()*endLoc.x() + endLoc.y()*endLoc.y() - a1*a1 - a2*a2)/(2*a1*a2);
    // double s2 = sqrt(1-c2*c2);
    // double t1 = atan2(endLoc.y(), endLoc.x()) - atan2(a2*s2, a1+a2*c2);
    // double t2 = atan2(s2, c2);

    // link_angles.push_back(t1);
    // link_angles.push_back(t2);

    link_angles.push_back(0);
    link_angles.push_back(0);
    link_angles.push_back(0);
    return link_angles;
}

Eigen::MatrixXd Link2d::Tmatrix(double theta, double a) const{
    Eigen::MatrixXd T(3,3);
    T << cos(theta), -sin(theta), a,
            sin(theta), cos(theta), 0,
            0, 0, 1;
    return T;
}

std::unique_ptr<amp::GridCSpace2D> gridConstruct::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
    // create c-space, hardcoded
    double density = 100;
    double x0_min_c = 0;
    double x0_max_c = 2*M_PI;
    double x1_min_c = 0;
    double x1_max_c = 2*M_PI;

    Link2d linkTest(manipulator.getLinkLengths()); // construct new Link2d instance, so that grid can have correct arguments

    std::unique_ptr<amp::GridCSpace2D> cspace = std::make_unique<grid>(linkTest, env, density, density, x0_min_c, x0_max_c, x1_min_c, x1_max_c);

    amp::DenseArray2D collisions(density,density);

    // now we need to check for collisions, loop through all cells
    for (int i = 0; i < density; i++){
        for (int j = 0; j < density; j++){
            // get the current t0 and t1 values
            double t0 = x0_min_c + i*(x0_max_c-x0_min_c)/density;
            double t1 = x1_min_c + j*(x1_max_c-x1_min_c)/density;

            // check if in collision with any one of the 3 vertices, or between!
            bool collision = cspace->inCollision(t0,t1);
            if (collision == true){
                cspace->operator()(i,j) = true;
            }
        }
    }
    return cspace;
}