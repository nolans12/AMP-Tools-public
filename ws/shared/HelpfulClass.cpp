#include "HelpfulClass.h"
#include "helperFuncs.h"


/// @brief This function computes the grid c space given an environment and linkLengths for a 2 dof manipulator
/// @param environment 
/// @param linkLengths 
void computeGrid(amp::Environment2D environment, std::vector<double> linkLengths){
    double x0_min = environment.x_min;
    double x0_max = environment.x_max;
    double x1_min = environment.y_min;
    double x1_max = environment.y_max;

    double density = 100; // hardset? will be density x density grid.

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

    // now we need to check for collisions
    // loop through all cells
    for (int i = 0; i < density; i++){
        for (int j = 0; j < density; j++){
            // get the current x0 and x1 values
            double x0 = x0_min_c + i*(x0_max_c-x0_min_c)/density;
            double x1 = x1_min_c + j*(x1_max_c-x1_min_c)/density;
            // std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;

            // check if in collision with any one of the 3 vertices, or between!
            bool collision = c.inCollision(x0,x1);
            if (collision){
                std::cout << "Detected collision at t0: " << x0 << " t1: " << x1 << std::endl;
            }
        }
    }

    amp::Visualizer::makeFigure(c);
    amp::Visualizer::showFigures();
}

bool grid::inCollision(double x0, double x1) const{ 
    
        amp::ManipulatorState state; // joint angles
        state.push_back(x0);
        state.push_back(x1);
        // output the joint locations
        //std::cout << "at t0: " << x0 << " at t1: " << x1 << std::endl;

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
                if (collisionDetection(v0,v1,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()]) || collisionDetection(v1,v2,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()])){
                    return true; // if happens at all, return true
                }
            }
        }
        return false;
}

bool onSegment(Eigen::Vector2d p, Eigen::Vector2d q, Eigen::Vector2d r) {
  if (q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
      q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y()))
        return true;
  return false;
}

/// @brief This function takes in four vertices and detects if the lines created by them intersect
bool collisionDetection(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2) {
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);
  
  if (o1 != o2 && o3 != o4)
    return true;
  
  //p1 p2 q1 are colinear. Check whether q1 is on p1->p2
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;
  if (o4 == 0 && onSegment(p2, q1, q2))
    return true;
  return false;
}

int orientation(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c){
    int val = (b.y()-a.y())*(c.x()-b.x()) - (b.x()-a.x())*(c.y()-b.y());
    if (val == 0){
        return 0;
    }
    else if (val > 0){
        return 1;
    }
    else{
        return 2;
    }
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
    //std::cout << curr << std::endl;
    // now compute final position
    Eigen::MatrixXd final = curr*Eigen::Vector3d(base_loc.x(), base_loc.y(), 1);
    // output final[0] and final[1]
    //std::cout << "joint index: " << joint_index << ", x: " << final(0) << " y: " << final(1) << std::endl;

    return Eigen::Vector2d(final(0),final(1));
}

amp::ManipulatorState Link2d::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    // get link lengths
    std::vector<double> link_lens = getLinkLengths();
    // get base location
    Eigen::Vector2d base_loc = getBaseLocation();
    Eigen::Vector2d endLoc(end_effector_location);
    std::vector<double> link_angles;
    int count = 0;
    // if we have 2 links, we can solve for the angles
    // thus, set first angle to 0, if 3 links, to constain to 2 dof.
    if (link_lens.size() == 3){
        link_angles.push_back(0); // make it 0 
        // now we solve for new end effector position we are aiming for, with 2 dof
        endLoc = endLoc - Eigen::Vector2d(link_lens[0], 0);
        count++; // increment so that we skip the first link later
    }
    //std::cout << "New end location: x: " << endLoc.x() << " y: " << endLoc.y() << std::endl;

    double a1 = link_lens[0+count];
    double a2 = link_lens[1+count];
    //std::cout << "a1: " << a1 << " a2: " << a2 << std::endl;

    double c2 = (endLoc.x()*endLoc.x() + endLoc.y()*endLoc.y() - a1*a1 - a2*a2)/(2*a1*a2);
    double s2 = sqrt(1-c2*c2);
    double t1 = atan2(endLoc.y(), endLoc.x()) - atan2(a2*s2, a1+a2*c2);
    double t2 = atan2(s2, c2);

    link_angles.push_back(t1);
    link_angles.push_back(t2);

    
    return link_angles;
}

Eigen::MatrixXd Link2d::Tmatrix(double theta, double a) const{
    Eigen::MatrixXd T(3,3);
    T << cos(theta), -sin(theta), a,
            sin(theta), cos(theta), 0,
            0, 0, 1;
    return T;
}
