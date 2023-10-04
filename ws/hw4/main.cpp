// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "helperFuncs.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

std::cout << "/////// EXERCISE 1 ////////" << std::endl;
    amp::Polygon obstacle = amp::HW4::getEx1TriangleObstacle();
    amp::Polygon robot = amp::HW4::getEx1TriangleObstacle();

    // get the vertices of the robot
    std::vector<Eigen::Vector2d> vertices = robot.verticesCCW();
    // get the reference point
    Eigen::Vector2d refPoint = vertices[0]; // told to use lower left

    // test case
    // std::vector<Eigen::Vector2d> vRob;
    // vRob.push_back(Eigen::Vector2d(0,1));
    // vRob.push_back(Eigen::Vector2d(2,1));
    // vRob.push_back(Eigen::Vector2d(2,3));
    // vRob.push_back(Eigen::Vector2d(0,3));
    // amp::Polygon obstacle(vRob);

    amp::Polygon negRobot = negativeVert(robot, refPoint); // told to chose lower left vertex as reference point
    //output the original vertices of robot, including x and y for each line
    vertices = negRobot.verticesCCW();
    std::cout << "Robot: " << std::endl;
    for (int i = 0; i < vertices.size(); i++){
        std::cout << "x: " << vertices[i].x() << " y: " << vertices[i].y() << std::endl;
    }
    // output all vertices of Obstacle, including x and y for each line
    vertices = obstacle.verticesCCW();
    std::cout << "Obstacle: " << std::endl;
    for (int i = 0; i < vertices.size(); i++){
        std::cout << "x: " << vertices[i].x() << " y: " << vertices[i].y() << std::endl;
    }
    // do minkowski sum on negative robot + obstacle
    amp::Polygon cObst = minkowski(negRobot, obstacle);

    // now plot
    std::vector<Polygon> origPolygons;
    std::vector<Polygon> resultPolygons;
    // plot the polygons, original
    origPolygons.push_back(obstacle);
    origPolygons.push_back(negRobot);
    std::vector<std::string> labels;
    labels.push_back("obstacle");
    labels.push_back("negative robot");
    // result
    resultPolygons.push_back(cObst);
    // plot
    amp::Visualizer::makeFigure(origPolygons, labels, true);
    amp::Visualizer::makeFigure(resultPolygons, true);
    //amp::Visualizer::showFigures();

// 1B
    // use 12 equally spaced rotation angles, rotating about ref point of robot, (0,0) or [0]
    // get the angles
    std::vector<double> angles;
    for (int i = 0; i < 12; i++){
        angles.push_back(i*M_PI/6);
    }
    // rotate the robot
    std::vector<Polygon> rotatedRobots;
    for (int i = 0; i < angles.size(); i++){
        rotatedRobots.push_back(rotatePolygon(robot, refPoint, angles[i]));
    }
    // plot the rotated robots
    amp::Visualizer::makeFigure(rotatedRobots, true);
    //amp::Visualizer::showFigures();

    // now perform minkowski difference on all
    std::vector<Polygon> negRotatedRobots;
    std::vector<Polygon> minkowskiRobots;
    for (int i = 0; i < angles.size(); i++){
        // negate all robots around ref point
        negRotatedRobots.push_back(negativeVert(rotatedRobots[i], refPoint));
        minkowskiRobots.push_back(minkowski(negRotatedRobots[i], obstacle));
    }
    // plot the minkowski robots
    // add a heights going from 1 to 360 to be able to visualize results
    std::vector<double> heights;
    for (int i = 0; i < angles.size(); i++){
        heights.push_back(i*360/12);
    }
    amp::Visualizer::makeFigure(minkowskiRobots, heights);
    //amp::Visualizer::showFigures();


std::cout << "/////// EXERCISE 2 ////////" << std::endl;

// 2a
// inputs:
std::cout << "Forward Kinematics: " << std::endl;
    double a1 = 0.5;
    double a2 = 1;
    double a3 = 0.5;
    double t1 = M_PI/6;
    double t2 = M_PI/3;
    double t3 = 7*M_PI/4;
    Eigen::Vector2d base(0,0);

    std::vector<double> link_lengths;
    link_lengths.push_back(a1);
    link_lengths.push_back(a2);
    link_lengths.push_back(a3);
    std::vector<double> link_angles;
    link_angles.push_back(t1);
    link_angles.push_back(t2);
    link_angles.push_back(t3);

    // make the link manipulator
    Link2d manipulator(base, link_lengths);
    
    // perform forward kinematics
    manipulator.getJointLocation(link_angles, 0);

    // plot
    amp::Visualizer::makeFigure(manipulator, link_angles);
    //amp::Visualizer::showFigures();
//2b
std::cout << "Inverse Kinematics: " << std::endl;
    // perform inverse kinematics
    Eigen::Vector2d end_effector(2, 0);
    a1 = 1;
    a2 = 0.5;
    a3 = 1;
    std::vector<double> link_lengths2;
    link_lengths2.push_back(a1);
    link_lengths2.push_back(a2);
    link_lengths2.push_back(a3);
    Link2d manipulator2(base, link_lengths2);

    std::vector<double> ik_angles = manipulator2.getConfigurationFromIK(end_effector);
    // output all ik_angles
    for (int i = 0; i < ik_angles.size(); i++){
        std::cout << "angle " << i << ": " << ik_angles[i] << std::endl;
    }
    // plot
    amp::Visualizer::makeFigure(manipulator2, ik_angles);
    //amp::Visualizer::showFigures();

std::cout << "/////// EXERCISE 3 ////////" << std::endl;
    // create a c-space constructor for 2-link manipulator
    // create manipulator

    // Link2d linkRobot = Link2d(Eigen::Vector2d(0,0),link_lengths3); // base of (0,0) 2 lengths of 1.

    // // get environment
    // amp::Environment2D partA = amp::HW4::getEx3Workspace1();

    // // create empty state
    // amp::ManipulatorState state; // joint angles
    // state.push_back(0);
    // state.push_back(0);

    // // plot workspace environment
    // amp::Visualizer::makeFigure(partA, linkRobot, state);

    std::vector<double> links;
    links.push_back(1);
    links.push_back(1);
    computeGrid(amp::HW4::getEx3Workspace1(), links);

    // // create c-space
    // // we will use x0 and x1 min and max as 0 and 2pi, as they are rotating link manipulators
    // double x0_min = 0;
    // double x0_max = M_PI*2;
    // double x1_min = 0;
    // double x1_max = M_PI*2;
    // double density = 100;

    // grid c(density, density, x0_min, x0_max, x1_min, x1_max);
    // //std::cout << c.size().first << " " << c.size().second << std::endl;

    // amp::DenseArray2D collisions(density,density);

    // // now we need to check for collisions
    // // loop through all cells
    // for (int i = 0; i < density; i++){
    //     for (int j = 0; j < density; j++){
    //         // get the current x0 and x1 values
    //         double x0 = x0_min + i*(x0_max-x0_min)/density;
    //         double x1 = x1_min + j*(x1_max-x1_min)/density;
    //         // std::cout << "x0: " << x0 << " x1: " << x1 << std::endl;

    //         // check if in collision
    //         bool collision = c.inCollision(x0,x1);
           
    //     }
    // }


    // // try plotitng
    // //amp::Visualizer::makeFigure(c);
    // amp::Visualizer::showFigures();

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}