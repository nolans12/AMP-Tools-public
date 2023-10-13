// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

// Include the header of the shared class
#include "GDAlgo.h"
#include "HelpfulClass.h"
#include "helperFuncs.h"

int main(int argc, char** argv) {

    // initiate the workspace needed to test
    std::vector<amp::Problem2D> workspaces;
    workspaces.push_back(amp::HW5::getWorkspace1());
    workspaces.push_back(amp::HW2::getWorkspace1());
    workspaces.push_back(amp::HW2::getWorkspace2());

// hyper parameters:
    // attractive
    std::vector<double> d_star; // switches from quadratic to linear
    d_star.push_back(0.1);
    d_star.push_back(0.1);
    d_star.push_back(0.1);

    std::vector<double> zeta; // amplitude of attractive potential
    zeta.push_back(1);
    zeta.push_back(1);
    zeta.push_back(1);

    // repulsive
    std::vector<double> q_star; // switch from repulsing to none
    q_star.push_back(0.25);
    q_star.push_back(0.25);
    q_star.push_back(0.25);

    std::vector<double> eta; // amplitude of repulsive potential
    eta.push_back(0.1);
    eta.push_back(0.1);
    eta.push_back(0.1);

    // goal detection
    double epsilon = 0.01; // distance req to be considered at goal
    double step = 0.25; // step size for gradient decent
    double maxStep = 1000; // max number of steps for gradient decent before returns, to prevent infinite loops at local min

    // obstacle detection
    double numRay = 150; // number of rays to cast from current point

    for (int i = 0; i < d_star.size(); i++){

        GDAlgo gd(d_star[i], zeta[i], q_star[i], eta[i], epsilon, step, maxStep, numRay); // hyper parameters for gradient decent

        amp::Path2D currPath = gd.plan(workspaces[i]);

        // visualize the paths
        amp::Visualizer::makeFigure(workspaces[i], currPath);
        bool currSuccess = amp::HW5::check(currPath, workspaces[i]);
        LOG("Found valid solution found?: " << (currSuccess ? "Yes!" : "No :("));
        LOG("path length: " << currPath.length());
    }
    amp::Visualizer::showFigures();

    // get to pass for workspace 1 and 2:
    GDAlgo gd(d_star[1], zeta[1], q_star[1], eta[1], epsilon, step, maxStep, numRay); 
    amp::HW5::grade(gd, "nolan.stevenson@colorado.edu", argc, argv);

    return 0;
}