// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"
#include "bug1.h"
#include "bug2.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    bug1 buggy1;
    bug2 buggy2;
    
    // check problem 1
    Problem2D problem = HW2::getWorkspace1();
    {
        // bug 1
        amp::Path2D pathBug1 = buggy1.plan(problem);
        bool successBug1 = HW2::check(pathBug1, problem);
        LOG("Found valid solution to workspace 1 using bug 1?: " << (successBug1 ? "Yes!" : "No :("));
        // bug 2
        amp::Path2D pathBug2 = buggy2.plan(problem);
        bool successBug2 = HW2::check(pathBug2, problem);
        LOG("Found valid solution to workspace 1 using bug 2?: " << (successBug2 ? "Yes!" : "No :("));
        
        // Visualize the path and environment
        Visualizer::makeFigure(problem, pathBug1);
        Visualizer::makeFigure(problem, pathBug2);
    }

    // check problem 2
    problem = HW2::getWorkspace2();
    {
        // // bug 1
        amp::Path2D pathBug1 = buggy1.plan(problem);
        bool successBug1 = HW2::check(pathBug1, problem);
        LOG("Found valid solution to workspace 2 using bug 1?: " << (successBug1 ? "Yes!" : "No :("));
        // // bug 2
        amp::Path2D pathBug2 = buggy2.plan(problem);
        bool successBug2 = HW2::check(pathBug2, problem);
        LOG("Found valid solution to workspace 2 using bug 2?: " << (successBug2 ? "Yes!" : "No :("));

        Visualizer::makeFigure(problem, pathBug1);
        Visualizer::makeFigure(problem, pathBug2);
    }

    //Let's get CRAZY generate a random environment and test your algorithm
    {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = HW2::generateAndCheck(buggy2, path, random_prob, collision_points); // replace with buggy1
        LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));
        LOG("path length: " << path.length());

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::showFigures();
    
    //HW2::grade<bug1>("nolan.stevenson@colorado.edu", argc, argv); // will abt 15 mins bc I put step size 0.001 to be certain
    //HW2::grade<bug2>("nolan.stevenson@colorado.edu", argc, argv);

    return 0;
}