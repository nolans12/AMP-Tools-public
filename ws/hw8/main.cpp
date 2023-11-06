#include "AMPCore.h"
#include "hw/HW8.h"
#include "Eigen/Geometry"
#include "RRT.h"
#include "newHelper.h"

int main(int argc, char** argv) {

 // Which cases to run
    struct cases {
        bool log;
        std::string name;
    };
    std::vector<cases> options = {{true, "Exercise 1.b."}, {false, "Exercise 1.c."}, {false, "Exercise 1.d."}, {false, "Exercise 2.b."}, {false, "Exercise 2.c."}, {false, "Exercise 2.d."}};


// Exercise 1
    
// 1.b

    if (options[0].log){
        // Get the workspace for m = 2 robots
        amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1(2);

        // Set the parameters
        double n = 7500;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;

        centralRRT cRRT(n, r, p_goal, epsilon);

        // Solve the problem
        amp::MultiAgentPath2D path = cRRT.plan(prob);

        // plot the workspace
        // amp::Visualizer::makeFigure(prob);
         amp::Visualizer::makeFigure(prob, path);
    }

// Exercise 2



    amp::Visualizer::showFigures();
    return 0;
}
