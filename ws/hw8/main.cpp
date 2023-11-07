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
    std::vector<cases> options = {{false, "Exercise 1.b."}, {false, "Exercise 1.c/d."}, {true, "Exercise 2.b."}, {true, "Exercise 2.c/d"}};


// Exercise 1
    
// 1.b
    if (options[0].log){
        // Get the workspace for m = 2 robots

        double m = 4;
        amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1(m);

        // Set the parameters
        double n = 7500;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;

        centralRRT cRRT(n, r, p_goal, epsilon);

        // Solve the problem
        amp::MultiAgentPath2D path = cRRT.plan(prob);

        // Check the solution
        bool valid = amp::HW8::check(path, prob);

        // plot the workspace
        // amp::Visualizer::makeFigure(prob);
         amp::Visualizer::makeFigure(prob, path);
    }

// 1.c/d
    if (options[1].log){
        // Run a benchmark using 100 runs of m = 2 to compare ocmpuation time and size of the tree
        // Show result using boxplot
        
        // std::vector<std::string> labels = {"n=200, r=0.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};
        std::list<std::vector<double>> success_list;
        std::list<std::vector<double>> treeSize_list;
        std::list<std::vector<double>> computationTime_list;
        std::vector<std::string> labels = {"n=7500, r=0.5, p_goal = 0.05, epsilon = 0.25"};
        double success_amount;
        std::vector<double> treeSize_vec;
        std::vector<double> computationTime_vec;
       
        double n = 7500;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;
        double m = 2;

        // Get problem
        amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1(m);

        for (int i = 0; i < 100; i++){

            // run 100 iterations
            centralRRT cRRT(n, r, p_goal, epsilon);
            std::tuple<bool, double, double> result = cRRT.planCompare(prob);

            // Store values in vectors
            if (std::get<0>(result)){
                    success_amount = success_amount + 1;
            }
            if (std::get<0>(result)){
                treeSize_vec.push_back(cRRT.getTreeSize());
            }
            computationTime_vec.push_back(std::get<1>(result)/1000);
        }

        success_list.push_back({success_amount});
        treeSize_list.push_back(treeSize_vec);
        computationTime_list.push_back(computationTime_vec);

        // Plot
        amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 1.c., m = 2", "Hyper Parameters", "Number of Successes");
        amp::Visualizer::makeBoxPlot(treeSize_list, labels, "Excersie 1.c., m = 2", "Hyper Parameters", "Size of Tree");
        amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 1.c., m = 2", "Hyper Parameters", "Computation Time (milliseconds)");
    }

// Exercise 2
    if (options[2].log){

        double m = 6;
        amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1(m);

        // Set the parameters
        double n = 7500;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;

        decentralRRT dRRT(n, r, p_goal, epsilon);

        // Solve the problem
        amp::MultiAgentPath2D path = dRRT.plan(prob);

        // Check the solution
        bool valid = amp::HW8::check(path, prob);

        // plot the workspace
        // amp::Visualizer::makeFigure(prob);
         amp::Visualizer::makeFigure(prob, path);

    }

    if (options[3].log){
        // Run a benchmark using 100 runs of m = 2 to compare ocmpuation time and size of the tree
        // Show result using boxplot
        
        // std::vector<std::string> labels = {"n=200, r=0.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};
        std::list<std::vector<double>> success_list;
        std::list<std::vector<double>> computationTime_list;
        std::vector<std::string> labels = {"n=7500, r=0.5, p_goal = 0.05, epsilon = 0.25"};
        double success_amount;
        std::vector<double> computationTime_vec;
       
        double n = 7500;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;
        double m = 6;

        // Get problem
        amp::MultiAgentProblem2D prob = amp::HW8::getWorkspace1(m);

        for (int i = 0; i < 100; i++){

            // run 100 iterations
            decentralRRT dRRT(n, r, p_goal, epsilon);
            std::tuple<bool, double, double> result = dRRT.planCompare(prob);

            // Store values in vectors
            if (std::get<0>(result)){
                    success_amount = success_amount + 1;
            }
            computationTime_vec.push_back(std::get<1>(result)/1000);
        }

        success_list.push_back({success_amount});
        computationTime_list.push_back(computationTime_vec);

        // Plot
        amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 2.d., m = 6", "Hyper Parameters", "Number of Successes");
        amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 2.d., m = 6", "Hyper Parameters", "Computation Time (milliseconds)");
    }

    // centralRRT();
    // DecentralizedMultiAgentRRT temp;

    // use grade
    // amp::HW8::grade<centralRRT, decentralRRT>("nolan.stevenson@colorado.edu", argc, argv);

    amp::Visualizer::showFigures();
    return 0;
}
