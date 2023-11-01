#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "Eigen/Geometry"
#include "HelpfulClass.h"
#include "helper.h"
#include "helperFuncs.h"
#include "PRM.h"
#include "RRT.h"
#include "AStar.h"
#include "newHelper.h"

int main(int argc, char** argv) {

    // Which cases to run
    struct cases {
        bool log;
        std::string name;
    };
    std::vector<cases> options = {{false, "Exercise 1.a.i."}, {false, "Exercise 1.a.ii."}, {false, "Exercise 1.a.iv."}, {false, "Exercise 1.b.i."}, {false, "Exercise 1.b.ii."}, {false, "Exercise 1.b.iv."}, {false, "Exercise 2.a."}, {false, "Exercise 2.b."}};

// Exercise 1, PRM

// a.i.
    if (options[0].log){

        // Get problem, Hw5
        amp::Problem2D problem = amp::HW5::getWorkspace1();

        // Bounds as defined
        Eigen::Vector2d xBound = Eigen::Vector2d(-1, 11);
        Eigen::Vector2d yBound = Eigen::Vector2d(-3, 3);

        // Hyper parameters
        int n = 200;
        double r = 1;

        // Create PRM
        MyPRM2D prm(xBound, yBound, n, r);
        // Plan using .plan
        amp::Path2D path = prm.plan(problem);
    
        // log path length
        if (path.waypoints.size() > 0){
            std::cout << "1.a.i. Path length: " << path.length() << std::endl;
        }
        else{
            std::cout << "1.a.i. Path length: " << "No path found" << std::endl;
        }

        // Visualize
        // amp::HW7::hint();
        amp::Visualizer::makeFigure(problem, path);
    }

// a.ii.
    if (options[1].log){
        std::list<std::vector<double>> data_set = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2}, {500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
        std::vector<std::string> labels = {"n=200, r=0.5", "n=200, r=1", "n=200, r=1.5", "n=200, r=2", "n=500, r=0.5", "n=500, r=1", "n=500, r=1.5", "n=500, r=2"};
        std::list<std::vector<double>> success_list;
        std::list<std::vector<double>> pathLength_list;
        std::list<std::vector<double>> computationTime_list;
       
        // std::list<std::vector<double>> data_set = {{200,2}};
        // std::vector<std::string> labels = {"n=200, r=2"};
        // now loop through all data_set trials, running prm with the given parameters, storing the success rate
        for (auto curr : data_set){
            // Get problem, Hw5
            amp::Problem2D problem = amp::HW5::getWorkspace1();

            // Bounds as defined
            Eigen::Vector2d xBound = Eigen::Vector2d(-1, 11);
            Eigen::Vector2d yBound = Eigen::Vector2d(-3, 3);

            double success_amount = 0;
            // Hyper parameters
            double n = curr[0];
            double r = curr[1];

            // output the hypers
            std::cout << "n: " << n << " r: " << r << std::endl;

            // create vectors to store the path length and computation time for each 100 iteration loop
            std::vector<double> pathLength_vec;
            std::vector<double> computationTime_vec;

            // run 100 iterations
            for (int i = 0; i<100; i++){

                // Create PRM
                MyPRM2D prm(xBound, yBound, n, r);

                // Get the result of the planCompare function
                std::tuple<bool, double, double, amp::Path2D> result = prm.planCompare(problem);

                // amp::Visualizer::makeFigure(problem, std::get<3>(result));

// a.iv. Path smoothing for benchmark
                amp::Path2D newPath;
                if (options[2].log && std::get<0>(result)){ 
                    newPath = pathSmooth(std::get<3>(result), problem);
                }

                // Add to the success vector
                if (std::get<0>(result)){
                    success_amount = success_amount + 1;
                }

                // Add the path length vector, only if its non zero
                if (options[2].log && std::get<0>(result)){
                    pathLength_vec.push_back(newPath.length());
                }else{
                    if (std::get<1>(result) > 0){
                        pathLength_vec.push_back(std::get<1>(result));
                    }
                }

                // Add to the computation time vector, dividing by 1e6 to get into seconds
                computationTime_vec.push_back({std::get<2>(result)/1e3});
            }
            // Store in lists
            success_list.push_back({success_amount});
            pathLength_list.push_back(pathLength_vec);
            computationTime_list.push_back(computationTime_vec);
        }
        // Plot
        amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 1.a.ii., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Number of Successes");
        amp::Visualizer::makeBoxPlot(pathLength_list, labels, "Excersie 1.a.ii., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Path Length");
        amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 1.a.ii., HW5 workspace", "Number of Nodes, Neighborhood Radius", "Computation Time (milliseconds)");
    }

// b.i.
    if (options[3].log){
        // Get problem, Hw2, ws1
        amp::Problem2D problem = amp::HW2::getWorkspace1();

        // Bounds as defined
        Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
        Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);

        // Hyper parameters
        int n = 200;
        double r = 2;

        // Create PRM
        MyPRM2D prm(xBound, yBound, n, r);

        // Plan using .plan
        amp::Path2D path = prm.plan(problem);

        if (path.waypoints.size() > 0){
            std::cout << "1.b.i.ws1 Path length: " << path.length() << std::endl;
        }
        else{
            std::cout << "1.b.i.ws1 Path length: " << "No path found" << std::endl;
        }
        // Visualize
        amp::Visualizer::makeFigure(problem, path);

        // Get problem, Hw2, ws2
        amp::Problem2D problem2 = amp::HW2::getWorkspace2();

        // Bounds as defined
        Eigen::Vector2d xBound2 = Eigen::Vector2d(-7, 35);
        Eigen::Vector2d yBound2 = Eigen::Vector2d(-7, 7);

        // Hyper parameters
        int n2 = 200;
        double r2 = 2;

        // Create PRM
        MyPRM2D prm2(xBound2, yBound2, n2, r2);

        // Plan using .plan
        amp::Path2D path2 = prm2.plan(problem2);

        if (path2.waypoints.size() > 0){
            std::cout << "1.b.i.ws2 Path length: " << path2.length() << std::endl;
        }
        else{
            std::cout << "1.b.i.ws2 Path length: " << "No path found" << std::endl;
        }
        // Visualize
        amp::Visualizer::makeFigure(problem2, path2);
    }

// b.ii.
    if (options[4].log){
        std::list<std::vector<double>> data_set = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000,1}, {1000,2}};
        std::vector<std::string> labels = {"n=200, r=1", "n=200, r=2", "n=500, r=1", "n=500, r=2", "n=1000, r=1", "n=1000, r=2"};
        std::list<std::vector<double>> success_list;
        std::list<std::vector<double>> pathLength_list;
        std::list<std::vector<double>> computationTime_list;
       
        // now loop through all data_set trials, running prm with the given parameters, storing the success rate
        for (auto curr : data_set){
            // Get problem, Hw5
            amp::Problem2D problem = amp::HW2::getWorkspace2();

            // Bounds as defined
            Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
            Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);

            double success_amount = 0;
            // Hyper parameters
            double n = curr[0];
            double r = curr[1];

            // output the hypers
            std::cout << "n: " << n << " r: " << r << std::endl;

            // create vectors to store the path length and computation time for each 100 iteration loop
            std::vector<double> pathLength_vec;
            std::vector<double> computationTime_vec;

            // run 100 iterations
            for (int i = 0; i<100; i++){

                // Create PRM
                MyPRM2D prm(xBound, yBound, n, r);

                // Get the result of the planCompare function
                std::tuple<bool, double, double, amp::Path2D> result = prm.planCompare(problem);

                amp::Path2D newPath;
                if (options[5].log && std::get<0>(result)){ 
                    newPath = pathSmooth(std::get<3>(result), problem);
                }

                // Add to the success vector
                if (std::get<0>(result)){
                    success_amount = success_amount + 1;
                }

                // Add the path length vector, only if its non zero
                if (options[5].log && std::get<0>(result)){
                    pathLength_vec.push_back(newPath.length());
                }else{
                    if (std::get<1>(result) > 0){
                        pathLength_vec.push_back(std::get<1>(result));
                    }
                }

                // Add to the computation time vector, dividing by 1e6 to get into seconds
                computationTime_vec.push_back({std::get<2>(result)/1e3});
            }
            // Store in lists
            success_list.push_back({success_amount});
            pathLength_list.push_back(pathLength_vec);
            computationTime_list.push_back(computationTime_vec);
        }
        // Plot
        amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 1.b.ii., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Number of Successes");
        amp::Visualizer::makeBoxPlot(pathLength_list, labels, "Excersie 1.b.ii., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Path Length");
        amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 1.b.ii., HW2 Workspace 2", "Number of Nodes, Neighborhood Radius", "Computation Time (milliseconds)");
    }

// Exercise 2, RRT

    // a
    if (options[6].log){
        // Define hypers
        double n = 5000;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;

        // // Get problem, Hw5 2a
        // amp::Problem2D problem1 = amp::HW5::getWorkspace1();

        // // Bounds as defined
        // Eigen::Vector2d xBound1 = Eigen::Vector2d(-1, 11);
        // Eigen::Vector2d yBound1 = Eigen::Vector2d(-3, 3);

        // // Create RRT
        // MyRRT2D rrt1(xBound1, yBound1, n, r, p_goal, epsilon);

        // // Plan using .plan
        // amp::Path2D path1 = rrt1.plan(problem1);

        // // Visualize
        // amp::Visualizer::makeFigure(problem1, path1);

        // // Output path length
        // if (path1.waypoints.size() > 0){
        //     std::cout << "2.a. Path length: " << path1.length() << std::endl;
        // }
        // else{
        //     std::cout << "2.a. Path length: " << "No path found" << std::endl;
        // }

        // // Get problem, Hw2 ws1 
        // amp::Problem2D problem2 = amp::HW2::getWorkspace1();

        // // Bounds as defined
        // Eigen::Vector2d xBound2 = Eigen::Vector2d(problem2.x_min, problem2.x_max);
        // Eigen::Vector2d yBound2 = Eigen::Vector2d(problem2.y_min, problem2.y_max);

        // // Create RRT
        // MyRRT2D rrt2(xBound2, yBound2, n, r, p_goal, epsilon);

        // // Plan using .plan
        // amp::Path2D path2 = rrt2.plan(problem2);

        // // Visualize
        // amp::Visualizer::makeFigure(problem2, path2);

        // // Output path length
        // if (path2.waypoints.size() > 0){
        //     std::cout << "2.a. Path length: " << path2.length() << std::endl;
        // }
        // else{
        //     std::cout << "2.a. Path length: " << "No path found" << std::endl;
        // }

        // Get problem, Hw2 ws2
        amp::Problem2D problem3 = amp::HW2::getWorkspace2();

        // Bounds as defined
        Eigen::Vector2d xBound3 = Eigen::Vector2d(-7, 35);
        Eigen::Vector2d yBound3 = Eigen::Vector2d(-7, 7);

        // Create RRT
        MyRRT2D rrt3(xBound3, yBound3, n, r, p_goal, epsilon);

        // Plan using .plan
        amp::Path2D path3 = rrt3.plan(problem3);

        // Visualize
        amp::Visualizer::makeFigure(problem3, path3);

        // Output path length
        if (path3.waypoints.size() > 0){
            std::cout << "2.a. Path length: " << path3.length() << std::endl;
        }
        else{
            std::cout << "2.a. Path length: " << "No path found" << std::endl;
        }
    }

    // b
    if (options[7].log){
        std::vector<amp::Problem2D> data_set = {amp::HW5::getWorkspace1(), amp::HW2::getWorkspace1(), amp::HW2::getWorkspace2()};
        std::vector<std::string> labels = {"HW5 Workspace", "HW2 Workspace 1", "HW2 Workspace 2"};
        std::list<std::vector<double>> success_list;
        std::list<std::vector<double>> pathLength_list;
        std::list<std::vector<double>> computationTime_list;

        // Hyper parameters
        double n = 5000;
        double r = 0.5;
        double p_goal = 0.05;
        double epsilon = 0.25;
        int i = 0;
        for (auto problem : data_set){
            i++;
            std::cout << "Running problem " << i << std::endl;
            Eigen::Vector2d xBound;
            Eigen::Vector2d yBound;
            if (i == 1){
                // Bounds as defined
                xBound = Eigen::Vector2d(-1, 11);
                yBound = Eigen::Vector2d(-3, 3);
            }else{
                // Bounds as defined
                xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
                yBound = Eigen::Vector2d(problem.y_min, problem.y_max);
            }

            // create vectors to store the path length and computation time for each 100 iteration loop
            double success_amount = 0;
            std::vector<double> pathLength_vec;
            std::vector<double> computationTime_vec;

            // run 100 iterations
            for (int i = 0; i<100; i++){

                // Create PRM
                MyRRT2D rrt(xBound, yBound, n, r, p_goal, epsilon);

                // Get the result of the planCompare function
                std::tuple<bool, double, double> result = rrt.planCompare(problem);

                // Add to the success vector
                if (std::get<0>(result)){
                    success_amount = success_amount + 1;
                }

                // Add the path length vector, only if its non zero
                if (std::get<1>(result) > 0){
                    pathLength_vec.push_back({std::get<1>(result)});
                }

                // Add to the computation time vector, dividing by 1e6 to get into seconds
                computationTime_vec.push_back({std::get<2>(result)/1e3});
            }
            // Store in lists
            success_list.push_back({success_amount});
            pathLength_list.push_back(pathLength_vec);
            computationTime_list.push_back(computationTime_vec);
        }
  
        // Plot
        amp::Visualizer::makeBoxPlot(success_list, labels, "Excersie 2.b., Success Comparison", "", "Number of Successes");
        amp::Visualizer::makeBoxPlot(pathLength_list, labels, "Excersie 2.b., Path Length Comparison", "", "Path Length");
        amp::Visualizer::makeBoxPlot(computationTime_list, labels, "Excersie 2.b., Computation Time Comparison", "", "Computation Time (milliseconds)");
    }

// Show plots
    amp::Visualizer::showFigures();

// Try grade feature
    // amp::HW7::grade<MyPRM2D, MyRRT2D>("nolan.stevenson@colorado.edu", argc, argv);

    return 0;
}