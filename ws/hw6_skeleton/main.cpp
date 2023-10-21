#include "helper.h"

int main(int argc, char** argv) {

//  Exercise 1
//  ==========/

    // create a MyPointWFAlgo object
    MyPointWFAlgo myPointWFAlgo;
    
    amp::Problem2D problem2a = amp::HW2::getWorkspace1();
    std::unique_ptr<amp::GridCSpace2D> gridCSpace2a = myPointWFAlgo.constructDiscretizedWorkspace(problem2a);
    amp::Path2D path2a = myPointWFAlgo.planInCSpace(problem2a.q_init, problem2a.q_goal, *gridCSpace2a);

    // do same for hw2 workspace2
    amp::Problem2D problem2b = amp::HW2::getWorkspace2();
    std::unique_ptr<amp::GridCSpace2D> gridCSpace2b = myPointWFAlgo.constructDiscretizedWorkspace(problem2b);
    amp::Path2D path2b = myPointWFAlgo.planInCSpace(problem2b.q_init, problem2b.q_goal, *gridCSpace2b);

    // visualize problem with path
    // amp::Visualizer::makeFigure(problem2a, path2a);
    // amp::Visualizer::makeFigure(problem2b, path2b);

    // print the path of path2a and path2b
    LOG("path length for 2a: " << path2a.length());
    LOG("path length for 2b: " << path2b.length());

//  Exercise 2
//  ==========/

    // Create a MyManipWFAlgo object
    MyManipWFAlgo myManipWFAlgo;

    // get the workspaces from hw4 
    amp::Problem2D problem4a = amp::HW6::getHW4Problem1();
    amp::Problem2D problem4b = amp::HW6::getHW4Problem2();
    amp::Problem2D problem4c = amp::HW6::getHW4Problem3();

    // get the robot object
    std::vector<double> links;
    links.push_back(1);
    links.push_back(1);
    Link2d robot = Link2d(Eigen::Vector2d(0,0),links); // base of (0,0) assumed

    // // make a MyCSpaceTor instance
    MyCSpaceCtor myCSpaceCtor;
    std::unique_ptr<amp::GridCSpace2D> grid4a = myCSpaceCtor.construct(robot, problem4a);
    std::unique_ptr<amp::GridCSpace2D> grid4b = myCSpaceCtor.construct(robot, problem4b);
    std::unique_ptr<amp::GridCSpace2D> grid4c = myCSpaceCtor.construct(robot, problem4c);

    // get the end points:
    ManipulatorState q_init = robot.getConfigurationFromIK(Eigen::Vector2d(-2,0));
    ManipulatorState q_goal = robot.getConfigurationFromIK(Eigen::Vector2d(2,0));

    // create paths
    amp::Path2D path4a = myManipWFAlgo.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid4a);
    amp::Path2D path4b = myManipWFAlgo.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid4b);
    amp::Path2D path4c = myManipWFAlgo.planInCSpace(Eigen::Vector2d(q_init[0], q_init[1]), Eigen::Vector2d(q_goal[0], q_goal[1]), *grid4c);

    amp::Path pathStruct;
    // try unwrapping path:
    // amp::unwrapPath(path4a, Eigen::Vector2d(0,0), Eigen::Vector2d(2*M_PI,2*M_PI));
    // amp::unwrapPath(path4b, Eigen::Vector2d(0,0), Eigen::Vector2d(2*M_PI,2*M_PI));
    // amp::unwrapPath(path4c, Eigen::Vector2d(0,0), Eigen::Vector2d(2*M_PI,2*M_PI));
   
    // now visualize:
    // amp::Visualizer::makeFigure(*grid4a);
    // amp::Visualizer::makeFigure(*grid4b);
    // amp::Visualizer::makeFigure(*grid4c);

    // amp::Visualizer::makeFigure(*grid4a, path4a);
    // amp::Visualizer::makeFigure(*grid4b, path4b);
    // amp::Visualizer::makeFigure(*grid4c, path4c);
    // amp::Visualizer::makeFigure(problem4a, robot, path4a);
    // amp::Visualizer::makeFigure(problem4b, robot, path4b);
    // amp::Visualizer::makeFigure(problem4c, robot, path4c);
    // amp::Visualizer::showFigures();

    bool successProb2a = HW6::checkPointAgentPlan(path2a, problem2a);
    LOG("Found valid solution to workspace 2a using point agent?: " << (successProb2a ? "Yes!" : "No :("));
    bool successProb2b = HW6::checkPointAgentPlan(path2b, problem2b);
    LOG("Found valid solution to workspace 2b using point agent?: " << (successProb2b ? "Yes!" : "No :("));
    bool successProb4a = HW6::checkLinkManipulatorPlan(path4a, robot, problem4a);
    LOG("Found valid solution to workspace 4a using point agent?: " << (successProb4a ? "Yes!" : "No :("));
    bool successProb4b = HW6::checkLinkManipulatorPlan(path4b, robot, problem4b);
    LOG("Found valid solution to workspace 4b using point agent?: " << (successProb4b ? "Yes!" : "No :("));
    bool successProb4c = HW6::checkLinkManipulatorPlan(path4c, robot, problem4c);
    LOG("Found valid solution to workspace 4c using point agent?: " << (successProb4c ? "Yes!" : "No :("));

// Exercise 3
// ==========/

    // Implement A*
    MyAStarAlgo myAStarAlgo;
    
    // Get the resulting GraphSearchResult
    MyAStarAlgo::GraphSearchResult result = myAStarAlgo.search(amp::HW6::getEx3SPP(), amp::HW6::getEx3Heuristic());

    // check for success
    bool successExerecise3 = HW6::checkGraphSearchResult(result, amp::HW6::getEx3SPP(), amp::HW6::getEx3Heuristic());
    LOG("Found valid solution using A*?: " << (successExerecise3 ? "Yes!" : "No :("));

    // amp::RNG::seed(amp::RNG::randiUnbounded());
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nolan.stevenson@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;

}