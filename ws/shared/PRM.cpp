#include "PRM.h"

amp::Path2D MyPRM2D::plan(const amp::Problem2D& problem ){

    // get bounds from the problem
    if (bounds.size() == 0){
        Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
        Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);
        this->bounds.push_back(xBound);
        this->bounds.push_back(yBound);
    }

    // Convert to a generic PRM problem, Xd dimensions
    Eigen::VectorXd startXd(2);
    Eigen::VectorXd goalXd(2);
    startXd << problem.q_init[0], problem.q_init[1];
    goalXd << problem.q_goal[0], problem.q_goal[1];

    amp::Path path_generic = planxd(startXd, goalXd, problem);

    // Eigen::Vector2d path
    amp::Path2D path2d;
    for (int i = 0; i < path_generic.waypoints.size(); i++){
        Eigen::Vector2d curr;
        curr << path_generic.waypoints[i][0], path_generic.waypoints[i][1];
        path2d.waypoints.push_back(curr);
    }

    // Return path
    return path2d;
}

amp::Path GenericPRM::planxd(Eigen::VectorXd& init_state, Eigen::VectorXd& goal_state, const amp::Problem2D& problem){

    // Add initial conditions to nodes
    nodes[0] = init_state;
    heur.heuristic_values[0] = (init_state - problem.q_goal).norm();
    nodes[1] = goal_state;
    heur.heuristic_values[1] = (goal_state - problem.q_goal).norm();

    // Sample n times
    sampleSpace(nodes, problem);

    // Add the neighbors to the graph
    for (auto it = nodes.begin(); it != nodes.end(); it++){
        // Get the current node
        amp::Node curr_node = it->first;
        // Get the current state
        Eigen::VectorXd curr_state = it->second;
        // Loop through all other nodes
        for (auto it2 = nodes.begin(); it2 != nodes.end(); it2++){
            // If it is the same node, skip
            if (it == it2){
                continue;
            }
            // Get the other node
            amp::Node other_node = it2->first;
            // Get the other state
            Eigen::VectorXd other_state = it2->second;
            // Check if the distance is less than r
            if ((curr_state - other_state).norm() < r){
                // Check if the line between the two states is in collision
                if (!lineFullCollision(curr_state, other_state, problem)){
                    // Add the edge to the graph
                    graph.connect(curr_node, other_node, (curr_state - other_state).norm());
                }
            }
        }
    }

    // Next create the A* path through
    amp::ShortestPathProblem searchProblem;
    searchProblem.graph = std::make_shared<amp::Graph<double>>(graph);
    searchProblem.init_node = 0; // set the init and goal nodes as the nodes defined by problem.q_init and problem.q_goal
    searchProblem.goal_node = 1; // hard coded these as position 0, 1 above

    MyAStarAlgo aStar;
    MyAStarAlgo::GraphSearchResult graphResult = aStar.search(searchProblem, heur);

    // Now convert the graphResult into a path, returning a non-empty path is had success
    amp::Path path;
    if (graphResult.success){
        // Add goal and init to path
        path.waypoints.push_back(problem.q_init);
        for (amp::Node curr : graphResult.node_path){
            path.waypoints.push_back(nodes[curr]);
        }
        path.waypoints.push_back(problem.q_goal);
    }

// // Plot roadmap
//     // convert nodes to a std::map<amp::Node, Eigen::Vector2d> instead of std::map<amp::Node, Eigen::VectorXd>
//     std::map<amp::Node, Eigen::Vector2d> nodes_2d;
//     for (auto& x : nodes){
//         nodes_2d[x.first] = x.second.head<2>();
//     }
//     amp::Visualizer::makeFigure(problem, graph, nodes_2d);

    return path;
}

void GenericPRM::sampleSpace(std::map<amp::Node, Eigen::VectorXd>& nodes, const amp::Problem2D& problem){
    int i = 2;
    while(i < n){
        // Create a random state
        Eigen::VectorXd random_state(nodes[0].size());

        for (int j = 0; j < bounds.size(); j++){
            random_state[j] = randomNum(bounds[j][0], bounds[j][1]);
        }
        // Check if the random state is in collision
        if (!pointCollision(problem, random_state)){
            i++;

            // If not, add it to the node map
            nodes[i] = random_state;

            // Also add the heuristic value to the heuristic map, just use distance to goal
            heur.heuristic_values[i] = (random_state - problem.q_goal).norm();
        }
    }
    return;
}

/// @brief Function to return success, path length and compuatation time
std::tuple<bool, double, double> MyPRM2D::planCompare(const amp::Problem2D& problem){

    // Create a timer
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Now call plan
    amp::Path2D path = plan(problem);

    // Stop the timer
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // Calculate the success
    bool success = path.waypoints.size() > 0;

    // Now calculate the path length
    double path_length;
    if (success){
        path_length = path.length();
    }else{
        path_length = 0;
    }
    
    // Now calculate the computation time
    double computation_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    // Return the tuple
    return std::make_tuple(success, path_length, computation_time);
}