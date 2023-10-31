#include "RRT.h"

/// @brief Runs RRT on the problem using the private hyper parameters
amp::Path2D MyRRT2D::plan(const amp::Problem2D& problem){

    // get bounds from the problem
    if (bounds.size() == 0){
        Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
        Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);
        this->bounds.push_back(xBound);
        this->bounds.push_back(yBound);
    }

    // Define the path
    amp::Path2D path;

    // Initialize start node
    nodes[0] = problem.q_init;
    heur.heuristic_values[0] = (problem.q_init - problem.q_goal).norm();

    // Sample up to n times
    int i = 2;
    while (i < n){

        // Sample a random state
        // Check if we should sample the goal
        double p = randomNum(0, 1);
        Eigen::Vector2d random_state;
        if (p < p_goal){
            random_state = problem.q_goal;
        }
        else{
            random_state = Eigen::Vector2d(randomNum(bounds[0][0], bounds[0][1]), randomNum(bounds[1][0], bounds[1][1]));
        }

        // Check if the random state is in collision
        if (!pointCollision(problem, random_state)){

            // Find the nearest node
            amp::Node nearest_node = nearestNode(random_state);
            Eigen::Vector2d nearest_state = nodes[nearest_node];

            // Try that direction going r distance
            Eigen::Vector2d new_state = r*(random_state - nearest_state).normalized() + nearest_state;
    
            // Now try connecting the nearest node to the random state
            // Check if the line between the two states is in collision
            if (!lineFullCollision(nearest_state, new_state, problem)){

                // Add new node to map
                nodes[i] = new_state;

                // Add the heuristic value to the heuristic map
                heur.heuristic_values[i] = (new_state - problem.q_goal).norm();
                
                // Add the node to the graph
                graph.connect(nearest_node, i, (nearest_state - new_state).norm());

                // Check if the random state is the goal state
                if ((new_state - problem.q_goal).norm() < epsilon){

                    // Add goal node to node map and heuristic
                    nodes[1] = problem.q_goal;
                    heur.heuristic_values[1] = 0;
                    
                    // Connect from random node to goal node
                    graph.connect(i, 1, (new_state - problem.q_goal).norm());
                    
                    // Run A*
                    amp::ShortestPathProblem searchProblem;
                    searchProblem.graph = std::make_shared<amp::Graph<double>>(graph);
                    searchProblem.init_node = 0; // set the init and goal nodes as the nodes defined by problem.q_init and problem.q_goal
                    searchProblem.goal_node = 1; // hard coded these as position 0, 1 above

                    MyAStarAlgo aStar;
                    MyAStarAlgo::GraphSearchResult graphResult = aStar.search(searchProblem, heur);

                    if (graphResult.success){
                        // amp::Visualizer::makeFigure(problem, graph, nodes);
                        // Add goal and init to path
                        path.waypoints.push_back(problem.q_init);
                        for (amp::Node curr : graphResult.node_path){
                            path.waypoints.push_back(nodes[curr]);
                        }
                        path.waypoints.push_back(problem.q_goal);
                        return path;
                    }
                }
                i++; // Iterate after adding a node (needs to pass both collision checks)
            }
        }
    }
    return path;
}

amp::Node MyRRT2D::nearestNode(Eigen::Vector2d state){
    // Go through every node in the map and find the nearest one
    double min_dist = std::numeric_limits<double>::infinity();
    amp::Node nearest;
    for (auto& x : nodes){
        double dist = (x.second - state).norm();
        if (dist < min_dist){
            min_dist = dist;
            nearest = x.first;
        }
    }
    return nearest;
}

/// @brief Function to return success, path length and compuatation time
std::tuple<bool, double, double> MyRRT2D::planCompare(const amp::Problem2D& problem){

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