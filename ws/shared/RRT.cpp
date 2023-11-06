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

                // Check if the new state is the goal state
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
                        amp::Visualizer::makeFigure(problem, graph, nodes);
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


//// MULTI AGENT PLANNING:

/// @brief Runs RRT on the problem using the private hyper parameters
amp::MultiAgentPath2D centralRRT::plan(const amp::MultiAgentProblem2D& origProblem){
    
    double mostRecentTest; 

    // First expand all obstacles to include the robot radius (c-space)
    const amp::MultiAgentProblem2D problem = expand(origProblem);

    // Create the path
    amp::MultiAgentPath2D path;
    // Make sure it is m size
    path.agent_paths = std::vector<amp::Path2D>(problem.agent_properties.size());

    // Get the bounds of the problem
    Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
    Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);
    bounds.push_back(xBound);
    bounds.push_back(yBound);

    // Get the robot stats
    radius = problem.agent_properties[0].radius;
    m = problem.agent_properties.size();

    // Initialize the start node
    // Get the std::vector<Eigen::Vector2d> of the init states
    std::vector<Eigen::Vector2d> init_states;
    for (int i = 0; i < m; i++){
        init_states.push_back(problem.agent_properties[i].q_init);
    }
    nodes[0] = init_states;

    // Initalize the goal node
    // Get the std::vector<Eigen::Vector2d> of the goal states
    std::vector<Eigen::Vector2d> goal_states;
    for (int i = 0; i < m; i++){
        goal_states.push_back(problem.agent_properties[i].q_goal);
    }

// Now run RRT
    // Sample up to n times
    int i = 2;
    while (i < n){

    // Sample a random states
        std::vector<Eigen::Vector2d> random_states;
        bool goalFlag = false;
        // Check if we should sample goal
        double p = randomNum(0, 1);
        if (p < p_goal){
            random_states = goal_states; // Sample goal
            mostRecentTest = i;
            goalFlag = true;
        }
        else{
            for (int j = 0; j < m; j++){
                random_states.push_back(Eigen::Vector2d(randomNum(bounds[0][0], bounds[0][1]), randomNum(bounds[1][0], bounds[1][1]))); // Sample randomly
            }
        }

    // Check if the random states are in collision with obstacle or eachother
        if (!multiPointCollision(problem, random_states)){
            
            // Find the nearest node
            amp::Node nearest_node = nearestNode(random_states);
            std::vector<Eigen::Vector2d> nearest_states = nodes[nearest_node];

            // std::cout << "Current random sample is:" << std::endl;
            // for (int j = 0; j < m; j++){
            //     std::cout << random_states[j] << std::endl;
            // }
            // std::cout << "Nearest state(s) is:" << std::endl;
            // for (int j = 0; j < m; j++){
            //     std::cout << nearest_states[j] << std::endl;
            // }

            // Try that direction going r distance for each robot towards the random state from the nearest state
            std::vector<Eigen::Vector2d> new_states;
            for (int j = 0; j < m; j++){
                // Extend the vector from the nearest state to the random state by r (or less if its closer)
                Eigen::Vector2d extend = r*(random_states[j] - nearest_states[j]).normalized();
                if (extend.norm() > (random_states[j] - nearest_states[j]).norm() && goalFlag){
                    extend = (random_states[j] - nearest_states[j]);
                    // std::cout << "Extend is too long and sampled goal, changing to random - nearest" << std::endl;
                    // std::cout << extend << std::endl;
                }
                Eigen::Vector2d new_state = extend + nearest_states[j];
                new_states.push_back(new_state);
            }

            // std::cout << "New state is:" << std::endl;
            // for (int j = 0; j < m; j++){
            //     std::cout << new_states[j] << std::endl;
            // }

            // Check if the line between the two states is in collision
            if (!multiLineCollision(problem, nearest_states, new_states)){

                // Add the new_states to the nodes map
                nodes[i] = new_states;

                // Add to graph
                // Get the distance between the nearest and new states
                double dist = 0;
                for (int j = 0; j < m; j++){
                    dist += (nearest_states[j] - new_states[j]).norm(); // should be m*r everytime
                }
                graph.connect(nearest_node, i, r);

                // Check if the new state is the goal state, if so return path
                if (multiGoalCheck(problem, new_states)){

                    std::cout << "Found a path!" << std::endl;
                    // std::cout << i << std::endl;
                    // std::cout << "New states has nodes: " << std::endl;
                    // for (int j = 0; j < m; j++){
                    //     std::cout << new_states[j] << std::endl;
                    // }

                    // Add goal node to node map
                    nodes[1] = goal_states;
                    
                    // Connect from random node to goal node, 0 distance, random node is goal_node (nearest is goal)
                    double dist = 0;
                    for (int j = 0; j < m; j++){
                        dist += (new_states[j] - goal_states[j]).norm(); // should be r*m everytime
                    }
                    graph.connect(i, 1, r);
                    
                    // Get the path from initial to goal node (1)
                    path.agent_paths = getPath(1);

                    return path;
                }
                i++;
            }
        } 
    }

    std::cout << "Couldn't find path" << std::endl;
    // std::cout << "Most recent test was: " << mostRecentTest << std::endl;
    // path.agent_paths = getPath(mostRecentTest);
    // // plot this path
    // amp::Visualizer::makeFigure(problem, path);

    // check multigoal
    // bool test = multiGoalCheck(problem, nodes[mostRecentTest]);

    return path;
}

/// @brief Returns nearest node to the given state
amp::Node centralRRT::nearestNode(std::vector<Eigen::Vector2d> state){
    // Go through every node in the map and find the nearest one
    double min_dist = std::numeric_limits<double>::infinity();
    amp::Node nearest;
    for (auto& x : nodes){
        double dist = 0;
        // Sum dist for all
        for (int i = 0; i < m; i++){
            dist += (x.second[i] - state[i]).norm();
        }
        if (dist < min_dist){
            min_dist = dist;
            nearest = x.first;
        }
    }
    return nearest;
}

/// @brief Expands the obstacles to include the robot radius
const amp::MultiAgentProblem2D centralRRT::expand(const amp::MultiAgentProblem2D& origProblem){
    
    radius = origProblem.agent_properties[0].radius;

    // Create a new problem
    amp::MultiAgentProblem2D problem = origProblem;

    // Now replace all obstacles in problem with new obstacles
    std::vector<amp::Obstacle2D> curr_obstacles = problem.obstacles;
    std::vector<amp::Obstacle2D> new_obstacles;
    // Loop through all curr_obstacles
    for (int i = 0; i < curr_obstacles.size(); i++){
        // Get the centroid of the curr_obstacles
        std::vector<Eigen::Vector2d> vertices = curr_obstacles[i].verticesCCW();
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (int j = 0; j < vertices.size(); j++){
            centroid += vertices[j];
        }
        centroid /= vertices.size();
        // Now expand the obstacle by the radius
        std::vector<Eigen::Vector2d> new_vertices;
        for (int j = 0; j < vertices.size(); j++){
            // Make a direction vec that says either 1 or -1 for each xy direction
            Eigen::Vector2d dirVec;
            for (int k = 0; k < 2; k++){
                if (vertices[j][k] > centroid[k]){
                    dirVec[k] = 1;
                }else{
                    dirVec[k] = -1;
                }
            }
            Eigen::Vector2d new_vertex = vertices[j] + radius*dirVec;
            new_vertices.push_back(new_vertex);

            // Use centroid method
            // Eigen::Vector2d new_vertex = vertices[j] + radius*(vertices[j] - centroid).normalized();
            // new_vertices.push_back(new_vertex);
        }
        // Now create a new obstacle
        amp::Obstacle2D new_obstacle(new_vertices);
        new_obstacles.push_back(new_obstacle);
    }
    // Now replace the obstacles in problem
    problem.obstacles = new_obstacles;
    // visualize
    amp::Visualizer::makeFigure(problem);

    return problem;
}

/// @brief This function checks a set of points for a obstacle-robot collision or robot-robot collision
bool centralRRT::multiPointCollision(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points){
   
    // First check if poitns are in collision against eachother
    for (int j = 0; j < m; j++){
        for (int k = j+1; k < m; k++){
            if ((points[j] - points[k]).norm() < 2*radius){
                // std::cout << "Robots in collision with eachother! Points are:" << std::endl;
                // for (int l = 0; l < m; l++){
                //     std::cout << points[l] << std::endl;
                // }
                return true;
            }
        }
    }

    // Next check if in collision with any obstacles
    // Use the same method as point collision
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;

    // Loop through all points
    for (int p = 0; p < points.size(); p++){
        for (int i = 0; i < obstacles.size(); i++){
            int count = 0;
            std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
            // loop through all vertices
            for (int j = 0; j < vertices.size(); j++){
                // check the normal angle of a line between each point
                Eigen::ParametrizedLine<double, 2> line(vertices[j], vertices[(j+1)%vertices.size()] - vertices[j]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);
                if ((normal.dot(points[p] - vertices[j]) <= 0)){ // method for finding if a point is inside polygon
                    count++;
                }
            }
            // at the end, if count is equal to the vertices size, then in collision
            if (count == vertices.size()){
                return true;
            }
        }
    }
    return false;
}

/// @brief This function checks a set of points for a obstacle-robot colllisions using line-line intersections
bool centralRRT::multiLineCollision(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> currState, std::vector<Eigen::Vector2d> nextState){
    
    // Use same method of line-line intersection but for all points
    
    // get all the obstacles
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;

    // loop through all points in state
    for (int p = 0; p < currState.size(); p++){
        // loop through all obstacles
        Eigen::Vector2d currNode = currState[p];
        Eigen::Vector2d nextNode = nextState[p];
        // std::cout << "The robot config is: " << std::endl;
        // std::cout << currNode << std::endl;
        // std::cout << nextNode << std::endl;

        for (int i = 0; i < obstacles.size(); i++){
            std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
            // loop through all vertices
            for (int j = 0; j < vertices.size(); j++){
                // check if the line collides with any of the vertices
                for (int r = 0; r < m; r++){
                    
                    if (lineCollision(currNode, nextNode, vertices[j%vertices.size()], vertices[(j+1)%vertices.size()])){
                        // std::cout << "Line collision detected, the obstacle verts are: " << std::endl;
                        // std::cout << vertices[j%vertices.size()] << std::endl;
                        // std::cout << vertices[(j+1)%vertices.size()] << std::endl;
                        // std::cout << "The robot configs are: " << std::endl;
                        // std::cout << currNode << std::endl;
                        // std::cout << nextNode << std::endl;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/// @brief This function checks if the points are within the goal state by a epsilon distance
bool centralRRT::multiGoalCheck(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points){
    // Check if the points are within epsilon of the goal
    // std::cout << "The points are: " << std::endl;
    // for (int i = 0; i < m; i++){
    //     std::cout << points[i] << std::endl;
    // }
    for (int i = 0; i < m; i++){
        // if ((points[i] - problem.agent_properties[i].q_goal).norm() > 2*epsilon){
        if ((points[i] - problem.agent_properties[i].q_goal).norm() > epsilon){
            return false;
        }
    }
    return true;
}

/// @brief Returns the path form the goal node to the init node
std::vector<amp::Path2D> centralRRT::getPath(int end_node){

    // Create the paths node
    std::vector<amp::Path2D> paths;

    // Loop for all robots
    for (int i = 0; i < m; i++){

        amp::Path2D path_curr;

        // Start from the given node
        int curr_node = end_node;

        // Loop through all nodes until we get to the init node (0)
        while(curr_node != 0){
            path_curr.waypoints.push_back((nodes[curr_node])[i]);
            curr_node = graph.parents(curr_node)[0]; // just take first parent
        }
        path_curr.waypoints.push_back((nodes[0])[i]); // Adds goal node

        // Add the path to the vector of paths
        std::reverse(path_curr.waypoints.begin(), path_curr.waypoints.end());
        paths.push_back(path_curr);
    }
    return paths;
}