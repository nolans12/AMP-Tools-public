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
amp::MultiAgentPath2D centralRRT::plan(const amp::MultiAgentProblem2D& problem){

    // First expand all obstacles to include the robot radius (c-space)
    // const amp::MultiAgentProblem2D problem = expand(origProblem);

    // Create the path
    amp::MultiAgentPath2D path;
    // Make sure it is m size
    path.agent_paths = std::vector<amp::Path2D>(problem.agent_properties.size());

    return path;

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
    int count = 0;
    int i = 2;
    while (i < n){

    // Sample a random states
        std::vector<Eigen::Vector2d> random_states;
        bool goalFlag = false;
        // Check if we should sample goal
        double p = randomNum(0, 1);
        if (p < p_goal){
            random_states = goal_states; // Sample goal
            goalFlag = true;
        }
        else{
            for (int j = 0; j < m; j++){
                random_states.push_back(Eigen::Vector2d(randomNum(bounds[0][0], bounds[0][1]), randomNum(bounds[1][0], bounds[1][1]))); // Sample randomly
            }
        }

    // Check if the random states are in collision with obstacle or eachother
        // Find the nearest node
        amp::Node nearest_node = nearestNode(random_states);
        std::vector<Eigen::Vector2d> nearest_states = nodes[nearest_node];

        // Try that direction going r distance for each robot towards the random state from the nearest state
        std::vector<Eigen::Vector2d> new_states;
        for (int j = 0; j < m; j++){
            // Extend the vector from the nearest state to the random state by r (or less if its closer)
            Eigen::Vector2d extend = r*(random_states[j] - nearest_states[j]).normalized();
            if (extend.norm() > (random_states[j] - nearest_states[j]).norm() && goalFlag){
                extend = (random_states[j] - nearest_states[j]);
            }
            Eigen::Vector2d new_state = extend + nearest_states[j];
            new_states.push_back(new_state);
        }

        // Check if the line between the two states is in collision or if the new_states are in collision with eachother
        if (!multiPointCollision(problem, new_states) && !multiLineCollision(problem, nearest_states, new_states)){
        // if (!multiPointCollision(problem, new_states)){

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
            // i++;
        }
        i++;
        // count++;
        // if (count > 50000){
        //     amp::Visualizer::makeFigure(problem);
        //     break;
        // }
    }

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
        }
        // Now create a new obstacle
        amp::Obstacle2D new_obstacle(new_vertices);
        new_obstacles.push_back(new_obstacle);
    }
    // Now replace the obstacles in problem
    problem.obstacles = new_obstacles;
    // visualize
    // amp::Visualizer::makeFigure(problem);

    return problem;
}

/// @brief This function checks a set of points for a obstacle-robot collision or robot-robot collision
bool centralRRT::multiPointCollision(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points){
   
// First check if points are in collision against eachother
    for (int j = 0; j < m; j++){
        for (int k = j+1; k < m; k++){
            if ((points[j] - points[k]).norm() <= 2*radius + epsilon){
                return true;
            }
        }
    }

// Next check if in collision with any obstacles
    // use ray casting to detect the closest obstacle
    for (int p = 0; p < points.size(); p++){
        Eigen::Vector2d currMin = rayDetect(problem, points[p]);
        if (currMin != Eigen::Vector2d(INT16_MAX, INT16_MAX)){
            if ((currMin - points[p]).norm() <= radius + epsilon){
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
   
        for (int i = 0; i < obstacles.size(); i++){
            std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
            // loop through all vertices
            for (int j = 0; j < vertices.size(); j++){
                // check if the line collides with any of the vertices
                for (int r = 0; r < m; r++){    
                    if (lineCollision(currNode, nextNode, vertices[j%vertices.size()], vertices[(j+1)%vertices.size()])){
                        return true;
                    }
                }
            }
        }
    }

    // Also check if each of the lines between the points are in collision with eachother, not just obstacles
    // for (int p = 0; p < currState.size(); p++){
    //     for (int q = p+1; q < currState.size(); q++){
    //         if (lineCollision(currState[p], nextState[p], currState[q], nextState[q])){
    //             return true;
    //         }
    //     }
    // }

    return false;
}

/// @brief This function checks if the points are within the goal state by a epsilon distance
bool centralRRT::multiGoalCheck(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points){
    // Check if the points are within epsilon of the goal
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
    for (int i = 0; i < m; i++){
        amp::Path2D path_curr;
        paths.push_back(path_curr);
    }

    // Start from the given node
    int curr_node = end_node;

    // Loop through all nodes until we get to the init node (0)
    while(curr_node != 0){
        for (int i = 0; i < m; i++){
            paths[i].waypoints.push_back((nodes[curr_node])[i]);
        }
        curr_node = graph.parents(curr_node)[0]; // just take first parent
    }

    // Add goal node
    for (int i = 0; i < m; i++){
        paths[i].waypoints.push_back((nodes[0])[i]);
    }

    // Reverse
    for (int i = 0; i < m; i++){
        std::reverse(paths[i].waypoints.begin(), paths[i].waypoints.end());
    }

    return paths;
}

/// @brief Function to return success, path length and compuatation time
std::tuple<bool, double, double> centralRRT::planCompare(const amp::MultiAgentProblem2D& problem){

    // Create a timer
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Now call plan
    amp::MultiAgentPath2D path = plan(problem);

    // Stop the timer
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // Calculate the success
    bool success = amp::HW8::check(path, problem);
    // bool success = path.agent_paths[0].waypoints.size() > 0;

    // Now calculate the computation time
    double computation_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    // Return the tuple
    return std::make_tuple(success, computation_time, 0);
}

/// @brief Runs Decentralized RRT on the problem using the private hyper parameters
amp::MultiAgentPath2D decentralRRT::plan(const amp::MultiAgentProblem2D& problem){

    // amp::MultiAgentProblem2D problem = expand(origProblem);

    // Create the path
    amp::MultiAgentPath2D path(problem.numAgents());

    // return path;
    
    // Get the bounds of the problem
    Eigen::Vector2d xBound = Eigen::Vector2d(problem.x_min, problem.x_max);
    Eigen::Vector2d yBound = Eigen::Vector2d(problem.y_min, problem.y_max);
    bounds.push_back(xBound);
    bounds.push_back(yBound);

    // Get the robot stats
    radius = problem.agent_properties[0].radius;
    m = problem.agent_properties.size();

    bool check = false;
    int count = 0;

    for (int agentNum; agentNum < problem.numAgents(); agentNum++){
        
        amp::Path2D pathCurr;

        // Initialize the start node
        nodes[0] = problem.agent_properties[agentNum].q_init;
        times[0] = 0;
    
        // Sample up to n times
        int i = 2;
        int test = 0;
        while (i < n){

        // Sample a random state
            // Check if we should sample the goal
            double p = randomNum(0, 1);
            Eigen::Vector2d random_state;
            if (p < p_goal){
                random_state = problem.agent_properties[agentNum].q_goal;
            }
            else{
                random_state = Eigen::Vector2d(randomNum(bounds[0][0], bounds[0][1]), randomNum(bounds[1][0], bounds[1][1]));
            }

            // Get the nearest node
            amp::Node nearest_node = nearestNode(random_state);
            Eigen::Vector2d nearest_state = nodes[nearest_node];
            int nearest_time = times[nearest_node];

            // Try that direction going r distance
            Eigen::Vector2d new_state = r*(random_state - nearest_state).normalized() + nearest_state;
            int new_time = nearest_time + 1;

            // Now try connecting the nearest node to the random state
            // Check if the line between the two states is in collision
            if (!pointCollision(problem, new_state) && !lineCollisionDecentral(problem, nearest_state, new_state) && !pathCollision(path, new_state, nearest_state, new_time, nearest_time)){

                // Add new node to maps, heuristic and graph
                nodes[i] = new_state;
                times[i] = new_time;
                graph.connect(nearest_node, i, (nearest_state - new_state).norm());

                // Check if the new state is the goal state
                if ((new_state - problem.agent_properties[agentNum].q_goal).norm() < epsilon){

                    nodes[1] = problem.agent_properties[agentNum].q_goal;
                    times[1] = new_time + 1;
                    graph.connect(i, 1, (new_state - problem.agent_properties[agentNum].q_goal).norm());

                    // Backout path
                    pathCurr = getPath(1);

                    // Store path in multi path struct
                    path.agent_paths[agentNum] = pathCurr;
                    
                    // Store each node in a vector
                    for (int z = 0; z < pathCurr.waypoints.size(); z++){
                        currLocation[z].push_back(pathCurr.waypoints[z]);
                    }
                    // Also add the goal node to the vector, for pathCurr.waypoints to n
                    for (int z = pathCurr.waypoints.size(); z < n; z++){
                        currLocation[z].push_back(problem.agent_properties[agentNum].q_goal);
                    }

                    break;
                }
                // i++;
            }
            i++; 
            // if (test > 50000){
            //     amp::Visualizer::makeFigure(problem);
            //     break;
            // }
        }
        graph.clear();
        nodes.clear();
        times.clear();
    }

    return path;
}

amp::Node decentralRRT::nearestNode(Eigen::Vector2d state){
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

bool decentralRRT::pointCollision(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d point){
    
    // use ray casting to detect the closest obstacle
    Eigen::Vector2d currMin = rayDetect(problem, point);
    if (currMin != Eigen::Vector2d(INT16_MAX, INT16_MAX)){
        if ((currMin - point).norm() <= radius + 2*epsilon){
            return true;
        }
    }
    
    return false;
}

bool decentralRRT::lineCollisionDecentral(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d curr, Eigen::Vector2d next){

    // get all the obstacles
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
    // loop through all obstacles
    for (int i = 0; i < obstacles.size(); i++){
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
            // check if the line collides with any of the vertices
            if (lineCollision(curr, next, vertices[j%vertices.size()], vertices[(j+1)%vertices.size()])){
                return true;
            }
        }
    }
    return false;
}

bool decentralRRT::pathCollision(amp::MultiAgentPath2D path, Eigen::Vector2d next, Eigen::Vector2d nearest, int timeNext, int timeNear){

    // Get all the amp::Path2D's from the amp::MultiAgentPath2D
    std::vector<amp::Path2D> paths = path.agent_paths;
    if (paths[0].waypoints.size() == 0){
        return false;
    }

    // Now check the time of the point
    std::vector<Eigen::Vector2d> nextTimePoints = currLocation[timeNext];
    for (int i = 0; i < nextTimePoints.size(); i++){
        if ((nextTimePoints[i] - next).norm() <= 2*radius + 2*epsilon){
            return true;
        }
    }

    // // Now check the line line intersections between the next and nearest points
    // std::vector<Eigen::Vector2d> nearTimePoints = currLocation[timeNear];
    // for (int i = 0; i < nextTimePoints.size(); i++){
    //     Eigen::Vector2d dirVec = (nextTimePoints[i] - nearTimePoints[i]).normalized();
    //     if (lineCollision(next, nearest, nextTimePoints[i] + dirVec, nearTimePoints[i] - dirVec)){
    //         std::cout << "Line-line" << std::endl;
    //         std::cout << "next: " << next << std::endl;
    //         std::cout << "nearest: " << nearest << std::endl;
    //         std::cout << "nextTimePoints[i]: " << nextTimePoints[i] << std::endl;
    //         std::cout << "nearTimePoints[i]: " << nearTimePoints[i] << std::endl;

    //         return true;
    //     }
    // }    

    return false;
}

/// @brief Returns the path form the goal node to the init node
amp::Path2D decentralRRT::getPath(int end_node){

    // create the path node
    amp::Path2D path;

    // Start from the given node
    int curr_node = end_node;

    // Loop through all nodes until we get to the init node (0)
    while(curr_node != 0){
        path.waypoints.push_back(nodes[curr_node]);
        curr_node = graph.parents(curr_node)[0]; // just take first parent
    }

    // Add goal node
    path.waypoints.push_back(nodes[0]);

    // Reverse
    std::reverse(path.waypoints.begin(), path.waypoints.end());

    return path;
}

const amp::MultiAgentProblem2D decentralRRT::expand(const amp::MultiAgentProblem2D& origProblem){
    
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
        }
        // Now create a new obstacle
        amp::Obstacle2D new_obstacle(new_vertices);
        new_obstacles.push_back(new_obstacle);
    }
    // Now replace the obstacles in problem
    problem.obstacles = new_obstacles;
    // visualize
    // amp::Visualizer::makeFigure(problem);

    return problem;
}

std::tuple<bool, double, amp::MultiAgentPath2D> decentralRRT::planCompare(const amp::MultiAgentProblem2D& problem){

    // Create a timer
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Now call plan
    amp::MultiAgentPath2D path = plan(problem);

    // Stop the timer
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    // Calculate the success
    bool success = amp::HW8::check(path, problem);
    // bool success = path.agent_paths[0].waypoints.size() > 0;

    // Now calculate the computation time
    double computation_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

    // Return the tuple
    return std::make_tuple(success, computation_time, path);
}