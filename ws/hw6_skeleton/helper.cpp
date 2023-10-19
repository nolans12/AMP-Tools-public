#include "helper.h"

/// @brief Given a point in continuous space that is between the bounds, determine what cell (i, j) that the point is in and also assign the boolean collision
/// @param x0 Value of the first configuration space variable
/// @param x1 Value of the second configuration space variable
/// @return A pair (i, j) of indices that correspond to the cell that (x0, x1) is in
std::pair<std::size_t, std::size_t> MyGridCSpace::getCellFromPoint(double x0, double x1) const{
    // first create a linspace of possible x0 and x1s
    std::vector<double> x0s;
    std::vector<double> x1s;
    for (int i = 0; i < x0_cells; i++){
        x0s.push_back(x0_min + i*(x0_max-x0_min)/x0_cells);
    }
    for (int i = 0; i < x1_cells; i++){
        x1s.push_back(x1_min + i*(x1_max-x1_min)/x1_cells);
    }
    // now find which x0s and x1s point bounds x0 and x1
    int x0_index = 0;
    int x1_index = 0;
    for (int i = 0; i < x0s.size(); i++){
        if (x0s[i] <= x0 && x0s[i+1] >= x0){
            x0_index = i;
        }
    }
    for (int i = 0; i < x1s.size(); i++){
        if (x1s[i] <= x1 && x1s[i+1] >= x1){
            x1_index = i;
        }
    }
    // return the pair
    return std::make_pair(x0_index, x1_index);
}

/// @brief This function takes in a workspace (environment) and outputs the resulting gridcspace using point robot
std::unique_ptr<amp::GridCSpace2D> MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& environment){
// Initiate a blank grid
    double gridSize = 0.25;
    double x0_cells = (environment.x_max-environment.x_min)/gridSize;
    double x1_cells = (environment.y_max-environment.y_min)/gridSize;
    MyGridCSpace cspace = MyGridCSpace(x0_cells, x1_cells, environment.x_min, environment.x_max, environment.y_min, environment.y_max);

//  Now for the blank cspace, assign collisions using .operator()
    for (int i = 0; i < x0_cells; i++){
        for (int j = 0; j < x1_cells; j++){
            // get the current x0 and x1 values
            double x0 = environment.x_min + i*(environment.x_max-environment.x_min)/x0_cells;
            double x1 = environment.y_min + j*(environment.y_max-environment.y_min)/x1_cells;
            // check if in collision the current x0 and x1 values are in collision with any of the obstacles
            bool collision = cellCollision(environment, Eigen::Vector2d(x0,x1));
            if (collision == true){
                // get current cell pair
                std::pair<std::size_t, std::size_t> cell = cspace.getCellFromPoint(x0,x1);
                cspace.operator()(cell.first,cell.second) = true;
            }
        }
    }
    // Plot check
    amp::Visualizer::makeFigure(cspace);

// Now return the cspace
    std::unique_ptr<amp::GridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace>(cspace);
    return cspace_ptr;
}

/// @brief This function checks all the obstacles in the environment for a collision with xy position x0, x1
bool cellCollision(amp::Environment2D environment, Eigen::Vector2d point){
    std::vector<amp::Obstacle2D> obstacles = environment.obstacles;
    // loop through all obstacles
    for (int i = 0; i < obstacles.size(); i++){
        int count = 0;
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW(); // all vertices of the obstacle
        // loop through all vertices
        for (int j = 0; j < vertices.size(); j++){
                // check the normal angle of a line between each point
                Eigen::ParametrizedLine<double, 2> line(vertices[j], vertices[(j+1)%vertices.size()] - vertices[j]);
                Eigen::Vector2d normal(line.direction()[1], -line.direction()[0]);

                if ((normal.dot(point - vertices[j]) <= 0)){ // method for finding if a point is inside polygon
                    count++;
                }
        }
        // at the end, if count is equal to the vertices size, then in collision
        if (count == vertices.size()){
            return true;
        }
    }
    return false;
}

/// @brief Function which takes in a grid_cspace and forms a path from q_init to q_goal, using wave front 
amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    // first get stats of grid_cspace
    std::pair<double,double> x0_bound = grid_cspace.x0Bounds();
    std::pair<double,double> x1_bound = grid_cspace.x1Bounds();
    std::pair<int,int> size = grid_cspace.size();

    std::cout << x0_bound.first << " " << x0_bound.second << std::endl;
    std::cout << x1_bound.first << " " << x1_bound.second << std::endl;
    std::cout << size.first << " " << size.second << std::endl;

    // 2d array where the int value will be the wave front cost
    int wfgrid[(size.first)][(size.second)];

    // loop through grid_cspace and anywhere where the .operator is equal to true, set the wfgrid to be 1
    for (int i = 0; i < size.first; i++){
        for (int j = 0; j < size.second; j++){
            if (grid_cspace.operator()(i,j) == true){
                wfgrid[i][j] = 1;
            }
            else{
                wfgrid[i][j] = 0;
            }
        }
    }

    // for collision avoidance, first expand all 1 obstacles by 1 grid:
    for (int i = 0; i < size.first; i++){
        for (int j = 0; j < size.second; j++){
            // if the cell is equal to 1, then check all less incrementating neighbors (else makes all nodes obstacles, just want 1 pass through)
            if (wfgrid[i][j] == 1){
                // check all neighbors
                // check up left
                if (i-1 >= 0 && j-1 >= 0){
                    if (wfgrid[i-1][j-1] == 0){
                        wfgrid[i-1][j-1] = 1;
                    }
                }
                // check up
                if (j-1 >= 0){
                    if (wfgrid[i][j-1] == 0){
                        wfgrid[i][j-1] = 1;
                    }
                }
                // check left
                if (i-1 >= 0){
                    if (wfgrid[i-1][j] == 0){
                        wfgrid[i-1][j] = 1;
                    }
                }
            }
        }
    }

    // set the wfgrid value of the goal cell to be 2
    std::pair<std::size_t, std::size_t> init_cell = grid_cspace.getCellFromPoint(q_init[0],q_init[1]);
    std::pair<std::size_t, std::size_t> goal_cell = grid_cspace.getCellFromPoint(q_goal[0],q_goal[1]);
    wfgrid[init_cell.first][init_cell.second] = 0;
    wfgrid[goal_cell.first][goal_cell.second] = 2;

    // now start at the goal cell and do wave front
    int count = 2;
    bool done = false;
    while (done == false){
        // loop through all cells
        for (int i = 0; i < size.first; i++){
            for (int j = 0; j < size.second; j++){
                // if the cell is equal to count, then check all neighbors
                if (wfgrid[i][j] == count){
                    // check all neighbors
                    // // check up left
                    // if (i-1 >= 0 && j-1 >= 0){
                    //     if (wfgrid[i-1][j-1] == 0){
                    //         wfgrid[i-1][j-1] = count+1;
                    //     }
                    // }
                    // check up
                    if (j-1 >= 0){
                        if (wfgrid[i][j-1] == 0){
                            wfgrid[i][j-1] = count+1;
                        }
                    }
                    // // check up right
                    // if (i+1 < size.first && j-1 >= 0){
                    //     if (wfgrid[i+1][j-1] == 0){
                    //         wfgrid[i+1][j-1] = count+1;
                    //     }
                    // }
                    // check right
                    if (i+1 < size.first){
                        if (wfgrid[i+1][j] == 0){
                            wfgrid[i+1][j] = count+1;
                        }
                    }
                    // // check down right
                    // if (i+1 < size.first && j+1 < size.second){
                    //     if (wfgrid[i+1][j+1] == 0){
                    //         wfgrid[i+1][j+1] = count+1;
                    //     }
                    // }
                    // check down
                    if (j+1 < size.second){
                        if (wfgrid[i][j+1] == 0){
                            wfgrid[i][j+1] = count+1;
                        }
                    }
                    // // check down left
                    // if (i-1 >= 0 && j+1 < size.second){
                    //     if (wfgrid[i-1][j+1] == 0){
                    //         wfgrid[i-1][j+1] = count+1;
                    //     }
                    // }
                    // check left
                    if (i-1 >= 0){
                        if (wfgrid[i-1][j] == 0){
                            wfgrid[i-1][j] = count+1;
                        }
                    }
                }
            }
        }
        // check if the q_init cell is equal to count, aka start point has been reached
        if (wfgrid[init_cell.first][init_cell.second] == count){
            done = true;
        }
        // std::cout << count << std::endl;
        count++;
    }

    // now, we have a wave front grid, we need to find the path
    // start at the q_init cell, and find the neighbor with a value of current value - 1
    amp::Path2D path;
    path.waypoints.push_back(q_init);
    int currVal = wfgrid[init_cell.first][init_cell.second];
    std::pair<std::size_t, std::size_t> currCell = init_cell;
    while (currVal != 2){
        std::cout << "Current cell: " << currCell.first << " " << currCell.second << std::endl;
        std::cout << "Current value: " << currVal << std::endl;
        // // check up left
        // if (currCell.first-1 >= 0 && currCell.second-1 >= 0){ // not out of bounds
        //     if (wfgrid[currCell.first-1][currCell.second-1] == currVal-1){ // only one less
        //         // std::cout << "Found in up left" << std::endl;
        //         currCell.first = currCell.first-1;
        //         currCell.second = currCell.second-1;
        //         currVal = currVal-1;
        //         // add to path
        //         Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
        //         path.waypoints.push_back(currPoint);
        //     }
        // }
        // check up
        if (currCell.second-1 >= 0){ // not out of bounds
            if (wfgrid[currCell.first][currCell.second-1] == currVal-1){ // only one less
                // std::cout << "Found in up" << std::endl;
                currCell.second = currCell.second-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // // check up right
        // if (currCell.first+1 < size.first && currCell.second-1 >= 0){ // not out of bounds
        //     if (wfgrid[currCell.first+1][currCell.second-1] == currVal-1){ // only one less
        //         // std::cout << "Found in up right" << std::endl;
        //         currCell.first = currCell.first+1;
        //         currCell.second = currCell.second-1;
        //         currVal = currVal-1;
        //         // add to path
        //         Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
        //         path.waypoints.push_back(currPoint);
        //     }
        // }
        // check right
        if (currCell.first+1 < size.first){ // not out of bounds
            if (wfgrid[currCell.first+1][currCell.second] == currVal-1){ // only one less
                // std::cout << "Found in right" << std::endl;
                currCell.first = currCell.first+1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // // check down right
        // if (currCell.first+1 < size.first && currCell.second+1 < size.second){ // not out of bounds
        //     if (wfgrid[currCell.first+1][currCell.second+1] == currVal-1){ // only one less
        //         // std::cout << "Found in down right" << std::endl;
        //         currCell.first = currCell.first+1;
        //         currCell.second = currCell.second+1;
        //         currVal = currVal-1;
        //         // add to path
        //         Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
        //         path.waypoints.push_back(currPoint);
        //     }
        // }
         // check down
        if (currCell.second+1 < size.second){ // not out of bounds
            if (wfgrid[currCell.first][currCell.second+1] == currVal-1){ // only one less
                // std::cout << "Found in down" << std::endl;
                currCell.second = currCell.second+1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // // check down left
        // if (currCell.first-1 >= 0 && currCell.second+1 < size.second){ // not out of bounds
        //     if (wfgrid[currCell.first-1][currCell.second+1] == currVal-1){ // only one less
        //         // std::cout << "Found in down left" << std::endl;
        //         currCell.first = currCell.first-1;
        //         currCell.second = currCell.second+1;
        //         currVal = currVal-1;
        //         // add to path
        //         Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
        //         path.waypoints.push_back(currPoint);
        //     }
        // }
        // check left
        if (currCell.first-1 >= 0){ // not out of bounds
            if (wfgrid[currCell.first-1][currCell.second] == currVal-1){ // only one less
                // std::cout << "Found in left" << std::endl;
                currCell.first = currCell.first-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
    }
    path.waypoints.push_back(q_goal);
    return path;
}

amp::Path2D MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    // first get stats of grid_cspace

    std::pair<double,double> x0_bound = grid_cspace.x0Bounds();
    std::pair<double,double> x1_bound = grid_cspace.x1Bounds();
    std::pair<int,int> size = grid_cspace.size();

    // std::cout << "x0 Bounds: " << x0_bound.first << " " << x0_bound.second << std::endl;
    // std::cout << "x1 Bounds: " << x1_bound.first << " " << x1_bound.second << std::endl;
    // std::cout << "Size: " << size.first << " " << size.second << std::endl;
    // std::cout << "Initial Point: " << q_init[0] << " " << q_init[1] << std::endl;
    // std::cout << "Goal Point: " << q_goal[0] << " " << q_goal[1] << std::endl;

    // 2d array where the int value will be the wave front cost
    int wfgrid[(size.first)][(size.second)];

    // loop through grid_cspace and anywhere where the .operator is equal to true, set the wfgrid to be 1
    for (int i = 0; i < size.first; i++){
        for (int j = 0; j < size.second; j++){
            if (grid_cspace.operator()(i,j) == true){
                // std::cout << "Found obstacle at x0: " << i << " x1: " << j << std::endl; 
                wfgrid[i][j] = 1;
            }
            else{
                wfgrid[i][j] = 0;
            }
        }
    }

    // // for collision avoidance, first expand all 1 obstacles by 1 grid:
    // for (int i = 0; i < size.first; i++){
    //     for (int j = 0; j < size.second; j++){
    //         // if the cell is equal to 1, then check all less incrementating neighbors (else makes all nodes obstacles, just want 1 pass through)
    //         if (wfgrid[i][j] == 1){
    //             // check all neighbors
    //             // check up left
    //             if (i-1 >= 0 && j-1 >= 0){
    //                 if (wfgrid[i-1][j-1] == 0){
    //                     wfgrid[i-1][j-1] = 1;
    //                 }
    //             }
    //             // check up
    //             if (j-1 >= 0){
    //                 if (wfgrid[i][j-1] == 0){
    //                     wfgrid[i][j-1] = 1;
    //                 }
    //             }
    //             // check left
    //             if (i-1 >= 0){
    //                 if (wfgrid[i-1][j] == 0){
    //                     wfgrid[i-1][j] = 1;
    //                 }
    //             }
    //         }
    //     }
    // }

    // set the wfgrid value of the goal cell to be 2
    std::pair<std::size_t, std::size_t> init_cell = grid_cspace.getCellFromPoint(q_init[0],q_init[1]);
    std::pair<std::size_t, std::size_t> goal_cell = grid_cspace.getCellFromPoint(q_goal[0],q_goal[1]);
    // std::cout << "Initial Cell: " << init_cell.first << " " << init_cell.second << std::endl;
    // std::cout << "Goal Cell: " << goal_cell.first << " " << goal_cell.second << std::endl;
    wfgrid[init_cell.first][init_cell.second] = 0;
    wfgrid[goal_cell.first][goal_cell.second] = 2;

    // now start at the goal cell and do wave front
    int count = 2;
    bool done = false;
    while (done == false){
        // loop through all cells
        for (int i = 0; i < size.first; i++){
            for (int j = 0; j < size.second; j++){
                // if the cell is equal to count, then check all neighbors
                if (wfgrid[i][j] == count){
                // need to adjust this function to also be able to wrap around the grid
                    // check up
                    if (wfgrid[i][(j+size.second-1)%size.second] == 0){
                        wfgrid[i][(j+size.second-1)%size.second] = count+1;
                    }
                   
                    // check right
                    if (wfgrid[(i+size.first+1)%size.first][j] == 0){
                        wfgrid[(i+size.first+1)%size.first][j] = count+1;
                    }
                   
                    // check down
                    if (wfgrid[i][(j+size.second+1)%size.second] == 0){
                        wfgrid[i][(j+size.second+1)%size.second] = count+1;
                    }
                 
                    // check left
                    if (wfgrid[(i+size.first-1)%size.first][j] == 0){
                        wfgrid[(i+size.first-1)%size.first][j] = count+1;
                    }
                }
            }
        }
        // check if the q_init cell is equal to count, aka start point has been reached
        if (wfgrid[init_cell.first][init_cell.second] == count){
            done = true;
        }
        // std::cout << count << std::endl;
        count++;
    }

    // now, we have a wave front grid, we need to find the path
    // start at the q_init cell, and find the neighbor with a value of current value - 1
    amp::Path2D path;
    // path.waypoints.push_back(q_init);
    path.waypoints.push_back(Eigen::Vector2d(M_PI,0));
    int currVal = wfgrid[init_cell.first][init_cell.second];
    std::pair<std::size_t, std::size_t> currCell = init_cell;
    while (currVal != 2){
        // std::cout << "Current cell: " << currCell.first << " " << currCell.second << std::endl;
        // std::cout << "Current value: " << currVal << std::endl;

        // check up
        if (currCell.second-1 >= 0){
            if (wfgrid[currCell.first][currCell.second-1] == currVal-1){ // only one less
                currCell.second = currCell.second-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
       
        // check right
        if (currCell.first+1 < size.second){
            if (wfgrid[currCell.first+1][currCell.second] == currVal-1){ // only one less
                currCell.first = currCell.first+1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }

         // check down
        if (currCell.second+1 < size.second){
            if (wfgrid[currCell.first][currCell.second+1] == currVal-1){ // only one less
                currCell.second = currCell.second+1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        
        // check left
        if (currCell.first-1 >= 0){ // not out of bounds
            if (wfgrid[currCell.first-1][currCell.second] == currVal-1){ // only one less
                currCell.first = currCell.first-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first + 0.5*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second  + 0.5*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
    // now also check all edge cases and deal with wrapping
        // check up edge
        if (currCell.second-1 < 0){ // need to loop back second to size - 1
            if (wfgrid[currCell.first][size.second-1] == currVal-1){ // only one less
                currCell.second = size.second-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // check down edge
        if (currCell.second+1 >= size.second){ // need to loop back second to 0
            if (wfgrid[currCell.first][0] == currVal-1){ // only one less
                currCell.second = 0;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // check edge left
        if (currCell.first-1 < 0){ // need to loop back first to size - 1
            if (wfgrid[size.first-1][currCell.second] == currVal-1){ // only one less
                currCell.first = size.first-1;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
        // check edge right
        if (currCell.first+1 >= size.first){ // need to loop back first to 0
            if (wfgrid[0][currCell.second] == currVal-1){ // only one less
                currCell.first = 0;
                currVal = currVal-1;
                // add to path
                Eigen::Vector2d currPoint = Eigen::Vector2d(x0_bound.first + currCell.first*(x0_bound.second-x0_bound.first)/size.first, x1_bound.first + currCell.second*(x1_bound.second-x1_bound.first)/size.second);
                path.waypoints.push_back(currPoint);
            }
        }
    }
    // std::cout << q_goal.x() << " " << q_goal.y() << std::endl;
    path.waypoints.push_back(Eigen::Vector2d(0,0));
    return path;
}

/// @brief This function computes the grid c space given an environment and linkLengths for a 2 dof manipulator
std::unique_ptr<amp::GridCSpace2D> MyCSpaceCtor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
    double x0_min = env.x_min;
    double x0_max = env.x_max;
    double x1_min = env.y_min;
    double x1_max = env.y_max;
    int density = 100; // density of grid

    amp::ManipulatorState state;
    state.push_back(0);
    state.push_back(0);

    // plot the workspace environment
    amp::Visualizer::makeFigure(env, manipulator, state);

    // create a 2dLink class based on manipulator, using getLinkLengths
    Link2d link = Link2d(manipulator.getLinkLengths());

  
    // create c-space
    double x0_min_c = 0;
    double x0_max_c = 2*M_PI;
    double x1_min_c = 0;
    double x1_max_c = 2*M_PI;
    MyGridCSpace c(density, density, x0_min_c, x0_max_c, x1_min_c, x1_max_c);

    amp::DenseArray2D collisions(density,density);

    // now we need to check for collisions, loop through all cells
    for (int i = 0; i < density; i++){
        for (int j = 0; j < density; j++){
            // get the current t0 and t1 values
            double t0 = x0_min_c + i*(x0_max_c-x0_min_c)/density;
            double t1 = x1_min_c + j*(x1_max_c-x1_min_c)/density;

            // check if in collision with any one of the 3 vertices, or between!
            bool collision = inCollisionCheck(t0,t1, env, link);
            if (collision == true){
                c.operator()(i,j) = true;
            }
        }
    }
    // make a pointer
    std::unique_ptr<amp::GridCSpace2D> cspace = std::make_unique<MyGridCSpace>(c);
    return cspace;
}

bool inCollisionCheck(double x0, double x1, amp::Environment2D environment, Link2d link){ 
        amp::ManipulatorState state; // joint angles
        state.push_back(x0);
        state.push_back(x1);

        // get foward kinematics of all vertices, then check if collide
        Eigen::Vector2d v0 = link.getJointLocation(state,0);
        Eigen::Vector2d v1 = link.getJointLocation(state,1);
        Eigen::Vector2d v2 = link.getJointLocation(state,2);

        // now, find if the line between v0 and v1 and v1 and v2 collides with any of the obstacles
        for (int i = 0; i < environment.obstacles.size(); i++){
            amp::Polygon currObs = environment.obstacles[i];
            // get vertices of the obstacle
            std::vector<Eigen::Vector2d> obsVertices = currObs.verticesCCW();
            // now, for each vertice, check if it collides with the line between v0 and v1 and v1 and v2
            for (int j = 0; j < obsVertices.size(); j++){
                // check if the line between v0 and v1 and v1 and v2 collides with the line between obsVertices[j] and obsVertices[j+1]
                if (intersect(v0,v1,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()]) || intersect(v1,v2,obsVertices[j%obsVertices.size()],obsVertices[(j+1)%obsVertices.size()])){
                    return true; // if happens at all, return true
                }
            }
        }
        return false;
}

