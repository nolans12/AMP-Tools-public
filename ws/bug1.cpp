#include "bug1.h"
#include "helperFuncs.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D bug1::plan(const amp::Problem2D& problem){

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init); // start point

    ////////////////////////////////////////////
    // Define all needed parameters
    ////////////////////////////////////////////
    double stepSize = 0.01; //((problem.q_goal - problem.q_init).norm())/5000; 
    double positionalError = 0.1; // defines how close can be to the goal/point before a success is declared
    bool goalReached = false; // go until goal is reached
    Eigen::Vector2d currPos; // current position
    Eigen::Vector2d nextPos; // next position
    Eigen::Vector2d contactLocation; // contact position
    amp::Obstacle2D currOb; // current obstacle
    Eigen::Vector2d rightVert;
    Eigen::Vector2d leftVert; 
    int hitObstacle; // index of obstacle hit
    int currLeftVert; // index of current left vertice
    int newHitObstacle; 
    bool intersect;
    int minIndex;
    int numCurrVerts;
    int numHitVerts;
    amp::Path2D contactPath; // path from contact point to goal
    std::vector<double> distToGoal; // array for storing distance to goal at each step once contact object
    int mode1Count = 0;
    int mode2Count = 0;

    // define mode description
    // mode = 1: moving towards goal
    // mode = 2: circumnavigating obstacle
    // mode = 3: moving back to closest distance to goal
    int mode = 1; 

    // output step size
    std::cout << "The step size is: " << stepSize << std::endl;
    std::cout << "Motion to goal..." << std::endl;
    // begin loop, will run until goal is reached
    while (goalReached == false){ 

        while (mode == 1){
        ////////////////////////////////////////////
        // Motion to goal movement
        ////////////////////////////////////////////

            currPos = path.waypoints.back();

            mode1Count++;
            mode2Count = 0;
            if (mode1Count > 1000000){
                return path; // infinite loop
            }

        // have you reached goal?
            if ((problem.q_goal - currPos).norm() < positionalError) {
                goalReached = true;
                path.waypoints.push_back(problem.q_goal); // add goal to path
                std::cout << "Goal reached!" << std::endl;
                return path; // end program
            }

        // if not, move forward towards goal
            nextPos = currPos + stepSize * getDirectionVector(currPos, problem.q_goal); // project forward towards goal

            Eigen::Vector2d currDir = getDirectionVector(currPos, nextPos); // get direction vector to goal

        // have you now hit an obstacle? move backwards to prior position

            hitObstacle = obstacleIndex(problem.obstacles, nextPos); // check for collision 

            // obstacle is defined as hit if the index is not -1!
            // if this is the case, we need to backtrack, move to mode 2, where we follow obstacle
            if (hitObstacle > -1){
 
                currOb = problem.obstacles[hitObstacle]; // define the obstacle we are currently following, use hit obstacle
                // find the vertices we are following in that obstacle
                numCurrVerts = currOb.verticesCW().size();

                // now find the left and right vertices that were hit by obstacleIndex!
                Eigen::Vector2d intersectionBest = Eigen::Vector2d(INT_MAX,INT_MAX);
                Eigen::Vector2d intersectionCurr;
                for (int i = 0; i < numCurrVerts+1; i++){
                    // get the current x,y intersection point
                    intersectionCurr = intersectionPt(currPos, nextPos, currOb.verticesCW()[i%numCurrVerts], currOb.verticesCW()[(i+1)%numCurrVerts]);
                    if ((intersectionCurr-currPos).norm() <= (intersectionBest-currPos).norm()){ // this distance from current pt is smaller
                        intersectionBest = intersectionCurr; // update the best intersection point
                        rightVert = currOb.verticesCW()[i%numCurrVerts]; // find the right and left vertices that you connected with
                        leftVert = currOb.verticesCW()[(i+1)%numCurrVerts]; 
                        currLeftVert = (i+1)%numCurrVerts; // left is i+1 index
                    }
                }

                // should now have right and left vertices that were hit!
                mode = 2; // did hit obstacle, go into circumnavigate mode
                numHitVerts = 0;
                contactLocation = currPos; // contact location is the point where the robot first hit the obstacle, we will come back to this point
                // empty contactPath and distToGoal
                contactPath.waypoints.clear();
                distToGoal.clear();

                //contactPath.waypoints.push_back(currPos); // store contact path
                std::cout << "Collision..." << std::endl;
                std::cout << "Hit obstacle at: " << currPos.x() << ", " << currPos.y() << std::endl;
                std::cout << "We are now following the obstacle in index: " << hitObstacle << std::endl;
                std::cout << "Obj " << hitObstacle << ", right vertice x is: " << rightVert.x() << " the right vertice y is: " << rightVert.y() << std::endl;
                std::cout << "Obj " << hitObstacle << ", left vertice x is: " << leftVert.x() << " the left vertice y is: " << leftVert.y() << std::endl;
            }
            else {
            // Didn't hit obstacle? officially move forward 
                path.waypoints.push_back(nextPos); 
            }
        }
        while (mode == 2){
        ////////////////////////////////////////////
        // Circumnavigate obstacle movement, left turning
        ////////////////////////////////////////////
        
        // Get current position
            currPos = path.waypoints.back(); 

        mode1Count = 0;
        mode2Count++;
        if (mode2Count > 1000000){
            return path; // infinite loop
        }

        // Have you reached goal?
            if ((problem.q_goal - currPos).norm() < positionalError) {
                goalReached = true;
                path.waypoints.push_back(problem.q_goal); // add goal to final point
                std::cout << "Goal reached!" << std::endl;
                return path; // end program
            }

        // Have you reached the contact point again?
            if ((contactLocation - currPos).norm() < positionalError && numHitVerts > 0){ // has passed at least one vert, by definition to not instantly register as contactPt at step 1
                mode = 3; // switch modes
                std::cout << "Have reached contact point again..." << std::endl;
                std::cout << "We are now in mode 3, current position is: " << currPos.x() << ", " << currPos.y() << std::endl;
                // also find the closest distance to goal
                double minDist = INT_MAX;
                int currIndex = 0;
                for (int i = 0; i < distToGoal.size(); i++){
                    if (distToGoal[i] < minDist){
                        minDist = distToGoal[i];
                        currIndex = i;
                    }
                }
                minIndex = currIndex;
            }else{

        // Propagate motion along wall
            Eigen::Vector2d dirObj = getDirectionVector(rightVert, leftVert); // current direction of velocity
            nextPos = currPos + stepSize * dirObj; // next position trying to goto

        // Check to see if a new obstacle is hit

            newHitObstacle = obstacleIndex(problem.obstacles, nextPos); // check for collision 
            //newHitObstacle = intersectionTest(currPos, nextPos, problem.obstacles); // check for collision 

            // -1 if no collision, otherwise index of obstacle hit
            if (newHitObstacle > -1){ 
            // Hit a new obstacle!, need to switch to circumnavigate this obstacle

                hitObstacle = newHitObstacle; // now define the new obstacle to circumnavigate
                currOb = problem.obstacles[hitObstacle]; // define the obstacle we are currently following, use hit obstacle
                numCurrVerts = currOb.verticesCW().size(); // find the vertices we are following in that obstacle
                //for each vert, check to see if doIntersect is true

               // now find the left and right vertices that were hit by obstacleIndex!
                Eigen::Vector2d intersectionBest = Eigen::Vector2d(INT_MAX,INT_MAX);
                Eigen::Vector2d intersectionCurr;
                for (int i = 0; i < numCurrVerts+1; i++){
                    // get the current x,y intersection point
                    intersectionCurr = intersectionPt(currPos, nextPos, currOb.verticesCW()[i%numCurrVerts], currOb.verticesCW()[(i+1)%numCurrVerts]);
                    if ((intersectionCurr-currPos).norm() <= (intersectionBest-currPos).norm()){ // this distance from current pt is smaller
                        intersectionBest = intersectionCurr; // update the best intersection point
                        rightVert = currOb.verticesCW()[i%numCurrVerts]; // find the right and left vertices that you connected with
                        leftVert = currOb.verticesCW()[(i+1)%numCurrVerts]; 
                        currLeftVert = (i+1)%numCurrVerts; // left is i+1 index
                    }
                }

                std::cout << "Collision... at x: " << currPos.x() << " y: " << currPos.y() << std::endl;
                std::cout << "We are now following the obstacle in index: " << hitObstacle << std::endl;
                std::cout << "Obj " << hitObstacle << ", right vertice x is: " << rightVert.x() << " the right vertice y is: " << rightVert.y() << std::endl;
                std::cout << "Obj " << hitObstacle << ", left vertice x is: " << leftVert.x() << " the left vertice y is: " << leftVert.y() << std::endl;
            }else{ 
            // No new obstacle collision is hit! Its okay to continue moving along wall

                // we will define the needed dist from currPos to rightVert to be sqrt(2)*stepSize more than leftVert to rightVert
                // check this
                double distToLeft = (leftVert - rightVert).norm();
                double distToCurr = (nextPos - rightVert).norm();

                // if distToCurr is greater than distToLeft by sqrt2*stepSize, we are far enough away to switch to new set of points
                if (distToCurr > (distToLeft + 1.414*stepSize)){
                // We have rounded corner of a vertice, need to rotate cw to new set of vertices!

                    // define new right and left vertices
                    rightVert = leftVert; // new rightVert is prior leftVert
                    // get new leftVert
                    if (currLeftVert == (currOb.verticesCW().size()-1)){ // edge case for next postion is going to reset
                        leftVert = currOb.verticesCW()[0];
                        currLeftVert = 0; // update currLeftVert
                    }else{
                        leftVert = currOb.verticesCW()[currLeftVert+1];
                        currLeftVert = currLeftVert + 1; // update currLeftVert
                    }
                    
                    numHitVerts++; // update the num of verts hit
                    path.waypoints.push_back(nextPos); // propagate that next position! Success!

                    // Output to console the updated rounding corner information
                    std::cout << "Rounded corner... current position is" << " x: " << currPos.x() << " y: " << currPos.y() << "... the new line we are following is: " << std::endl;
                    std::cout << "Obj " << hitObstacle << ", right vertice x is: " << rightVert.x() << " the right vertice y is: " << rightVert.y() << std::endl;
                    std::cout << "Obj " << hitObstacle << ", left vertice x is: " << leftVert.x() << " the left vertice y is: " << leftVert.y() << std::endl;
                    std::cout << "Obj " << hitObstacle << ", direction of motion is: " << dirObj.x() << " x and " << dirObj.y() << " y." << std::endl;
                }else{
                // If we have not rounded corner of vertice, just keep going in the same direction!

                    path.waypoints.push_back(nextPos); // propagate that next position! Success!
                    distToGoal.push_back((problem.q_goal - currPos).norm()); // store distance to goal
                    contactPath.waypoints.push_back(nextPos); // store contact path
                }
            }
            }
        }
        while (mode == 3){ // move back to closest distance to goal

            // we have the minIndex where the minimum is, now we need to move back to that point
            // lets find that minimum index
            // find the shortest path to it!
            amp::Path2D path1;
            amp::Path2D path2;
            // one will loop from 0 to size of path looking for minIndex, other will loop from size of path to 0
            // whichever is shorter is the one we will use
            int check1;
            int check2;
            for (int i = 0; i < contactPath.waypoints.size(); i++){
                if (i == minIndex){
                    // we have found the path!
                    path1.waypoints.push_back(contactPath.waypoints[i]);
                    check1 = path1.waypoints.size();
                    break;
                }else{
                    path1.waypoints.push_back(contactPath.waypoints[i]);
                }
            }
            //std:reverse(contactPath.waypoints.begin(), contactPath.waypoints.end());
            for (int i = (contactPath.waypoints.size()-1); i > 0; i--){ // reverse for loop
                if (i == minIndex){
                    // we have found the path!
                    path2.waypoints.push_back(contactPath.waypoints[i]);
                    check2 = path2.waypoints.size();
                    break;
                }else{
                    path2.waypoints.push_back(contactPath.waypoints[i]);
                }
            }
            // take minimum path
            if (check1 < check2){
                for (int i = 0; i < path1.waypoints.size(); i++){
                    path.waypoints.push_back(path1.waypoints[i]);
                }
            }else{
                for (int i = 0; i < path2.waypoints.size(); i++){
                    path.waypoints.push_back(path2.waypoints[i]);
                }
            }
            Eigen::Vector2d test = path.waypoints.back();
            std::cout << "Motion to goal..." << std::endl;
            mode = 1;
        }
    }
}
