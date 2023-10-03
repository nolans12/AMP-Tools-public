#include "bug2.h"
#include "helperFuncs.h"

amp::Path2D bug2::plan(const amp::Problem2D& problem){

    // Define path and put start point in
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
    amp::Obstacle2D currOb; // current obstacle
    Eigen::Vector2d rightVert;
    Eigen::Vector2d leftVert; 
    Eigen::Vector2d dirObj;
    int hitObstacle; // index of obstacle hit
    int newHitObstacle;
    int currLeftVert; // index of current left vertice
    double contactDist;
    int numCurrVerts;
    int numHitVerts;
    Eigen::Vector2d currDir; // current direction of velocity
    int mode1Count = 0;
    int mode2Count = 0;

    // find the direction vector from q_int to q_goal
    Eigen::Vector2d mDirection = getDirectionVector(problem.q_init, problem.q_goal);

    // 2 modes
    // mode = 1, motion towards goal
    // mode = 2, motion around obstacle, until detects m-line that is closer than obstacle
    int mode = 1;

    // output step size
    std::cout << "The step size is: " << stepSize << std::endl;
    std::cout << "Motion to goal..." << std::endl;

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
            if ((problem.q_goal - currPos).norm() < positionalError){
                goalReached = true;
                path.waypoints.push_back(problem.q_goal); // add goal to final point
                std::cout << "Goal reached!" << std::endl;
                return path; // end program
            }

        // if not, move toward a step towards goal
            currDir = getDirectionVector(currPos, problem.q_goal);
            nextPos = currPos + stepSize * currDir; // project forward towards goal
            
        // have you now hit an obstacle? move backwards to prior position if so

            hitObstacle = obstacleIndex(problem.obstacles, nextPos); // check for collision
            if (hitObstacle > -1){ // obstacle has been hit!

                currOb = problem.obstacles[hitObstacle]; // get obstacle
                numCurrVerts = currOb.verticesCW().size(); // number of vertices in obstacle
                numHitVerts = 0;

                // now find the left and right vertices that were hit by obstacleIndex!
                Eigen::Vector2d intersectionBest = Eigen::Vector2d(INT_MAX,INT_MAX);
                Eigen::Vector2d intersectionCurr;
                for (int i = 0; i < numCurrVerts; i++){
                    // get the current x,y intersection point
                    intersectionCurr = intersectionPt(currPos, nextPos, currOb.verticesCW()[i], currOb.verticesCW()[(i+1)%numCurrVerts]);
                    if ((intersectionCurr-currPos).norm() <= (intersectionBest-currPos).norm()){ // this distance from current pt is smaller
                        intersectionBest = intersectionCurr; // update the best intersection point
                        rightVert = currOb.verticesCW()[i]; // find the right and left vertices that you connected with
                        leftVert = currOb.verticesCW()[(i+1)%numCurrVerts]; 
                        currLeftVert = (i+1)%numCurrVerts; // left is i+1 index
                    }
                }

                // should now have right and left vertices that were hit!
                mode = 2; // did hit obstacle, go into circumnavigate mode
                contactDist = (problem.q_goal - currPos).norm(); // dist to goal of contact point, will look for m-line closer

                std::cout << "Collision... at x: " << currPos.x() << " y: " << currPos.y() << std::endl;
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
        // Circumnavigate obstacle, looking for m-line, left turning
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

        // Propagate motion along wall
            dirObj = getDirectionVector(rightVert, leftVert); // current direction of velocity
            nextPos = currPos + stepSize * dirObj; // next position trying to goto

        // Have you reached m-line again?
        // do this in some hi fidelity way
            if ((intersectionPt(currPos, nextPos, problem.q_init, problem.q_goal)-currPos).norm() < positionalError && ((problem.q_goal - nextPos).norm() < contactDist) && numHitVerts > 0){
                mode = 1; // go back to motion to goal
                std::cout << "Found m-line at the position x: "<< nextPos.x() << " y: "<< nextPos.y() << " going back to motion to goal..." << std::endl;
                numHitVerts = 0; // reset numHitVerts
                //path.waypoints.push_back(nextPos); // add the m-line point, i believe errors if include this, maybe average better
            }else{

        // Have you hit another obstacle?
            newHitObstacle = obstacleIndex(problem.obstacles, nextPos); // check for collision 
           
            if (newHitObstacle > -1){ 
                // Hit a new obstacle!, need to switch to circumnavigate this obstacle
                hitObstacle = newHitObstacle; // neeeded for terminal outputs

                currOb = problem.obstacles[hitObstacle]; // define the obstacle we are currently following, use hit obstacle
                numCurrVerts = currOb.verticesCW().size(); // find the vertices we are following in that obstacle
                //for each vert, check to see if doIntersect is true


                // now find the left and right vertices that were hit by obstacleIndex!
                Eigen::Vector2d intersectionBest = Eigen::Vector2d(INT_MAX,INT_MAX);
                Eigen::Vector2d intersectionCurr;
                for (int i = 0; i < numCurrVerts; i++){
                    // get the current x,y intersection point
                    intersectionCurr = intersectionPt(currPos, nextPos, currOb.verticesCW()[i], currOb.verticesCW()[(i+1)%numCurrVerts]);
                    if ((intersectionCurr-currPos).norm() <= (intersectionBest-currPos).norm()){ // this distance from current pt is smaller
                        intersectionBest = intersectionCurr; // update the best intersection point
                        rightVert = currOb.verticesCW()[i]; // find the right and left vertices that you connected with
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

                    // Output to console the updated rounding corner information
                    std::cout << "Rounded corner... current position is" << " x: " << currPos.x() << " y: " << currPos.y() << "... the new line we are following is: " << std::endl;
                    std::cout << "Obj " << hitObstacle << ", right vertice x is: " << rightVert.x() << " the right vertice y is: " << rightVert.y() << std::endl;
                    std::cout << "Obj " << hitObstacle << ", left vertice x is: " << leftVert.x() << " the left vertice y is: " << leftVert.y() << std::endl;

                    path.waypoints.push_back(nextPos);
                    numHitVerts++; // this is just to ensure at least one veritice is passed before swtiching to motion to goal
                }else{
                // If we have not rounded corner of vertice, just keep going in the same direction!
                    path.waypoints.push_back(nextPos); // propagate that next position! Success!
                }
            }
        }
        }
    }
}