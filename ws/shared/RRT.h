#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "HelpfulClass.h"
#include "helper.h"
#include "helperFuncs.h"
#include "newHelper.h"
#include "AStar.h"


class MyRRT2D : amp::GoalBiasRRT2D {
    public:
        MyRRT2D(){
            this->n = 10000;
            this->r = 1;
            this->p_goal = 0.1;
            this->epsilon = 0.25;
        }

        MyRRT2D(Eigen::Vector2d x_bound, Eigen::Vector2d y_bound, double n, double r, double p_goal, double epsilon){
            this->n = n;
            this->r = r;
            this->p_goal = p_goal;
            this->epsilon = epsilon;
            this->bounds.push_back(x_bound);
            this->bounds.push_back(y_bound);
        }

        /// @brief Solves and returns a 2d path for the problem
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        /// @brief Returns nearest node to the given state
        amp::Node nearestNode(Eigen::Vector2d state);

        /// @brief Solves and returns success, path length, and compuatation time
        std::tuple<bool, double, double> planCompare(const amp::Problem2D& problem);

    private:
        std::vector<Eigen::Vector2d> bounds; // Bounds of the map, can be N-dimensional of 2D
        double n; // Sample count
        double r; // Extend parameter
        double p_goal; // Probability of sampling the goal
        double epsilon; // Step size

        std::map<amp::Node, Eigen::Vector2d> nodes; // Map of nodes to their state, ONLY 2D FOR NOW
        amp::Graph<double> graph; // Graph of nodes
        amp::LookupSearchHeuristic heur; // Heuristic value for A*
};

class centralRRT : public amp::CentralizedMultiAgentRRT {
    public:
        centralRRT(){
            // Default parameters
            this->n = 75000;
            this->r = 0.5;
            this->p_goal = 0.05;
            this->epsilon = 0.25;
        }

        centralRRT(double n, double r, double p_goal, double epsilon){
            this->n = n;
            this->r = r;
            this->p_goal = p_goal;
            this->epsilon = epsilon;
        }

        /// @brief Solves and returns a path for the multi agent problem
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

        const amp::MultiAgentProblem2D expand(const amp::MultiAgentProblem2D& origProblem);
    
        /// @brief Returns nearest node to the given state
        amp::Node nearestNode(std::vector<Eigen::Vector2d> state);

        bool multiPointCollision(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points);

        bool multiLineCollision(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> curr, std::vector<Eigen::Vector2d> next);

        bool multiGoalCheck(const amp::MultiAgentProblem2D& problem, std::vector<Eigen::Vector2d> points);

        std::vector<amp::Path2D> getPath(int end_node);

        // /// @brief Solves and returns success, path length, and compuatation time
        std::tuple<bool, double, double> planCompare(const amp::MultiAgentProblem2D& problem);

        int getTreeSize(){
            return nodes.size();
        }

    private:
        double n; // Sample count
        double r; // Extend parameter
        double p_goal; // Probability of sampling the goal
        double epsilon; // Step size

        double radius; // Radius of the robots
        double m; // Number of robots

        std::vector<Eigen::Vector2d> bounds; // Bounds of the map, can be N-dimensional of 2D
        std::map<amp::Node, std::vector<Eigen::Vector2d>> nodes; // Map of nodes to their state, Is a vector of 2D states for each robot
        amp::Graph<double> graph; // Graph of nodes
};

class decentralRRT : public amp::DecentralizedMultiAgentRRT{
     public:
        decentralRRT(){
            this->n = 75000;
            this->r = 0.5;
            this->p_goal = 0.05;
            this->epsilon = 0.25;
        }

        decentralRRT(double n, double r, double p_goal, double epsilon){
            this->n = n;
            this->r = r;
            this->p_goal = p_goal;
            this->epsilon = epsilon;
        }

        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& origProblem) override;

        /// @brief Returns nearest node to the given state
        amp::Node nearestNode(Eigen::Vector2d state);

        bool pointCollision(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d point);

        bool lineCollisionDecentral(const amp::MultiAgentProblem2D& problem, Eigen::Vector2d curr, Eigen::Vector2d next);

        bool pathCollision(amp::MultiAgentPath2D path, Eigen::Vector2d next, Eigen::Vector2d nearest, int timeNew, int timeNear);

        amp::Path2D getPath(int end_node);

        const amp::MultiAgentProblem2D expand(const amp::MultiAgentProblem2D& origProblem);

        /// @brief Solves and returns success, path length, and compuatation time
        std::tuple<bool, double, double> planCompare(const amp::MultiAgentProblem2D& problem);

        // virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    private:
        double n; // Sample count
        double r; // Extend parameter
        double p_goal; // Probability of sampling the goal
        double epsilon; // Step size

        double radius; // Radius of the robots
        double m; // Number of robots

        std::vector<Eigen::Vector2d> bounds; // Bounds of the map, can be N-dimensional of 2D
        std::map<amp::Node, Eigen::Vector2d> nodes; // Map of nodes to their state, Is a vector of 2D states for each robot
        std::map<amp::Node, int> times; // Map from node to time of a robot
        std::map<int, std::vector<Eigen::Vector2d>> currLocation; // Map from time to current location of each robot
        amp::Graph<double> graph; // Graph of nodes
};