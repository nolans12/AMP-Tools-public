#pragma once

#include "AMPCore.h"
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