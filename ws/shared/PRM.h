#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW6.h"
#include "HelpfulClass.h"
#include "helper.h"
#include "helperFuncs.h"
#include "newHelper.h"
#include "AStar.h"

class GenericPRM{
    public:
        /// @brief Solves a path for a generic N-dimension prm problem
        amp::Path planxd(Eigen::VectorXd& init_state, Eigen::VectorXd& goal_state, const amp::Problem2D& problem);

        /// @brief Samples the map for a generic N-dimension prm problem
        void sampleSpace(std::map<amp::Node, Eigen::VectorXd>& node_map, const amp::Problem2D& problem);

    public:
        std::vector<Eigen::Vector2d> bounds; // Bounds of the map, can be N-dimensional of 2D
        int n; // Sample count
        double r; // Neighborhood parameter
        std::map<amp::Node, Eigen::VectorXd> nodes; // Map of nodes to their state
        amp::Graph<double> graph; // Graph of nodes
        amp::LookupSearchHeuristic heur; // Heuristic value for A*
};

class MyPRM2D : public amp::PRM2D, public GenericPRM{
    public:
        MyPRM2D(){
            this->n = 1000;
            this->r = 2;
        }

        /// @brief Constructor
        MyPRM2D(Eigen::Vector2d x_bound, Eigen::Vector2d y_bound, int n, double r){
            this->bounds.push_back(x_bound);
            this->bounds.push_back(y_bound);
            this->n = n;
            this->r = r;
        }

        /// @brief Solves and returns a 2d path for the problem
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        /// @brief Solves and returns success, path length, and compuatation time
        std::tuple<bool, double, double, amp::Path2D> planCompare(const amp::Problem2D& problem);

};