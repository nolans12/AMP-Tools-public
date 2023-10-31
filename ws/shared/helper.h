#pragma once

#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"
#include "hw/HW2.h"
#include "Eigen/Geometry"
#include "HelpfulClass.h"
#include <queue>

using namespace amp;

class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                this->x0_cells = x0_cells; // assign private variables
                this->x1_cells = x1_cells;
                this->x0_min = x0_min;
                this->x0_max = x0_max;
                this->x1_min = x1_min;
                this->x1_max = x1_max;
            }

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const;

    private:
        std::size_t x0_cells;
        std::size_t x1_cells;
        double x0_min;
        double x0_max;
        double x1_min;
        double x1_max;
};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        //     return amp::Path2D();
        // }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
        //     return amp::ManipulatorTrajectory2Link();
        // }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

// class MyAStarAlgo : public amp::AStar {
//     public:
//         virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
// };

bool cellCollision(amp::Environment2D environment, Eigen::Vector2d point);
MyGridCSpace computeManipulatorGrid(amp::Environment2D environment, std::vector<double> linkLengths);
bool inCollisionCheck(double x0, double x1, amp::Environment2D environment, Link2d robot);