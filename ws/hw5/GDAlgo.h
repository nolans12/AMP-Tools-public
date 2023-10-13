#pragma once
#include "AMPCore.h"
#include "hw/HW5.h"


class GDAlgo : public amp::GDAlgorithm{
    public:
        GDAlgo(double d_star, double zeta, double q_star, double eta, double epsilon, double step, double maxStep, double numRay){ // initiate with hypers
            this->d_star = d_star;
            this->zeta = zeta;
            this->q_star = q_star;
            this->eta = eta;
            this->epsilon = epsilon;
            this->step = step;
            this->maxStep = maxStep;
            this->numRay = numRay;
        }

        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        Eigen::Vector2d attract(Eigen::Vector2d curr, Eigen::Vector2d goal);
        
        Eigen::Vector2d repulse(Eigen::Vector2d curr, const amp::Problem2D& problem);

        Eigen::Vector2d rayDetect(const amp::Problem2D& problem, const Eigen::Vector2d& point);

        Eigen::Vector2d intersect(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);

        //Eigen::Vector2d additionalRepulse(const amp::Problem2D& problem, const Eigen::Vector2d& point);

    private:
        double d_star;
        double zeta;
        double q_star;
        double eta;
        double epsilon;
        double step;
        double maxStep;
        double numRay;
};