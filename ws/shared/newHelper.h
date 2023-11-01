#pragma once

#include "AMPCore.h"
#include "Eigen/Geometry"
#include "hw/HW7.h"
#include "hw/HW6.h"

bool lineFullCollision(Eigen::VectorXd p1, Eigen::VectorXd p2, amp::Problem2D problem);
bool pointCollision(amp::Environment2D environment, Eigen::Vector2d point);
bool lineCollision(Eigen::Vector2d p1, Eigen::Vector2d q1, Eigen::Vector2d p2, Eigen::Vector2d q2);
double randomNum(double min, double max);
amp::Path2D pathSmooth(amp::Path2D currPath, amp::Problem2D problem);