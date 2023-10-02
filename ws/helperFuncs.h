#pragma once

#include "AMPCore.h"
#include "Eigen/Geometry"
#include "hw/HW2.h"
#include "bug1.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>

Eigen::Vector2d getDirectionVector(Eigen::Vector2d start, Eigen::Vector2d end);
int obstacleIndex(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point);
Eigen::Vector2d intersectionPt(Eigen::Vector2d curr, Eigen::Vector2d next, Eigen::Vector2d left, Eigen::Vector2d right);