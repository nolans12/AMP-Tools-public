#pragma once

#include "AMPCore.h"
#include "Eigen/Geometry"
#include "hw/HW2.h"
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>

// Hw 2
Eigen::Vector2d getDirectionVector(Eigen::Vector2d start, Eigen::Vector2d end);
int obstacleIndex(std::vector<amp::Obstacle2D> obstacles, Eigen::Vector2d point);
Eigen::Vector2d intersectionPt(Eigen::Vector2d curr, Eigen::Vector2d next, Eigen::Vector2d left, Eigen::Vector2d right);

// Hw 4
amp::Polygon negativeVert(amp::Polygon posObj, Eigen::Vector2d refPoint);
amp::Polygon rotatePolygon(amp::Polygon, Eigen::Vector2d refPoint, double angle);
std::vector<Eigen::Vector2d> sort_verts(amp::Polygon poly);
double cross(Eigen::Vector2d a, Eigen::Vector2d b);
amp::Polygon minkowski(amp::Polygon robot, amp::Polygon obstacle);
Eigen::MatrixXd Tmatrix(double theta, double a);