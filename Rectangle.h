//
// Created by 25tor on 2023/11/15.
//

#ifndef DAY_4_RECTANGLE_H
#define DAY_4_RECTANGLE_H

#include <Eigen/Dense>
#include "Ray.h"

class Rectangle {
public:
    Eigen::Vector3d center, right, up, outwardNormal;
    double width, height;
    Eigen::Vector3d v0, v1, v2;


    Rectangle() = default;
    
    Rectangle(const Eigen::Vector3d &center, const Eigen::Vector3d &right, const Eigen::Vector3d &outwardNormal,
              double width, double height);

};


#endif //
