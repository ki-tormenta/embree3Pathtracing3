//
// Created by 25tor on 2023/11/06.
//

#ifndef DAY_2_RAY_H
#define DAY_2_RAY_H

#include <utility>
#include <iostream>

#include "Eigen/Dense"

struct Ray {
    Eigen::Vector3d org;
    Eigen::Vector3d dir;

    Ray() = default;

    Ray(const Eigen::Vector3d &org, const Eigen::Vector3d &dir) : org(org), dir(dir.normalized()) {}

    Eigen::Vector3d at(const double &t) const {
        return org + t * dir;
    }
};

#endif //DAY_2_RAY_H
