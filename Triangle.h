//
// Created by anna1 on 2023/10/06.
//

#ifndef DAY2_TRIANGLE_H
#define DAY2_TRIANGLE_H

#include "Color.h"
#include "Material.h"
#include <Eigen/Dense>

class Triangle {

private:
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Material material;
public:


    Triangle() = default;

    Triangle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Material &material)
            : v1(v1), v2(v2), v3(v3), material(material) {}

    const Eigen::Vector3d &getV1() const {
        return v1;
    }

    const Eigen::Vector3d &getV2() const {
        return v2;
    }

    const Eigen::Vector3d &getV3() const {
        return v3;
    }

    const Material &getMaterial() const {
        return material;
    }

};

#endif //DAY2_TRIANGLE_H
