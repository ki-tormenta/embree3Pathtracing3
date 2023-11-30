//
// Created by anna1 on 2023/10/06.
//

#ifndef DAY2_TRIANGLE_H
#define DAY2_TRIANGLE_H


#include "Color.h"
#include <Eigen/Dense>

class Triangle {

private:
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Color color;
    double kd;
    float emission;
    int ID;
public:


    Triangle() = default;

    Triangle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, const Color &color,
             double kd, float emission) : v1(v1), v2(v2), v3(v3), color(color), kd(kd), emission(emission) {}

    const Eigen::Vector3d &getV1() const {
        return v1;
    }

    void setV1(const Eigen::Vector3d &v1) {
        Triangle::v1 = v1;
    }

    const Eigen::Vector3d &getV2() const {
        return v2;
    }

    void setV2(const Eigen::Vector3d &v2) {
        Triangle::v2 = v2;
    }

    const Eigen::Vector3d &getV3() const {
        return v3;
    }

    void setV3(const Eigen::Vector3d &v3) {
        Triangle::v3 = v3;
    }

    const Color &getColor() const {
        return color;
    }

    void setColor(const Color &color) {
        Triangle::color = color;
    }

    float getEmission() const {
        return emission;
    }

    double getKd() const {
        return kd;
    }

    void setKd(double kd) {
        Triangle::kd = kd;
    }

    void setEmission(float emission) {
        Triangle::emission = emission;
    }

};

#endif //DAY2_TRIANGLE_H
