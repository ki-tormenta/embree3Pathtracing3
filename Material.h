//
// Created by 25tor on 2023/12/07.
//

#ifndef EMBREE3PATHTRACING3_MATERIAL_H
#define EMBREE3PATHTRACING3_MATERIAL_H


#include <Eigen/Dense>
#include "Color.h"


class Material {
private:
    Color color;
    double kd{};
    double emission{};
    double id{};

public:


    Material() = default;


    Material(const Color &color, double kd, double emission, double id) : color(color), kd(kd), emission(emission), id(id) {
    }

    const Color &getColor() const {
        return color;
    }

    double getKd() const {
        return kd;
    }

    double getEmission() const {
        return emission;
    }

    double getId() const {
        return id;
    }


};


#endif //EMBREE3PATHTRACING3_MATERIAL_H
