//
// Created by kango on 2023/03/14.
//

#ifndef DAY_1_IMAGE_H
#define DAY_1_IMAGE_H

#include "Color.h"
#include <Eigen/Dense>


class Image {
public:
    int width;
    int height;

//    Color *pixels;

    Image() = default;

    Image(const int &width, const int &height);


};


#endif //DAY_1_IMAGE_H
