//
// Created by 25tor on 2023/11/06.
//

#ifndef HELLO_EMBREE3_COLOR_H
#define HELLO_EMBREE3_COLOR_H

using Color = Eigen::Vector3d;


double getLuminance(const Color &c);

Color changeLuminance(const Color &c, const double &l_out);

Color codeToColor(const std::string &colorCode);

#endif //HELLO_EMBREE3_COLOR_H
