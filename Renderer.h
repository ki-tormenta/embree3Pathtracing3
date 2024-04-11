//
// Created by 25tor on 2023/12/15.
//

#include <embree3/rtcore.h>
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <random>
#include <chrono>

#include "embree3/stb_image_write.h"
#include "Triangle.h"


#ifndef EMBREE3VOLUMERENDERING_RENDERER_H
#define EMBREE3VOLUMERENDERING_RENDERER_H

struct ParticipatingMedia{
    float bb_min[3] {-2,-5.3,0.0};
    float bb_max[3] {2,5.3,100.0};
//    float bb_min[3] {-5.3,-2.0,0.0};
//    float bb_max[3] {5.3,5.3,100.0};
    Color color { 1 , 1, 1};
};


class Renderer {

private:
    RTCScene scene;
    RTCDevice device;
    std::vector<Triangle> triangles;
    ParticipatingMedia participatingMedia;
    Eigen::Vector3d P_Params;
    double tunnelDepth;

public:
    Renderer() = default;

    Renderer(RTCDevice scene, RTCScene device, std::vector<Triangle> triangles, const Eigen::Vector3d P_Params,const int samples,
             double tunnelDepth);

    void VolumeRendering(RTCScene &scene, const std::vector<Triangle> &triangles,const int samples,const Eigen::Vector3d P_Params,double tunnelDepth,const Eigen::Vector3f camera_pos,const Eigen::Vector3f camera_dir, int i);


    void DirectIlluminationRender(RTCScene &scene, const std::vector<Triangle> &triangles, const int samples,
                                  double Tunnel_depth, const Eigen::Vector3f camera_pos,
                                  const Eigen::Vector3f camera_dir);
};

#endif //EMBREE3VOLUMERENDERING_RENDERER_H
