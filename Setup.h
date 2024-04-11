//
// Created by anna1 on 2023/12/15.
//
#include "Triangle.h"
#include <Eigen/Dense>

#include "tiny_obj_loader2.h"


#ifndef EMBREE3VOLUMERENDERING_SETUP_H
#define EMBREE3VOLUMERENDERING_SETUP_H

class Setup {
public:
    std::vector<Triangle> triangles;

    double TunnelDepth;
    RTCScene scene;
    RTCDevice device;


    Setup() = default;
//    Setup(std::vector<Triangle> &triangles, double &TunnelDepth):triangles(triangles), TunnelDepth(TunnelDepth){}

    //static std::vector<Triangle> makingRoom(std::vector<Triangle> &triangles) ;

    static std::vector<Triangle> setupTunnel(std::vector<Triangle> &triangles, double TunnelDepth,const float camera_depth);
    static void TrianglesToEmbree(RTCDevice &device, RTCScene &scene, std::vector<Triangle> &triangles);
    static void errorHandler(void *userPtr, RTCError code, const char *str);

    static std::vector<Triangle> makingRoom(std::vector<Triangle> &triangles, const double RoomDepth);


    void MakingCube(std::vector<Triangle> &triangles, const Eigen::Vector3d startPos, double length, Material material);

    static std::vector<Triangle> makingIndirectLightingRoom(std::vector<Triangle> &triangles, const double RoomDepth);
};
#endif //EMBREE3VOLUMERENDERING_SETUP_H
