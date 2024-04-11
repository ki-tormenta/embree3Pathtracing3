#include <embree3/rtcore.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "Triangle.h"
#include "Setup.h"
#include "Renderer.h"


void cleanUp(RTCDevice &device, RTCScene &scene) {
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}


int main() {

    int samples = 3;

    std::ifstream patchFile("params.txt");
    std::vector<Eigen::Vector3d> paramSets;
    const Eigen::Vector3f camera_pos = {0.0,0.8,0.0001};//// default y=1.5,z=-12    y=-0.2 z==8
    //const Eigen::Vector3f camera_pos = {0.0,0.5,8.00};//// default y=0.5,z=8   for tunnel
    const Eigen::Vector3f camera_dir = {0.0,0.0,0.0};

    const double Tunnel_depth = 100;
    const double RoomDepth = 5.0;

    int scattering = 0;///// 2>>Malti  1>>Single  0>>PathTracing

    if (patchFile.is_open()) {
        float extinction, albedo, phaseFunction;
        while (patchFile >> extinction >> albedo >> phaseFunction) {
            paramSets.emplace_back(extinction, albedo, phaseFunction);
        }
        patchFile.close();
    } else {
        std::cerr << "Unable to open patch file." << std::endl;
        return -1;
    }

    for (int i = 0; i < paramSets.size(); ++i) {

        auto& P_Params = paramSets[i];
        auto start = std::chrono::high_resolution_clock::now();

        RTCDevice device;
        RTCScene scene;
        std::vector<Triangle> triangles;

        //triangles = Setup::setupTunnel(triangles,Tunnel_depth ,camera_pos.z());////   y=-0.2 z==8
        //triangles = Setup::makingRoom(triangles,RoomDepth);
        triangles = Setup::makingIndirectLightingRoom(triangles,RoomDepth);

        Setup::TrianglesToEmbree(device, scene, triangles);

        Renderer renderer(device,scene, triangles, P_Params , samples, Tunnel_depth);
        //renderer.DirectIlluminationRender(scene,triangles,samples ,Tunnel_depth,camera_pos,camera_dir);
        renderer.VolumeRendering(scene,triangles ,samples ,P_Params,Tunnel_depth ,camera_pos,camera_dir,scattering);

        cleanUp(device, scene);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Function took " << elapsed.count() << " seconds to execute with params ("
                  << P_Params[0] << ", " << P_Params[1] << ", " << P_Params[2] << ").\n";
    }
    return 0;
}