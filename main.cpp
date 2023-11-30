#include <embree3/rtcore.h>
#include <iostream>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <random>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "embree3/stb_image_write.h"
#include "Triangle.h"


std::mt19937_64 engine;
std::uniform_real_distribution<> dist;


struct BoundingBox {
    Eigen::Vector3d min;
    Eigen::Vector3d max;

    bool contains(const Eigen::Vector3d& point) const {
        return (point.x() >= min.x() && point.x() <= max.x() &&
                point.y() >= min.y() && point.y() <= max.y() &&
                point.z() >= min.z() && point.z() <= max.z());
    }
};


RTCRay makeRay(const Eigen::Vector3d &org, const Eigen::Vector3d &dir) {
    RTCRay ray;
    ray.org_x = org.x();
    ray.org_y = org.y();
    ray.org_z = org.z();

    // 方向ベクトルを正規化
    Eigen::Vector3d normalized_dir = dir.normalized();
    ray.dir_x = normalized_dir.x();
    ray.dir_y = normalized_dir.y();
    ray.dir_z = normalized_dir.z();

    return ray;
}


void errorHandler(void *userPtr, RTCError code, const char *str) {
    if (code == RTC_ERROR_NONE)
        return;

    std::cerr << "Error " << code << ": " << str << std::endl;
    exit(1); // Consider proper error handling here
}

void makingTunnel(std::vector<Triangle> &triangles) {

    int numberOfTriangles = 500;
    float radius = 2.0;//// Radius for positioning triangles
    float ang = EIGEN_PI / (float) numberOfTriangles; //// Angle for each triangle

    Color white = {1.0f, 1.0f, 1.f};
    Color yellow = {1.0f, 0.9f, 0.7f};

    float spacing = 20; ////space of two triangle
    for (int i = 0; i < numberOfTriangles; ++i) {
        // Calculate or determine parameters for the triangle, such as position or color.
        // This can be based on uniform distribution, randomness, or some calculated pattern.
        // Example: Positioning triangles around a circle
        float angle = (float) i / numberOfTriangles * EIGEN_PI; //// Angle for each triangle
        float x = cos(angle) * radius;
        float y = sin(angle) * radius;
        float x_2 = cos(angle + ang) * radius;
        float y_2 = sin(angle + ang) * radius;
        ////     Set the vertices and colors for the triangle (this is simplified but can be made as complex as desired)

        Color red = {1.0f, 0.001f, 0.01f};

        if ((i <= numberOfTriangles * 0.25 && i >= numberOfTriangles * 0.2) ||
            i <= numberOfTriangles * 0.8 && i >= numberOfTriangles * 0.75) {
            ///  top
            Eigen::Vector3d v1 = {x, y, 0};
            Eigen::Vector3d v2 = {x, y, spacing};
            Eigen::Vector3d v3 = {x_2, y_2, spacing};
            /// Add the new triangle to the list of triangles.
            triangles.push_back({v1, v2, v3, yellow, 0.7, 2.5});
            v1 = {x, y, 0};
            v2 = {x_2, y_2, spacing};
            v3 = {x_2, y_2, 0};
            /// Add the new triangle to the list of triangles.
            triangles.push_back({v1, v2, v3, yellow, 0.7, 2.5});
        } else {
            ///  top
            Eigen::Vector3d v1 = {x, y, 0};
            Eigen::Vector3d v2 = {x, y, spacing};
            Eigen::Vector3d v3 = {x_2, y_2, spacing};
            /// Add the new triangle to the list of triangles.
            triangles.push_back({v1, v2, v3, red, 0.6, 0});
            v1 = {x, y, 0};
            v2 = {x_2, y_2, spacing};
            v3 = {x_2, y_2, 0};
            /// Add the new triangle to the list of triangles.
            triangles.push_back({v1, v2, v3, red, 0.6, 0});
        }

    }

    ////ground
    Eigen::Vector3d v1 = {radius, 0.0, 0};
    Eigen::Vector3d v2 = {radius, 0.0, spacing};
    Eigen::Vector3d v3 = {-radius, 0.0, 0};
    triangles.push_back({v2, v1, v3, white, 0.9, 0});
    v1 = {radius, 0.0, spacing};
    v2 = {-radius, 0.0, spacing};
    v3 = {-radius, 0.0, 0};
    triangles.push_back({v2, v1, v3, white, 0.9, 0});


    std::cout << "Tunnel is ready" << std::endl;

}


void computeLocalFrame(const Eigen::Vector3d &w, Eigen::Vector3d &u, Eigen::Vector3d &v) {
    if (fabs(w.x()) > 1e-3)
        u = Eigen::Vector3d::UnitY().cross(w).normalized();
    else
        u = Eigen::Vector3d::UnitX().cross(w).normalized();

    v = w.cross(u).normalized();

}

void diffuseSample(const RTCRayHit &in_rayhit, RTCRayHit &out_rayhit) {
    Eigen::Vector3d incidentPoint{
            in_rayhit.ray.org_x + in_rayhit.ray.tfar * in_rayhit.ray.dir_x,
            in_rayhit.ray.org_y + in_rayhit.ray.tfar * in_rayhit.ray.dir_y,
            in_rayhit.ray.org_z + in_rayhit.ray.tfar * in_rayhit.ray.dir_z
    };

    /// compute normal
    Eigen::Vector3d normal(in_rayhit.hit.Ng_x, in_rayhit.hit.Ng_y, in_rayhit.hit.Ng_z);
    const double phi = 2.0 * EIGEN_PI * dist(engine);
    const double theta = asin(sqrt(dist(engine)));

    normal = normal.normalized();

    Eigen::Vector3d direction(in_rayhit.ray.dir_x, in_rayhit.ray.dir_y, in_rayhit.ray.dir_z);

    //applyOffset(incidentPoint, normal, direction);

    /// normalの方向をy軸とした正規直交基底を作る
    Eigen::Vector3d u, v;
    computeLocalFrame(normal, u, v);

    const double _x = sin(theta) * cos(phi);
    const double _y = cos(theta);
    const double _z = sin(theta) * sin(phi);

    const Eigen::Vector3d diffuseDir = _x * u + _y * normal + _z * v;

    out_rayhit.ray = makeRay(incidentPoint, diffuseDir);
}


Color trace(RTCScene &scene,RTCRayHit &rayhit ,const std::vector<Triangle> &triangles, const BoundingBox& box) {

    Color out_color = Color::Zero();

    const double scattering_coefficient = 0.8;//percentage of scattering\

    Eigen::Vector3d rayPosition(rayhit.ray.org_x + rayhit.ray.tfar * rayhit.ray.dir_x,
                                rayhit.ray.org_y + rayhit.ray.tfar * rayhit.ray.dir_y,
                                rayhit.ray.org_z + rayhit.ray.tfar * rayhit.ray.dir_z);




    while(true){
        if (box.contains(rayPosition)) {
            const auto &hitPrim = triangles[rayhit.hit.primID];

            out_color = hitPrim.getEmission() * hitPrim.getColor();////adding self emission


            const double diffuse_q = hitPrim.getKd();

            const double xi = dist(engine);

            if(xi < diffuse_q) {
                RTCRayHit _rayhit;
                diffuseSample(rayhit, _rayhit);
                _rayhit.ray.tnear = 0.001f;
                _rayhit.ray.tfar = std::numeric_limits<float>::infinity();

                _rayhit.ray.flags = 0;
                _rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                _rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;

                RTCIntersectContext context;
                rtcInitIntersectContext(&context);

                rtcIntersect1(scene, &context, &_rayhit);

                out_color += (hitPrim.getKd() * hitPrim.getColor()).cwiseProduct(trace(scene,_rayhit,triangles,box));

                return out_color;

            }

            break;
        }else{

            double scatterDistance = -log(dist(engine)) / scattering_coefficient;
            double travelDistance = 0.0;
            double participating_media_kd = 0.25;
            double travel_max_distance = 0;

            while (scatterDistance < travel_max_distance) {

                //scattering
                if (dist(engine) < participating_media_kd) {

                    //std::cout << "here" << std::endl;

                    //choose the direction in randomly
                    RTCRay newRay;
                    travelDistance = std::min(travelDistance, travel_max_distance);
                    travelDistance += FLT_EPSILON; // to prevent fake true of next step
                    newRay.org_x = rayhit.ray.org_x + travelDistance * rayhit.ray.dir_x;
                    newRay.org_y = rayhit.ray.org_y + travelDistance * rayhit.ray.dir_y;
                    newRay.org_z = rayhit.ray.org_z + travelDistance * rayhit.ray.dir_z;

                    //new direction
                    newRay.dir_x = 2 * dist(engine) - 1;
                    newRay.dir_y = 2 * dist(engine) - 1;
                    newRay.dir_z = 2 * dist(engine) - 1;

                    newRay.tnear = 0.001f;
                    newRay.tfar = std::numeric_limits<float>::infinity();
                    newRay.flags = 0;

                    RTCRayHit newRayHit;
                    newRayHit.ray = newRay;
                    newRayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                    newRayHit.hit.primID = RTC_INVALID_GEOMETRY_ID;



                    RTCIntersectContext context;
                    rtcInitIntersectContext(&context);

                    rtcIntersect1(scene, &context, &newRayHit);



                    auto next_raytracing = trace(scene,newRayHit,triangles,box);
                    out_color = next_raytracing;

                    return out_color;


                    return trace(scene, newRayHit, triangles,box);
                }else{
                    return Color::Zero();
                }
            }

        }

    }

    return out_color;////if no intersection... return only self emission
}

void VolumeRendering(RTCScene &scene, const std::vector<Triangle> &triangles) {
    const int width = 800;
    const int height = 600;

    const int samples = 100;

    // Camera setup
    float aspectRatio = (float) width / (float) height;
    float fov = (float) (EIGEN_PI / 4.0); // Field of view in radians (45 degrees)

    // Resulting image data (3 channels, for RGB)
    std::vector<unsigned char> imageData(width * height * 3, 0);

    BoundingBox box;
    box.min = Eigen::Vector3d(-0.5, 0.0, 0.0); // 最小座標（例：x, y, zがそれぞれ-1）
    box.max = Eigen::Vector3d(0.5, 1.0, 1.0);   // 最大

#pragma omp parallel for
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Normalized pixel coordinates from -1 to 1
            float px = (2.0f * ((x + 0.5f) / width) - 1.0f) * tan(fov / 2.0f) * aspectRatio;
            float py = (1.0f - 2.0f * ((y + 0.5f) / height)) * tan(fov / 2.0f);


            // Setting up the ray
            RTCRayHit rayhit = {};
            rayhit.ray.org_x = 0.0f; // Camera position
            rayhit.ray.org_y = 1.0f; ////for Room,tunnel
            //rayhit.ray.org_z = -8.5f;   ////for Room
            rayhit.ray.org_z = -4.0f;    ////forTunnel

            Eigen::Vector3d CameraPos = {rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z};

            rayhit.ray.dir_x = px; // Ray direction based on the pixel position
            rayhit.ray.dir_y = py;
            rayhit.ray.dir_z = 1.0f; // Looking forward

            Eigen::Vector3d Ray_dir = {rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z};

            rayhit.ray.tnear = 0.001f;
            rayhit.ray.tfar = std::numeric_limits<float>::infinity();

            rayhit.ray.flags = 0;
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;

            RTCIntersectContext context;
            rtcInitIntersectContext(&context);
            rtcIntersect1(scene, &context, &rayhit);

            auto *pixel = &imageData[(y * width + x) * 3];

            Color color;
            const Color bgColor = {20/ 255.0, 34/ 255.0, 34/ 255.0};

            Color accumulatedColor = Color::Zero();

            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
                Color _accumulatedColor = Color::Zero();
                for (int s = 0; s < samples; s++) {
                    _accumulatedColor += trace(scene,rayhit,triangles,box);
                }
                color = _accumulatedColor / static_cast<double>(samples);
            } else {
                color = bgColor;
            }

            pixel[0] = (std::clamp(static_cast<float>(color.x()) * 255.f, 0.f, 255.f));
            pixel[1] = (std::clamp(static_cast<float>(color.y()) * 255.f, 0.f, 255.f));
            pixel[2] = (std::clamp(static_cast<float>(color.z()) * 255.f, 0.f, 255.f));

        }
    }

    stbi_write_png("output_VolumeRendering.png", width, height, 3, imageData.data(), width * 3);

}

//void NormalPathTracing(RTCScene &scene, const std::vector<Triangle> &triangles) {
//
//    // Dimensions of the image
//    const int width = 800;
//    const int height = 600;
//
//    const int samples = 100;
//
//    // Camera setup
//    float aspectRatio = (float) width / (float) height;
//    float fov = (float) (EIGEN_PI / 4.0); // Field of view in radians (45 degrees)
//
//    // Resulting image data (3 channels, for RGB)
//    std::vector<unsigned char> imageData(width * height * 3, 0);
//
//#pragma omp parallel for
//    for (int y = 0; y < height; ++y) {
//        for (int x = 0; x < width; ++x) {
//            // Normalized pixel coordinates from -1 to 1
//            float px = (2.0f * ((x + 0.5f) / width) - 1.0f) * tan(fov / 2.0f) * aspectRatio;
//            float py = (1.0f - 2.0f * ((y + 0.5f) / height)) * tan(fov / 2.0f);
//
//
//            // Setting up the ray
//            RTCRayHit rayhit = {};
//            rayhit.ray.org_x = 0.0f; // Camera position
//            rayhit.ray.org_y = 1.0f; ////for Room,tunnel
//            //rayhit.ray.org_z = -8.5f;   ////for Room
//            rayhit.ray.org_z = -4.0f;    ////forTunnel
//
//            Eigen::Vector3d CameraPos = {rayhit.ray.org_x, rayhit.ray.org_y, rayhit.ray.org_z};
//
//            rayhit.ray.dir_x = px; // Ray direction based on the pixel position
//            rayhit.ray.dir_y = py;
//            rayhit.ray.dir_z = 1.0f; // Looking forward
//
//            Eigen::Vector3d Ray_dir = {rayhit.ray.dir_x, rayhit.ray.dir_y, rayhit.ray.dir_z};
//
//            rayhit.ray.tnear = 0.001f;
//            rayhit.ray.tfar = std::numeric_limits<float>::infinity();
//
//            rayhit.ray.flags = 0;
//            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
//            rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
//
//            RTCIntersectContext context;
//            rtcInitIntersectContext(&context);
//            rtcIntersect1(scene, &context, &rayhit);
//
//            auto *pixel = &imageData[(y * width + x) * 3];
//
//            Color color;
//            const Color bgColor = {20/ 255.0, 34/ 255.0, 34/ 255.0};
//
//            Color accumulatedColor = Color::Zero();
//
//            if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
//                Color _accumulatedColor = Color::Zero();
//                for (int s = 0; s < samples; s++) {
//                    _accumulatedColor += trace(scene, rayhit, triangles, box);
//                }
//                color = _accumulatedColor / static_cast<double>(samples);
//            } else {
//                color = bgColor;
//            }
//
//            pixel[0] = (std::clamp(static_cast<float>(color.x()) * 255.f, 0.f, 255.f));
//            pixel[1] = (std::clamp(static_cast<float>(color.y()) * 255.f, 0.f, 255.f));
//            pixel[2] = (std::clamp(static_cast<float>(color.z()) * 255.f, 0.f, 255.f));
//
//        }
//    }
//
//    stbi_write_png("output.png", width, height, 3, imageData.data(), width * 3);
//
//}


void cleanUp(RTCDevice &device, RTCScene &scene) {
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
}


void makingRoom(std::vector<Triangle> &triangles) {
    Color white = {1.0f, 1.0f, 1.0f};
    Color purple = {1.0f, 0.001f, 1.f};
    Color red = {1.0f, 0.001f, 0.01f};

    Color blue = {0.001f, 0.001f, 1.f};
    Color yellow = {1.0f, 0.9f, 0.7f};


    const auto t1 = Eigen::Vector3d{0.7f, 0.7f, -4.4f};
    const auto t2 = Eigen::Vector3d{0.0f, 1.3f, -1.0f};
    const auto t3 = Eigen::Vector3d{-0.7f, 0.7f, -4.4f};

    const auto t4 = Eigen::Vector3d{0.7f, 0.7f, -1.0f};
    const auto t5 = Eigen::Vector3d{1.9f, 1.3f, -2.0f};
    const auto t6 = Eigen::Vector3d{1.7f, 0.7f, -4.4f};

    const auto t7 = Eigen::Vector3d{-0.7f, 0.7f, -1.0f};
    const auto t8 = Eigen::Vector3d{-1.9f, 1.3f, -2.0f};
    const auto t9 = Eigen::Vector3d{-1.7f, 0.7f, -4.4f};


    triangles.push_back({t2, t1, t3, red, 0.8, 0});
    triangles.push_back({t4, t5, t6, red, 0.8, 0});
    triangles.push_back({t8, t7, t9, red, 0.8, 0});
//    triangles.push_back({t1, t2, t4, red, 0, 0});
//    triangles.push_back({t2, t3, t4, red, 0, 0});

    const auto v1 = Eigen::Vector3d{-2.0f, 0.0f, 0.0f};
    const auto v2 = Eigen::Vector3d{-2.0f, 2.0f, 0.0f};
    const auto v3 = Eigen::Vector3d{2.0f, 2.0f, 0.0f};
    const auto v4 = Eigen::Vector3d{2.0f, 0.0f, 0.0f};

    const auto v5 = Eigen::Vector3d{-2.0f, 0.0f, -5.0f};
    const auto v6 = Eigen::Vector3d{-2.0f, 2.0f, -5.0f};
    const auto v7 = Eigen::Vector3d{2.0f, 2.0f, -5.0f};
    const auto v8 = Eigen::Vector3d{2.0f, 0.0f, -5.0f};


    /// Add the new triangle to the list of triangles.
    triangles.push_back({v1, v2, v3, white, 0.8, 0});
    triangles.push_back({v1, v3, v4, white, 0.8, 0});

    triangles.push_back({v1, v4, v5, white, 0.8, 0});
    triangles.push_back({v5, v4, v8, white, 0.8, 0});

    triangles.push_back({v3, v7, v8, white, 0.8, 0});
    triangles.push_back({v4, v3, v8, white, 0.8, 0});

    triangles.push_back({v3, v2, v7, white, 0.8, 1});
    triangles.push_back({v2, v6, v7, white, 0.8, 1});

    triangles.push_back({v2, v5, v6, white, 0.8, 0});
    triangles.push_back({v2, v1, v5, white, 0.8, 0});



}

void TrianglesToEmbree(RTCDevice &device, RTCScene &scene, const std::vector<Triangle> &triangles) {

    std::vector<Eigen::Vector3d> vertices;
    std::vector<unsigned int> indices;

    for (const auto &triangle: triangles) {
        size_t index = vertices.size();
        vertices.push_back(triangle.getV1());
        vertices.push_back(triangle.getV2());
        vertices.push_back(triangle.getV3());

        indices.push_back(index);
        indices.push_back(index + 1);
        indices.push_back(index + 2);
    }

    ////Initialize Embree
    device = rtcNewDevice(nullptr);
    if (!device) {
        std::cerr << "Error creating Embree device" << std::endl;
        exit(0);
    }
    rtcSetDeviceErrorFunction(device, errorHandler, nullptr);

    ////Create a scene
    scene = rtcNewScene(device);
    ////Create a triangle mesh
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    ////// Allocate the vertex buffer
    float *vertexBuffer = (float *) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                                                            sizeof(float) * 3, vertices.size());
    if (!vertexBuffer) {
        std::cerr << "Error creating vertex buffer" << std::endl;
        exit(0);
    }




    ////copy the vertex data
    for (size_t i = 0; i < vertices.size(); i++) {
        vertexBuffer[i * 3 + 0] = vertices[i].x();
        vertexBuffer[i * 3 + 1] = vertices[i].y();
        vertexBuffer[i * 3 + 2] = vertices[i].z();

    }

    //// Allocate the index buffer
    unsigned int *indexBuffer = (unsigned int *) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0,
                                                                         RTC_FORMAT_UINT3, sizeof(unsigned int) * 3,
                                                                         indices.size() / 3);
    if (!indexBuffer) {
        std::cerr << "Error creating index buffer" << std::endl;
        exit(0);
    }
    ////copy the index data
    std::copy(indices.begin(), indices.end(), indexBuffer);

    ////Complete the geometry setup
    rtcCommitGeometry(geom);
    ////unsigned int geomID = rtcAttachGeometry(scene, geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    ////Commit the scene
    rtcCommitScene(scene);

    std::cout << "embree is ready" << std::endl;
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();

    RTCDevice device;
    RTCScene scene;

    engine = std::mt19937_64(0);
    dist = std::uniform_real_distribution<>(0, 1);


    std::vector<Triangle> triangles;

    //makingRoom(triangles);
    makingTunnel(triangles);


    TrianglesToEmbree(device, scene, triangles);
    //NormalPathTracing(scene, triangles);
    VolumeRendering(scene,triangles);
    cleanUp(device, scene);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Function took " << elapsed.count() << " seconds to execute.\n";

    return 0;
}
