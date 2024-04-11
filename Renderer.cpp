//
// Created by 25tor on 2023/12/15.
//

#include "Renderer.h"
#include "Setup.h"

#include <embree3/rtcore.h>
#include <vector>
#include <limits>
#include <cmath>
#include <Eigen/Core>
#include <random>
#include <chrono>
#include <iomanip>

#include "embree3/stb_image_write.h"
#include "Triangle.h"


Renderer::Renderer(RTCDevice scene, RTCScene device, std::vector<Triangle> triangles, const Eigen::Vector3d P_Params, const int samples,
                   double tunnelDepth) {
}

std::mt19937_64 engine;
std::uniform_real_distribution<> dist;

Color traceWithMulti(const RTCScene &scene, const RTCRayHit &ray, const std::vector<Triangle> &triangles, const Eigen::Vector3d P_Params, const ParticipatingMedia& participatingMedia);


void rayBoxIntersect(const RTCRayHit& in_ray, const float in_bvmin[3], const float in_bvmax[3], float& out_near, float& out_far);

void update_dir(RTCRayHit &IO_ray, const Eigen::Vector3d &dir){
    Eigen::Vector3d normalized_dir = dir.normalized();
    IO_ray.ray.dir_x = normalized_dir.x();
    IO_ray.ray.dir_y = normalized_dir.y();
    IO_ray.ray.dir_z = normalized_dir.z();
}

void SetRayEmbree(RTCRayHit &ray){
    ray.ray.tnear = 0.00001f;
    ray.ray.tfar = std::numeric_limits<float>::max();
    ray.ray.flags = 0;
    ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.hit.primID = RTC_INVALID_GEOMETRY_ID;
}


void computeLocalFrame(const Eigen::Vector3d &w, Eigen::Vector3d &u, Eigen::Vector3d &v) {
    if (fabs(w.x()) > 1e-3)
        u = Eigen::Vector3d::UnitY().cross(w).normalized();
    else
        u = Eigen::Vector3d::UnitX().cross(w).normalized();

    v = w.cross(u).normalized();

}


void diffuseSample(const RTCRayHit &in_rayhit, RTCRayHit &IO_rayhit) {

    /// compute normal
    Eigen::Vector3d normal(in_rayhit.hit.Ng_x, in_rayhit.hit.Ng_y, in_rayhit.hit.Ng_z);
    const double phi = 2.0 * EIGEN_PI * dist(engine);
    const double theta = asin(sqrt(dist(engine)));

    normal = normal.normalized();

    /// normalの方向をy軸とした正規直交基底を作る
    Eigen::Vector3d u, v;
    computeLocalFrame(normal, u, v);

    const double _x = sin(theta) * cos(phi);
    const double _y = cos(theta);
    const double _z = sin(theta) * sin(phi);

    const Eigen::Vector3d diffuseDir = _x * u + _y * normal + _z * v;

    ////update and normalize
    update_dir(IO_rayhit, diffuseDir);

    ////set t_near t_far ID
    SetRayEmbree(IO_rayhit);

    ////little process
    IO_rayhit.ray.org_x +=  IO_rayhit.ray.tnear * IO_rayhit.ray.dir_x;
    IO_rayhit.ray.org_y +=  IO_rayhit.ray.tnear * IO_rayhit.ray.dir_y;
    IO_rayhit.ray.org_z +=  IO_rayhit.ray.tnear * IO_rayhit.ray.dir_z;
}




void rayTracing(RTCRayHit &IO_rayhit,const RTCScene &scene,const std::vector<Triangle> &triangles) {

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1(scene, &context, &IO_rayhit);////update t_far

}

double tracking(const RTCRayHit &rayhit,const Eigen::Vector3d P_Params, const ParticipatingMedia& participatingMedia) {


    float t_near = rayhit.ray.tnear;
    float t_far = rayhit.ray.tfar;
    rayBoxIntersect(rayhit, participatingMedia.bb_min, participatingMedia.bb_max ,t_near,t_far);
    bool in_bound_box = t_near < t_far;
    double distance;

    // not hit bouding box
    if (std::abs( t_far - std::numeric_limits<float>::max() ) < 1e-3 || !in_bound_box) {
        distance = std::numeric_limits<float>::max();
        //distance = 1000000.0;
        return distance;
        return std::numeric_limits<float>::infinity();

    // hit bounding box
    }else {

        double xi = dist(engine);
        distance = -log(1.0 - xi) / P_Params.x() + t_near;
        return distance;
    }
}

bool shadingSurface(const RTCRayHit &in_ray,RTCRayHit &out_ray, double distance,const std::vector<Triangle> &triangles,const Eigen::Vector3d P_Params,Color &out_color) {

    out_ray.ray.org_x = in_ray.ray.org_x + distance * in_ray.ray.dir_x;
    out_ray.ray.org_y = in_ray.ray.org_y + distance * in_ray.ray.dir_y;
    out_ray.ray.org_z = in_ray.ray.org_z + distance * in_ray.ray.dir_z;

    const auto &hitPrim = triangles[in_ray.hit.primID];
    const double diffuse_q = hitPrim.getMaterial().getKd();
    const double xi = dist(engine);

    if(hitPrim.getMaterial().getId() == 2.0){
        /////mirror reflection
        Eigen::Vector3d normal(in_ray.hit.Ng_x, in_ray.hit.Ng_y, in_ray.hit.Ng_z);
        normal = normal.normalized();
        double dot = normal.dot(Eigen::Vector3d(in_ray.ray.dir_x, in_ray.ray.dir_y, in_ray.ray.dir_z));
        double reflect_ray_dir_x = in_ray.ray.dir_x - 2 * dot * normal.x();
        double reflect_ray_dir_y = in_ray.ray.dir_y - 2 * dot * normal.y();
        double reflect_ray_dir_z = in_ray.ray.dir_z - 2 * dot * normal.z();
        Eigen::Vector3d reflected_ray_dir(reflect_ray_dir_x, reflect_ray_dir_y, reflect_ray_dir_z);

        update_dir(out_ray, reflected_ray_dir);

        ////little process
        out_ray.ray.org_x += out_ray.ray.tnear  * out_ray.ray.dir_x;
        out_ray.ray.org_y += out_ray.ray.tnear  * out_ray.ray.dir_y;
        out_ray.ray.org_z += out_ray.ray.tnear  * out_ray.ray.dir_z;

        ////set t_near t_far ID
        SetRayEmbree(out_ray);


        if (xi < diffuse_q) {
            out_color = (hitPrim.getMaterial().getKd() * hitPrim.getMaterial().getColor()) ;
            return true;
        }else{
            out_color = hitPrim.getMaterial().getEmission() * hitPrim.getMaterial().getColor();
            return false;
        }
    }else{
        if (xi < diffuse_q) {
            diffuseSample(in_ray, out_ray);////sampling to decide a direction

            out_color = (hitPrim.getMaterial().getKd() * hitPrim.getMaterial().getColor()) ;
            return true;
        }else{
            out_color = hitPrim.getMaterial().getEmission() * hitPrim.getMaterial().getColor();
            return false;
        }
    }
}

//
//
//bool shadingSurface2(const RTCRayHit &in_ray, RTCRayHit &out_ray, double distance, const std::vector<Triangle> &triangles, const Eigen::Vector3d P_Params, Color &out_color, const LightSource &light) {
//    // 通常のシェーディング処理
//    out_ray.ray.org_x = in_ray.ray.org_x + distance * in_ray.ray.dir_x;
//    out_ray.ray.org_y = in_ray.ray.org_y + distance * in_ray.ray.dir_y;
//    out_ray.ray.org_z = in_ray.ray.org_z + distance * in_ray.ray.dir_z;
//
//    const auto &hitPrim = triangles[in_ray.hit.primID];
//    Eigen::Vector3d normal(in_ray.hit.Ng_x, in_ray.hit.Ng_y, in_ray.hit.Ng_z);
//    normal.normalize();
//
//    // NEEを用いた直接光の寄与の計算
//    Eigen::Vector3d lightSample = SampleLightSource(light); // 光源のサンプリング
//    Eigen::Vector3d toLight = lightSample - Eigen::Vector3d(out_ray.ray.org_x, out_ray.ray.org_y, out_ray.ray.org_z);
//    double distanceToLight = toLight.norm();
//    toLight.normalize();
//
//    // シャドウレイのテスト
//    if (!ShadowRayIntersects(scene, out_ray.ray.org_x, out_ray.ray.org_y, out_ray.ray.org_z, toLight, distanceToLight)) {
//        // 光源からの放射照度の計算
//        double radiance = light.intensity / (4 * EIGEN_PI * distanceToLight * distanceToLight);
//        // Lambertの反射則による光の寄与
//        double cosTheta = std::max(normal.dot(toLight), 0.0);
//        Color directLightContribution = radiance * cosTheta * hitPrim.getMaterial().getColor();
//
//        // 光の寄与を加算
//        out_color += directLightContribution;
//    }
//}
//
//Eigen::Vector3d SampleLightSource(const LightSource &light) {
//    // 面光源の局所座標系でのランダムな点を生成
//    double u = dist(engine); // [0, 1] の範囲で乱数を生成
//    double v = dist(engine); // [0, 1] の範囲で乱数を生成
//
//    // 矩形の面光源の場合、光源の一辺の長さを使用して位置を計算
//    Eigen::Vector3d pointOnLight = light.position + u * light.uVec * light.sizeU + v * light.vVec * light.sizeV;
//
//    return pointOnLight;
//}
//
//
//bool ShadowRayIntersects(const RTCScene &scene, const Eigen::Vector3d &origin, const Eigen::Vector3d &direction, double distanceToLight) {
//    // シャドウレイの作成
//    RTCRay shadowRay;
//    shadowRay.org_x = origin.x();
//    shadowRay.org_y = origin.y();
//    shadowRay.org_z = origin.z();
//    shadowRay.dir_x = direction.x();
//    shadowRay.dir_y = direction.y();
//    shadowRay.dir_z = direction.z();
//    shadowRay.tnear = 0.001;  // 自己交差を避けるために非ゼロの値を設定
//    shadowRay.tfar = distanceToLight - 0.001; // 光源への距離よりわずかに小さい値に設定
//
//    // シャドウレイをシーンに対してキャストし、交差をチェック
//    RTCIntersectContext context;
//    rtcInitIntersectContext(&context);
//    rtcOccluded1(scene, &context, &shadowRay);
//
//    // tfar が負の値に設定されている場合、衝突が発生したことを意味します
//    return shadowRay.tfar < 0;
//}
//


bool shadingMedia(const RTCRayHit &in_ray, RTCRayHit &out_ray, double distance,const ParticipatingMedia participatingMedia,const Eigen::Vector3d P_Params,Color &out_color) {
    ////little process
    out_ray.ray.org_x = in_ray.ray.org_x + distance * in_ray.ray.dir_x;
    out_ray.ray.org_y = in_ray.ray.org_y + distance * in_ray.ray.dir_y;
    out_ray.ray.org_z = in_ray.ray.org_z + distance * in_ray.ray.dir_z;

    double xi = dist(engine);
    double albedo = P_Params.y();
    double g = P_Params.z();////henyeyGreenstein

    if (xi >= albedo){
        out_color = {0,0,0};
        return false;//absorbed
    }else{////change dir

        double cosT;
        float s = 2 * xi - 1;

        if (fabs(g) <= 1.0e-4) {
            cosT = s + 3/2.0* g* (1-s*s) + - 2 *g*g*s*(1-s*s) ;
        }else {
            const double f = (1.0 - g * g) /
                             (1.0 - g +
                              2.0 * g * dist(engine));

            cosT = (1.0 + g * g - f * f) / (2.0 * g);
        }

        const double theta = acos(std::clamp(cosT, -1.0, 1.0));
        const double phi = 2.0 * EIGEN_PI * dist(engine);

        Eigen::Vector3d bn, cn, outScatteringDir;
        Eigen::Vector3d incidentDir{in_ray.ray.dir_x,in_ray.ray.dir_y,in_ray.ray.dir_z};

        computeLocalFrame(incidentDir, bn, cn);

        const double _x = sin(theta) * cos(phi);
        const double _y = cos(theta);
        const double _z = sin(theta) * sin(phi);

        outScatteringDir = _x * bn + _y * incidentDir + _z * cn;

        update_dir(out_ray, outScatteringDir);


        ////set t_near t_far ID
        SetRayEmbree(out_ray);

        out_color = participatingMedia.color;

        return true;
    }
}

Color TraceWithSingle(const RTCScene &scene, const RTCRayHit &ray, const std::vector<Triangle> &triangles, const Eigen::Vector3d P_Params, const ParticipatingMedia& participatingMedia) {
    Color bgColor = {10 / 255.0, 20 / 255.0, 20 / 255.0};
    //bgColor = {1,1,1};

    RTCRayHit Ray_surface = ray;
    RTCRayHit Ray_media = ray;

    ////set t_near t_far ID
    SetRayEmbree(Ray_media);

    rayTracing(Ray_surface, scene, triangles);////update t_far
    double distance_surface_mode = Ray_surface.ray.tfar;
    double distance_media_mode = tracking(Ray_media, P_Params, participatingMedia);

    if(distance_media_mode>10000000000 && distance_surface_mode > 1000000000){
        return bgColor;
    }

    if(distance_media_mode < distance_surface_mode) {
        RTCRayHit out_ray;
        Color out_color;
        bool is_scattered = shadingMedia(Ray_media,out_ray, distance_media_mode,participatingMedia, P_Params,out_color);///scattering
        if (!is_scattered) {
            return out_color;////absorbed
        }else {
            rayTracing(out_ray, scene, triangles);////update t_far

            RTCRayHit out_ray2;
            Color out_color2;

            if (out_ray.hit.geomID != RTC_INVALID_GEOMETRY_ID) {

                bool is_reflected = shadingSurface(out_ray,out_ray2,out_ray.ray.tfar,triangles,P_Params,out_color2);////reflect
                if (!is_reflected) {
                    return out_color.cwiseProduct( out_color2 );////terminated
                }else {
                    Color next_ray_color = TraceWithSingle(scene, out_ray2, triangles, P_Params, participatingMedia);
                    return out_color.cwiseProduct( out_color2.cwiseProduct(next_ray_color) );
                }
            }else{
                return bgColor;
            }
        }
    }else{
        RTCRayHit out_ray;
        Color out_color;
        bool is_reflected = shadingSurface(Ray_surface,out_ray,distance_surface_mode,triangles,P_Params,out_color);////reflect
        if (!is_reflected) {
            return out_color;////terminated
        }else {
            Color next_ray_color = TraceWithSingle(scene, out_ray, triangles, P_Params, participatingMedia);
            return out_color.cwiseProduct( next_ray_color );
        }
    }
}


// 色を蓄積, 新しい方向決める(rayが更新)
Color traceWithMulti(const RTCScene &scene, const RTCRayHit &ray, const std::vector<Triangle> &triangles, const Eigen::Vector3d P_Params, const ParticipatingMedia& participatingMedia) {
    Color bgColor = {10 / 255.0, 20 / 255.0, 20 / 255.0};


    RTCRayHit Ray_surface = ray;
    RTCRayHit Ray_media = ray;

    ////set t_near t_far ID
    SetRayEmbree(Ray_media);


    rayTracing(Ray_surface, scene, triangles);////update t_far
    double distance_surface_mode = Ray_surface.ray.tfar;
    double distance_media_mode = tracking(Ray_media, P_Params, participatingMedia);

    if(distance_media_mode>100000 && distance_surface_mode > 100000){
        return bgColor;
    }

    if(distance_media_mode < distance_surface_mode) {
        RTCRayHit out_ray;
        Color out_color;
        bool is_scattered = shadingMedia(Ray_media,out_ray, distance_media_mode,participatingMedia, P_Params,out_color);///scattering
        if (!is_scattered) {
            return out_color;////absorbed
        }else {
            Color next_ray_color = traceWithMulti(scene, out_ray, triangles, P_Params, participatingMedia);
            return out_color.cwiseProduct( next_ray_color );
        }
    }else{
        RTCRayHit out_ray;
        Color out_color;
        bool is_reflected = shadingSurface(Ray_surface,out_ray,distance_surface_mode,triangles,P_Params,out_color);////reflect
        if (!is_reflected) {
            return out_color;////terminated
        }else {
            Color next_ray_color = traceWithMulti(scene, out_ray, triangles, P_Params, participatingMedia);
            return out_color.cwiseProduct( next_ray_color );
        }
    }
}

Color trace(const RTCScene &scene, const RTCRayHit &ray, const std::vector<Triangle> &triangles) {
    Color bgColor = {10 / 255.0, 20 / 255.0, 20 / 255.0};


    RTCRayHit Ray_surface = ray;

    rayTracing(Ray_surface, scene, triangles);////update t_far
    double distance_surface_mode = Ray_surface.ray.tfar;

    if(distance_surface_mode > 100000){
        return bgColor;
    }else{
        RTCRayHit out_ray;
        Color out_color;
        bool is_reflected = shadingSurface(Ray_surface,out_ray,distance_surface_mode,triangles,Eigen::Vector3d {0.5,0.5,0.5},out_color);////reflect
        if (!is_reflected) {
            return out_color;////terminated
        }else {
            Color next_ray_color = trace(scene, out_ray, triangles);
            return out_color.cwiseProduct( next_ray_color );
        }
    }
}



void Renderer::VolumeRendering(RTCScene &scene, const std::vector<Triangle> &triangles, const int samples,
                               const Eigen::Vector3d P_Params, double Tunnel_depth,const Eigen::Vector3f camera_pos,const Eigen::Vector3f camera_dir, int i) {
    ////default 800 600
    const int width = 4000;
    const int height = 3000;

    // Camera setup
    float aspectRatio = (float) width / (float) height;
    float fov = (float) (EIGEN_PI / 4.0); // Field of view in radians (45 degrees)

    // Resulting image data (3 channels, for RGB)
    std::vector<unsigned char> imageData(width * height * 3, 0);


#pragma omp parallel for
    for (int y = 0; y < height; ++y) {

        for (int x = 0; x < width; ++x) {
            // Normalized pixel coordinates from -1 to 1
            float px = (2.0f * ((x + 0.5f) / width) - 1.0f) * tan(fov / 2.0f) * aspectRatio;
            float py = (1.0f - 2.0f * ((y + 0.5f) / height)) * tan(fov / 2.0f);

            // Setting up the ray
            RTCRayHit rayhit = {};
            /// Camera position
            rayhit.ray.org_x = camera_pos.x();
            rayhit.ray.org_y = camera_pos.y();
            rayhit.ray.org_z = camera_pos.z();


            if(camera_dir.x()==0.0){
                rayhit.ray.dir_x = px;
            }else{
                rayhit.ray.dir_x = camera_dir.x(); /// Ray direction based on the pixel position
            }
            rayhit.ray.dir_y = camera_dir.y() + py;
            rayhit.ray.dir_z = camera_dir.z() + 1.0f; /// Looking forward

            ////set t_near t_far ID
            SetRayEmbree(rayhit);


            auto *pixel = &imageData[(y * width + x) * 3];
            Color color;
            Color accumulatedColor = Color::Zero();


            for (int s = 0; s < samples; s++) {
                if(i==1){////Single
                    accumulatedColor += TraceWithSingle(scene, rayhit, triangles, P_Params, participatingMedia);
                }else if(i==2){////Multi
                    accumulatedColor += traceWithMulti(scene, rayhit, triangles, P_Params, participatingMedia);
                }else{
                    accumulatedColor += trace(scene, rayhit, triangles);
                }
            }
            color = accumulatedColor / static_cast<double>(samples);


            pixel[0] = (unsigned char)(std::clamp(color.x() * 255.0, 0.0, 255.0));
            pixel[1] = (unsigned char)(std::clamp(color.y() * 255.0, 0.0, 255.0));
            pixel[2] = (unsigned char)(std::clamp(color.z() * 255.0, 0.0, 255.0));

        }
    }

    std::string scattering;
    std::string filename;

    std::ostringstream oss_x;
    std::ostringstream oss_y;
    std::ostringstream oss_z;
    std::ostringstream oss_posx;
    std::ostringstream oss_posy;
    std::ostringstream oss_posz;
    std::ostringstream oss_dirx;
    std::ostringstream oss_diry;
    std::ostringstream oss_dirz;
    oss_posx << std::fixed << std::setprecision(1) << camera_pos.x();
    oss_posy << std::fixed << std::setprecision(1) << camera_pos.y();
    oss_posz << std::fixed << std::setprecision(1) << camera_pos.z();
    oss_dirx << std::fixed << std::setprecision(1) << camera_dir.x();
    oss_diry << std::fixed << std::setprecision(1) << camera_dir.y();
    oss_dirz << std::fixed << std::setprecision(1) << camera_dir.z();
    oss_x << std::fixed << std::setprecision(9) << P_Params.x();
    oss_y << std::fixed << std::setprecision(4) << P_Params.y();
    oss_z << std::fixed << std::setprecision(4) << P_Params.z();
    std::string Ext = oss_x.str();
    std::string Alb = oss_y.str();
    std::string Pha = oss_z.str();
    std::string posX = oss_posx.str();
    std::string posY = oss_posy.str();
    std::string posZ = oss_posz.str();
    std::string dirX = oss_dirx.str();
    std::string dirY = oss_diry.str();
    std::string dirZ = oss_dirz.str();


    if(i==2){
        scattering = "MultiScattering_";
        filename = "Sample" + std::to_string(samples) + "_Pos( "  + posX +  "," + posY +  "," + posZ +  ")_Dir(" + dirX  +  "," + dirY +  "," + dirZ +  ")" + "_Ext" + Ext + "_Alb" + Alb + "_Pha" + Pha;
    }else if(i==1){
        scattering = "SingScattering_";
        filename = "Sample" + std::to_string(samples) + "_Pos( "  + posX +  "," + posY +  "," + posZ +  ")_Dir(" + dirX  +  "," + dirY +  "," + dirZ +  ")" + "_Ext" + Ext + "_Alb" + Alb + "_Pha" + Pha;
    }else{
        scattering = "PathTracing_";
        filename = "Sample" + std::to_string(samples) + "_Pos( "  + posX +  "," + posY +  "," + posZ +  ")_Dir(" + dirX  +  "," + dirY +  "," + dirZ +  ")";
    }

    const std::string output_file_name = scattering + filename + ".png";
    stbi_write_png(output_file_name.c_str(), width, height, 3, imageData.data(), width * 3);

}


void rayBoxIntersect(const RTCRayHit& in_ray, const float in_bvmin[3], const float in_bvmax[3], float& out_near, float& out_far)
{
    float inv_dir[3] = {float(1.0) / in_ray.ray.dir_x, float(1.0) / in_ray.ray.dir_y, float(1.0) / in_ray.ray.dir_z};

    float t[2][3];
    t[0][0] = (in_bvmin[0] - in_ray.ray.org_x) * inv_dir[0];
    t[0][1] = (in_bvmin[1] - in_ray.ray.org_y) * inv_dir[1];
    t[0][2] = (in_bvmin[2] - in_ray.ray.org_z) * inv_dir[2];

    t[1][0] = (in_bvmax[0] - in_ray.ray.org_x) * inv_dir[0];
    t[1][1] = (in_bvmax[1] - in_ray.ray.org_y) * inv_dir[1];
    t[1][2] = (in_bvmax[2] - in_ray.ray.org_z) * inv_dir[2];

    int sel0 = (in_ray.ray.dir_x < 0.0f);
    int sel1 = (in_ray.ray.dir_y < 0.0f);
    int sel2 = (in_ray.ray.dir_z < 0.0f);

    //min
    out_near = std::max(std::max(t[sel2][2], in_ray.ray.tnear), std::max(t[sel1][1], t[sel0][0]));
    //max
    out_far = std::min(std::min(t[1-sel2][2], in_ray.ray.tfar), std::min(t[1-sel1][1], t[1-sel0][0]));
}