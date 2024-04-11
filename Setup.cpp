//
// Created by anna1 on 2023/12/15.
//
#include <Eigen/Core>
#include <embree3/rtcore.h>
#include <iostream>
#include "Setup.h"



std::vector<Triangle> Setup::makingRoom(std::vector<Triangle> &triangles,const double RoomDepth) {

    const Color white = {1.0f, 1.0f, 1.0f};
    const Color purple = {1.0f, 0.001f, 1.f};
    const Color red = {1.0f, 0.001f, 0.01f};
    const Color WallRed = {0.9f, 0.3f, 0.3f};

    const Color blue = {0.3f, 0.3f, 1.f};
    const Color yellow = {1.0f, 0.9f, 0.7f};
    const Color green = {0.001f, 1.0f,0.001f};
    const Color WallGreen = {0.31f, 0.9f,0.31f};

    const Color VividRed = {204.0/255.0, 0.001f, 0.01f};
    const Color VividPink = {204.0/255.0, 0.001f, 102.0/255.0};
    const Color VividSkyBlue = {0.0001f/255.0, 255.0f/255.0, 255.0f/255.0};
    const Color VividSkyBlue2 = {0.0001f/255.0, 204.0f/255.0, 204.0f/255.0};
    const Color CalmSkyBlue = {0.0001f/255.0, 153.0f/255.0, 153.0f/255.0};
    const Color SkyBlue = {204.0f/255.0, 255.0f/255.0, 255.0f/255.0};
    const Color orange = {204.0/255.0, 102.0/255.0, 0.01f};
    const Color CalmYellow = {204.0/255.0, 204.0/255.0, 0.01f};


    const double light_kd = 0.0;
    const double diffuse_kd = 0.8;
    const double diffuse_id = 1.0;
    const double mirror_id = 2.0;

    const double Room_x = 2.0;
    const double Room_y = 2.0;

    const Color Light_color = yellow;
    const Color Wall_back_color = white;
    const Color Wall_front_color = white;
    const Color Wall_side_color1 = SkyBlue;
    const Color Wall_side_color2 = SkyBlue;
    const Color Wall_floor_color = white;


    const auto v1 = Eigen::Vector3d{-Room_x, 0.0f, RoomDepth};
    const auto v2 = Eigen::Vector3d{-Room_x, Room_y, RoomDepth};
    const auto v3 = Eigen::Vector3d{Room_x, Room_y, RoomDepth};
    const auto v4 = Eigen::Vector3d{Room_x, 0.0f, RoomDepth};

    const auto v5 = Eigen::Vector3d{-Room_x, 0.0f, 0.0f};
    const auto v6 = Eigen::Vector3d{-Room_x, Room_y, 0.0f};
    const auto v7 = Eigen::Vector3d{Room_x, Room_y, 0.0f};
    const auto v8 = Eigen::Vector3d{Room_x, 0.0f, 0.0f};

    const auto m1 = Eigen::Vector3d{Room_x, 0.0f, RoomDepth/2.0};
    const auto m2 = Eigen::Vector3d{0.0f, 0.0f, RoomDepth};
    const auto m3 = Eigen::Vector3d{-Room_x, 0.0f, RoomDepth/2.0};
    

    ////celling
    triangles.push_back({v3, v2, v7, Material(Wall_floor_color, diffuse_kd, 0.0,diffuse_id)});
    triangles.push_back({v2, v6, v7, Material(Wall_floor_color, diffuse_kd, 0.0, diffuse_id)});

    
    ///Room Wall
    ////back
    triangles.push_back({v1, v2,v3,Material(Wall_back_color, diffuse_kd, 0.0, mirror_id)});
    triangles.push_back({v1, v3, v4, Material(Wall_back_color, diffuse_kd, 0.0, mirror_id)});

    ////floor
    triangles.push_back({v1, v4, v5, Material(Wall_floor_color, diffuse_kd, 0.0, mirror_id)});
    triangles.push_back({v5, v4, v8, Material(Wall_floor_color, diffuse_kd, 0.0, mirror_id)});

    const double S_distance = RoomDepth/15.0;
    int count = 1;
    ////side
    for(double j=0.0; j < RoomDepth ;j+= S_distance){
        Eigen::Vector3d Phi1 = {Room_x, Room_y, j+S_distance};
        Eigen::Vector3d Phi2 = {Room_x, 0.0f, j+S_distance};
        Eigen::Vector3d Phi3 = {Room_x, 0.0f, j};
        Eigen::Vector3d Phi4 = {Room_x, Room_y, j};

        if(count % 2 == 1){
            triangles.push_back({Phi2, Phi1, Phi3, Material(white,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi3, Phi1, Phi4, Material(white,diffuse_kd,0.0,diffuse_id)});
        }else{
            triangles.push_back({Phi2, Phi1, Phi3, Material(Light_color,light_kd,2.5,diffuse_id)});
            triangles.push_back({Phi3, Phi4, Phi1, Material(Light_color,light_kd,2.5,diffuse_id)});
        }

        Phi1.x() = Phi2.x() = Phi3.x() = Phi4.x() = - Room_x;

        if(count % 2 == 1){
            triangles.push_back({Phi1, Phi2, Phi3, Material(white,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi1, Phi3, Phi4, Material(white,diffuse_kd,0.0,diffuse_id)});
        }else{
            triangles.push_back({Phi1, Phi2, Phi3, Material(Light_color,light_kd,2.5,diffuse_id)});
            triangles.push_back({Phi1, Phi3, Phi4, Material(Light_color,light_kd,2.5,diffuse_id)});
        }

        count++;
    }
    
    ////entrance
    triangles.push_back({v6, v5, v7, Material(Wall_front_color, diffuse_kd, 0.0, mirror_id)});
    triangles.push_back({v5, v8, v7, Material(Wall_front_color, diffuse_kd, 0.0, mirror_id)});


    float obj_length = 0.4;
    float obj_depth1 = RoomDepth*0.63;
    float obj_depth2 = obj_depth1 + obj_length;
    float obj_height = 0.4;
    float obj_width = 0.4;
    float obj_x = 0.25;
    Color objColor = green;

    Eigen::Vector3d c1 = {obj_x, obj_height, obj_depth2};
    Eigen::Vector3d c2 = {obj_x, obj_height, obj_depth1};
    Eigen::Vector3d c3 = {obj_x + obj_width, obj_height, obj_depth1};
    Eigen::Vector3d c4 = {obj_x + obj_width, obj_height, obj_depth2};
    Eigen::Vector3d c5 = {obj_x, 0, obj_depth2};
    Eigen::Vector3d c6=  {obj_x, 0, obj_depth1};
    Eigen::Vector3d c7 = {obj_x + obj_width, 0, obj_depth1};
    Eigen::Vector3d c8 = {obj_x + obj_width, 0, obj_depth2};

    ////object1

    triangles.push_back({c2, c1, c3, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 right side
    triangles.push_back({c1, c2, c5, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 left side
    triangles.push_back({c3, c4, c7, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 entrance
    triangles.push_back({c2, c3, c6, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(objColor,diffuse_kd,0,diffuse_id)});


    ////object2
    c1.x() = c2.x() = c5.x() = c6.x() = -obj_x - obj_width;
    c3.x() = c4.x() = c7.x() = c8.x() = -obj_x;

    objColor = VividPink;

    ////object2 top
    triangles.push_back({c2, c1, c3, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 right side
    triangles.push_back({c1, c2, c5, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 left side
    triangles.push_back({c3, c4, c7, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 front side
    triangles.push_back({c2, c3, c6, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(objColor,diffuse_kd,0.0,diffuse_id)});


    return triangles;
}


std::vector<Triangle> Setup::makingIndirectLightingRoom(std::vector<Triangle> &triangles, const double RoomDepth) {
    const Color white = {1.0f, 1.0f, 1.0f};
    const Color yellow = {1.0f, 0.9f, 0.7f};
    const Color green = {0.1f, 1.0f,0.1f};
    const Color pink = {204.0 / 255.0, 0.001f, 102.0 / 255.0};
    
    const double light_kd = 0.0;
    const double diffuse_kd = 0.8;
    const double diffuse_id = 1.0;
    const double mirror_id = 2.0;

    const double Room_x = 2.0;
    const double Room_y = 2.0;

    const Color Light_color = yellow;
    const Color Wall_back_color = white;
    const Color Wall_front_color = white;
    const Color Wall_floor_color = white;


    const auto v1 = Eigen::Vector3d{-Room_x, Room_y, RoomDepth};
    const auto v2 = Eigen::Vector3d{-Room_x, Room_y, 0};
    const auto v3 = Eigen::Vector3d{Room_x, Room_y, 0};
    const auto v4 = Eigen::Vector3d{Room_x, Room_y, RoomDepth};

    const auto v5 = Eigen::Vector3d{-Room_x, 0.0f, RoomDepth};
    const auto v6 = Eigen::Vector3d{-Room_x, 0.0f, 0.0f};
    const auto v7 = Eigen::Vector3d{Room_x, 0.0f, 0.0f};
    const auto v8 = Eigen::Vector3d{Room_x, 0.0f, RoomDepth};

    const auto m1 = Eigen::Vector3d{Room_x, 0.0f, RoomDepth/2.0};
    const auto m2 = Eigen::Vector3d{0.0f, 0.0f, RoomDepth};
    const auto m3 = Eigen::Vector3d{-Room_x, 0.0f, RoomDepth/2.0};


    ///IndirectPart--------------------------------------------------------------------------
    const auto down_y = 0.2;
    const auto ratio = 0.9;

    auto l1 = v1;auto l2 = v2;auto l3 = v3;auto l4 = v4;
    l1.y() -= down_y; l2.y() -= down_y; l3.y() -= down_y; l4.y() -= down_y;

    ///left
    triangles.push_back({v2, v1, l1, Material(white, diffuse_kd, 0.0,diffuse_id)});
    triangles.push_back({l1, l2, v2, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///back
    triangles.push_back({v1, v4, l1, Material(white, diffuse_kd, 0.0,diffuse_id)});
    triangles.push_back({l1, l4, v4, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ////right
    triangles.push_back({v3, l4, v4, Material(white, diffuse_kd, 0.0,diffuse_id)});
    triangles.push_back({l3, l4, v3, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ////front
    triangles.push_back({v3, v2, l2, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({l2, l3, v3, Material(white, diffuse_kd, 0.0, diffuse_id)});


    auto Inl1 = l1;auto Inl2 = l2;auto Inl3 = l3;auto Inl4 = l4;
    Inl1.x() *= ratio;Inl2.x() *= ratio;Inl3.x() *= ratio;Inl4.x() *= ratio;
    Inl1.z() *= ratio;Inl2.z() += down_y ;Inl3.z() += down_y;Inl4.z() *= ratio;

    const auto IndirectEmission = 2.5;

    ///left
    triangles.push_back({l1, l2, Inl1, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});
    triangles.push_back({Inl1, Inl2, l2, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});

    ///back
    triangles.push_back({Inl1, l1, l4, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});
    triangles.push_back({Inl1, Inl4, l4, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});

    ///right
    triangles.push_back({Inl4, l3, l4, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});
    triangles.push_back({Inl3, Inl4, l3, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});

    ///front
    triangles.push_back({Inl2, l2, Inl3, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});
    triangles.push_back({Inl3, l2, l3, Material(Light_color,light_kd,IndirectEmission,diffuse_id)});


    auto Inl5 = Inl1;auto Inl6 = Inl2;auto Inl7 = Inl3;auto Inl8 = Inl4;
    Inl5.y() -= down_y;Inl6.y() -= down_y;Inl7.y() -= down_y;Inl8.y() -= down_y;

    ///left
    triangles.push_back({Inl2, Inl1, Inl6, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({Inl1, Inl5, Inl6, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///back
    triangles.push_back({Inl5, Inl1, Inl8, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({Inl1, Inl4, Inl8, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///right
    triangles.push_back({Inl4, Inl3, Inl7, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({Inl4, Inl7, Inl8, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///front
    triangles.push_back({Inl2, Inl6, Inl7, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({Inl7, Inl3, Inl2, Material(white, diffuse_kd, 0.0, diffuse_id)});

    auto l5 = l1;auto l6 = l2;auto l7 = l3;auto l8 = l4;
    l5.y() -= down_y; l6.y() -= down_y; l7.y() -= down_y; l8.y() -= down_y;


    ///left
    triangles.push_back({Inl5, l5, Inl6, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({l5, l6, Inl6, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///back
    triangles.push_back({l5, Inl5, l8, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({l8, Inl5, Inl8, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///right
    triangles.push_back({l7, l8, Inl7, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({Inl7, l8, Inl8, Material(white, diffuse_kd, 0.0, diffuse_id)});

    ///front
    triangles.push_back({Inl6, l6, Inl7, Material(white, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({l6, l7, Inl7, Material(white, diffuse_kd, 0.0, diffuse_id)});



    ///Room Wall------------------------------------------------------------------------------------------------------------------------------------------------------------
    ////celling
    triangles.push_back({v1, v2, v3, Material(Wall_floor_color, diffuse_kd, 0.0,diffuse_id)});
    triangles.push_back({v1, v3, v4, Material(Wall_floor_color, diffuse_kd, 0.0, diffuse_id)});
    ////back
    triangles.push_back({v5, v1,v8,Material(Wall_back_color, diffuse_kd, 0.0, diffuse_id)});
    triangles.push_back({v8, v1, v4, Material(Wall_back_color, diffuse_kd, 0.0, diffuse_id)});
    ////floor
    triangles.push_back({v6, v5, v7, Material(Wall_floor_color, diffuse_kd, 0.0, mirror_id)});
    triangles.push_back({v7, v5, v8, Material(Wall_floor_color, diffuse_kd, 0.0, mirror_id)});

    const double S_distance = RoomDepth/21.0;
    const double LightEmission = 1.2;
    int count = 1;
    ////side
    for(double j=0.0; j < RoomDepth ;j+= S_distance){
        Eigen::Vector3d Phi1 = {Room_x, Room_y-2*down_y, j+S_distance};
        Eigen::Vector3d Phi2 = {Room_x, 0.0f, j+S_distance};
        Eigen::Vector3d Phi3 = {Room_x, 0.0f, j};
        Eigen::Vector3d Phi4 = {Room_x, Room_y-2*down_y, j};

        if(count % 2 == 1){
            triangles.push_back({Phi2, Phi1, Phi3, Material(white,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi3, Phi1, Phi4, Material(white,diffuse_kd,0.0,diffuse_id)});
        }else{
            triangles.push_back({Phi2, Phi1, Phi3, Material(Light_color,light_kd,LightEmission,diffuse_id)});
            triangles.push_back({Phi3, Phi4, Phi1, Material(Light_color,light_kd,LightEmission,diffuse_id)});
        }

        Phi1.x() = Phi2.x() = Phi3.x() = Phi4.x() = - Room_x;

        if(count % 2 == 1){
            triangles.push_back({Phi1, Phi2, Phi3, Material(white,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi1, Phi3, Phi4, Material(white,diffuse_kd,0.0,diffuse_id)});
        }else{
            triangles.push_back({Phi1, Phi2, Phi3, Material(Light_color,light_kd,LightEmission,diffuse_id)});
            triangles.push_back({Phi1, Phi3, Phi4, Material(Light_color,light_kd,LightEmission,diffuse_id)});
        }

        count++;
    }

    ////entrance
    triangles.push_back({v2, v6, v7, Material(Wall_front_color, light_kd, 1.2, diffuse_id)});
    triangles.push_back({v7, v3, v2, Material(Wall_front_color, light_kd, 1.2, diffuse_id)});


    float obj_length = 0.6;
    float obj_depth1 = RoomDepth*0.72;
    float obj_depth2 = obj_depth1 + obj_length;
    float obj_height = obj_length;
    float obj_width = obj_length;
    float obj_x = 0.2;
    Color objColor = green;

    Eigen::Vector3d c1 = {obj_x, obj_height, obj_depth2};
    Eigen::Vector3d c2 = {obj_x, obj_height, obj_depth1};
    Eigen::Vector3d c3 = {obj_x + obj_width, obj_height, obj_depth1};
    Eigen::Vector3d c4 = {obj_x + obj_width, obj_height, obj_depth2};
    Eigen::Vector3d c5 = {obj_x, 0, obj_depth2};
    Eigen::Vector3d c6=  {obj_x, 0, obj_depth1};
    Eigen::Vector3d c7 = {obj_x + obj_width, 0, obj_depth1};
    Eigen::Vector3d c8 = {obj_x + obj_width, 0, obj_depth2};

    ////object1

    triangles.push_back({c2, c1, c3, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 right side
    triangles.push_back({c1, c2, c5, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 left side
    triangles.push_back({c3, c4, c7, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(objColor,diffuse_kd,0,diffuse_id)});
    ////object1 entrance
    triangles.push_back({c2, c3, c6, Material(objColor,diffuse_kd,0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(objColor,diffuse_kd,0,diffuse_id)});


    ////object2
    c1.x() = c2.x() = c5.x() = c6.x() = -obj_x - obj_width;
    c3.x() = c4.x() = c7.x() = c8.x() = -obj_x;

    objColor = pink;

    ////object2 top
    triangles.push_back({c2, c1, c3, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 right side
    triangles.push_back({c1, c2, c5, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 left side
    triangles.push_back({c3, c4, c7, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    ////object2 front side
    triangles.push_back({c2, c3, c6, Material(objColor,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(objColor,diffuse_kd,0.0,diffuse_id)});

/////3D triangle
//    obj_x = 0.7;
//    obj_height = 0.2;
//    obj_length = 0.65;
//    obj_depth1 = RoomDepth*0.55;
//    obj_depth2 = obj_depth1 + obj_length;
//
//    Eigen::Vector3d t1 = {-obj_x, obj_height, obj_depth2+0.1};
//    Eigen::Vector3d t2 = {-obj_x-0.5, obj_height, obj_depth1+0.1};
//    Eigen::Vector3d t3 = {obj_x-0.5 , obj_height, obj_depth1-0.1};
//    Eigen::Vector3d t4 = {obj_x , obj_height, obj_depth2-0.1};
//    Eigen::Vector3d t5 = {0, obj_height+0.5, (obj_depth2+obj_depth1)/2.0};
//
//    triangles.push_back({t1, t2, t3, Material(objColor,diffuse_kd,0.0,diffuse_id)});
//    triangles.push_back({t1, t3, t4, Material(objColor,diffuse_kd,0.0,diffuse_id)});
//
//    triangles.push_back({t1, t2, t5, Material(objColor,diffuse_kd,0.0,diffuse_id)});
//    triangles.push_back({t3, t2, t5, Material(objColor,diffuse_kd,0.0,diffuse_id)});
//    triangles.push_back({t4, t3, t5, Material(objColor,diffuse_kd,0.0,diffuse_id)});
//    triangles.push_back({t4, t1, t5, Material(objColor,diffuse_kd,0.0,diffuse_id)});



    return triangles;
}

std::vector<Triangle> Setup::setupTunnel(std::vector<Triangle> &triangles,double TunnelDepth ,const float camera_depth) {
    Color white = {1.0f, 1.0f, 1.f};
    Color black = {0.4f, 0.4f, 0.4f};
    Color yellow = {240.0f/255.0f, 180.0f/255.0f, 89.0f/255.0f};
    Color red = {1.0f,0.001f,0.001f};
    Color Wallred = {231.0/255.0f, 57.0/255.0f, 43.0/255.0f};
    Color blue = {0.001f, 0.001f,1.0f};
    Color green = {0.001f, 1.0f,0.001f};

    Color gray = {80.0/255.0f, 80.0/255.0f, 80.0/255.0f};
    Color LightRed = {190.0/255.0f, 77.0/255.0f, 60.0/255.0f};
    Color LightGreen = {34.0/255.0f, 255.0/255.0f, 17.0/255.0f};
    Color Wall_Gray = {240.0/255.0f, 240.0/255.0f, 240.0/255.0f};
    Color orange = {0.63333,0.37,0.2001};

    const Color VividRed = {204.0/255.0, 0.001f, 0.01f};
    const Color VividPink = {204.0/255.0, 0.001f, 102.0/255.0};
    const Color VividSkyBlue = {0.0001f/255.0, 255.0f/255.0, 255.0f/255.0};
    const Color VividSkyBlue2 = {0.0001f/255.0, 204.0f/255.0, 204.0f/255.0};
    const Color CalmSkyBlue = {0.0001f/255.0, 153.0f/255.0, 153.0f/255.0};
    const Color SkyBlue = {204.0f/255.0, 255.0f/255.0, 255.0f/255.0};
    const Color CalmYellow = {204.0/255.0, 204.0/255.0, 0.01f};



    Color Wall_Color = Wall_Gray;
    Color LightColor = yellow;
    Color EscapeLight = LightGreen;
    Color Ground_Color = gray;

    /// Add the new triangle to the list of triangles.


    const double light_kd = 0.0;
    const double diffuse_kd = 0.8;

    const double diffuse_id = 1.0;
    const double mirror_id = 2.0;

    int numberOfTriangles = 360;
    int extend_angle = 18;
    float radius = 5.3;//// Radius for positioning triangles
    float ang = EIGEN_PI / (float) numberOfTriangles; //// Angle for each triangle
    ////light angle is 48.66 degree
    float l_ang = 0.25; //0.0~0.5

    float l_width = 0.06; //defalut:0.05


    for (int i = -extend_angle; i < numberOfTriangles+extend_angle; ++i) {
        //// Calculate or determine parameters for the triangle, such as position or color.
        //// This can be based on uniform distribution, randomness, or some calculated pattern.
        float angle = (float) i *ang; //// Angle for each triangle
        float x = cos(angle) * radius;
        float y = sin(angle) * radius;
        float x_2 = cos(angle + ang) * radius;
        float y_2 = sin(angle + ang) * radius;
        double interval = 1.5;

        for(double j=0; j<TunnelDepth;j+=interval) {

            if (((i <= numberOfTriangles * (l_ang + l_width) && i >= numberOfTriangles * l_ang) ||
                 (i <= numberOfTriangles * (1 - l_ang) && i >= numberOfTriangles * (1 - l_width - l_ang))) &&
                ((int) j % 2 == 1)) {

                ///  side light
                Eigen::Vector3d v1 = {x, y, j};
                Eigen::Vector3d v2 = {x, y, j + interval};
                Eigen::Vector3d v3 = {x_2, y_2, j + interval};
                Eigen::Vector3d v4 = {x_2, y_2, j};

                //// emission default 7.5
                triangles.push_back({v1, v2, v3, Material(LightColor, light_kd, 6.5, diffuse_id)});
                triangles.push_back({v1, v3, v4, Material(LightColor, light_kd, 6.5, diffuse_id)});

            }
            else {
                ///  top
                Eigen::Vector3d v1 = {x, y, j};
                Eigen::Vector3d v2 = {x, y, j + interval};
                Eigen::Vector3d v3 = {x_2, y_2, j + interval};
                Eigen::Vector3d v4 = {x_2, y_2, j};

                triangles.push_back({v1, v2, v3, Material(Wall_Color, diffuse_kd, 0.0, diffuse_id)});
                triangles.push_back({v1, v3, v4, Material(Wall_Color, diffuse_kd, 0.0, diffuse_id)});

            }

        }
    }

    ////ground 4と８
    float ground_x = cos(-extend_angle *ang) *radius; //// Angle for each triangle
    float ground_y = sin(-extend_angle*ang) * radius;

    float ground_x2 = cos((360+extend_angle)*ang) * radius;
    float ground_y2 = sin((360+extend_angle)*ang) * radius;

    float WalkRatio = 0.6949;////(3500/5300cos(phi))

    float ground_x2_for_walk = ground_x2 * WalkRatio;
    float ground_x_for_walk = ground_x * WalkRatio;
    float ground_y_for_walk = ground_y -0.3;

    float WalkWidth = ground_x_for_walk - ground_x2_for_walk;

    Eigen::Vector3d g1 = {ground_x, ground_y, 0};
    Eigen::Vector3d g2 = {ground_x, ground_y, TunnelDepth};
    Eigen::Vector3d g3 = {ground_x2, ground_y2, 0};
    Eigen::Vector3d g4 = {ground_x2, ground_y2, TunnelDepth};

    Eigen::Vector3d gW1 = {ground_x_for_walk, ground_y2, 0};
    Eigen::Vector3d gW2 = {ground_x_for_walk, ground_y_for_walk, 0};
    Eigen::Vector3d gW3 = {ground_x2_for_walk, ground_y2, 0};
    Eigen::Vector3d gW4 = {ground_x2_for_walk, ground_y_for_walk, 0};
    Eigen::Vector3d gW5 = {ground_x_for_walk, ground_y2, TunnelDepth};
    Eigen::Vector3d gW6=  {ground_x_for_walk, ground_y_for_walk, TunnelDepth};
    Eigen::Vector3d gW7 = {ground_x2_for_walk, ground_y2, TunnelDepth};
    Eigen::Vector3d gW8 = {ground_x2_for_walk, ground_y_for_walk, TunnelDepth};

    /////right and right side
    triangles.push_back({g2, g1, gW1, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({g2, gW1, gW5, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({gW1, gW2, gW5, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({gW2, gW6, gW5, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});

    float Line_width = WalkWidth/25.0;////30better??


    for(double j=ground_x2_for_walk; j < ground_x_for_walk-Line_width*0.9 ;j+= Line_width){
        Eigen::Vector3d Phi1 = {j, ground_y_for_walk, -10};
        Eigen::Vector3d Phi2 = {j, ground_y_for_walk, TunnelDepth};
        Eigen::Vector3d Phi3 = {j+Line_width, ground_y_for_walk, -10};
        Eigen::Vector3d Phi4 = {j+Line_width, ground_y_for_walk, TunnelDepth};

        if(j== ground_x2_for_walk || j >= ground_x_for_walk-Line_width*1.1 || abs(j) < Line_width){
            triangles.push_back({Phi1, Phi2, Phi3, Material(white,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi4, Phi3, Phi2, Material(white,diffuse_kd,0.0,diffuse_id)});
        }else{
            triangles.push_back({Phi1, Phi2, Phi3,Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
            triangles.push_back({Phi4, Phi3, Phi2, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
        }

    }
    ////left and left side
    triangles.push_back({g4, gW7, g3, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({gW3, g3, gW7, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({gW7, gW4, gW3, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({gW7, gW8, gW4, Material(Ground_Color,diffuse_kd,0.0,diffuse_id)});


    ////entrance edge
    triangles.push_back({g1, gW2, gW1, Material(Ground_Color*0.5,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({g3, gW3, gW4, Material(Ground_Color*0.5,diffuse_kd,0.0,diffuse_id)});


    float car_length = 4.0;
    float car_depth1 = TunnelDepth*0.18;
    float car_depth2 = car_depth1 + car_length;
    float car_height = 1.5;
    float car_width = 1.55;
    float x_car = 1.0;

    const Color obj1 = red;

    Eigen::Vector3d c1 = {x_car, ground_y_for_walk+car_height, car_depth2};
    Eigen::Vector3d c2 = {x_car, ground_y_for_walk+car_height, car_depth1};
    Eigen::Vector3d c3 = {x_car+car_width, ground_y_for_walk+car_height, car_depth1};
    Eigen::Vector3d c4 = {x_car+car_width, ground_y_for_walk+car_height, car_depth2};
    Eigen::Vector3d c5 = {x_car, ground_y_for_walk+FLT_EPSILON, car_depth2};
    Eigen::Vector3d c6=  {x_car, ground_y_for_walk+FLT_EPSILON, car_depth1};
    Eigen::Vector3d c7 = {x_car+car_width, ground_y_for_walk+FLT_EPSILON, car_depth1};
    Eigen::Vector3d c8 = {x_car+car_width, ground_y_for_walk+FLT_EPSILON, car_depth2};

    triangles.push_back({c2, c1, c3, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    ////car right side
    triangles.push_back({c1, c2, c5, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    ////car left side
    triangles.push_back({c3, c4, c7, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    ////entrance
    triangles.push_back({c2, c3, c6, Material(obj1,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(obj1,diffuse_kd,0.0,diffuse_id)});

    ////green car
    c1.x() = c2.x() = c5.x() = c6.x() = -x_car-car_width;
    c3.x() = c4.x() = c7.x() = c8.x() = -x_car;
    c1.z() = c4.z() = c5.z()= c8.z() = TunnelDepth*0.16+ car_length;
    c2.z() = c3.z() = c6.z()= c7.z() = TunnelDepth*0.16;////15>>17

    Color obj2 = green;

    ////car top
    triangles.push_back({c2, c1, c3, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c4, c3, c1, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    ////car right side
    triangles.push_back({c1, c2, c5, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c6, c5, c2, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    ////car left side
    triangles.push_back({c3, c4, c7, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c7, c4, c8, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    ////entrance
    triangles.push_back({c2, c3, c6, Material(obj2,diffuse_kd,0.0,diffuse_id)});
    triangles.push_back({c3, c7, c6, Material(obj2,diffuse_kd,0.0,diffuse_id)});

    ////SUN
//    float  Sun_z = -20;
//    Color sky = {184.0f/255.0f, 215.0f/255.0f, 236.0f/255.0f};
//
//    Eigen::Vector3d s1 = {-10, 10, Sun_z};
//    Eigen::Vector3d s2 = {-10, -10, Sun_z};
//    Eigen::Vector3d s3 = {10, -10, Sun_z};
//    Eigen::Vector3d s4 = {10, 10, Sun_z};
//
//    triangles.push_back({s2, s1, s3, Material(white,light_kd,2.0,diffuse_id)});
//    triangles.push_back({s3, s4, s1, Material(white,light_kd,2.0,diffuse_id)});
//
//
//    s1.z() = s2.z() = s3.z() = s4.z() = TunnelDepth+30;
//
//    triangles.push_back({s2, s1, s3, Material(white,light_kd,2.0,diffuse_id)});
//    triangles.push_back({s4, s3, s1, Material(white,light_kd,2.0,diffuse_id)});
//


    return triangles;
}

void Setup::TrianglesToEmbree(RTCDevice &device, RTCScene &scene, std::vector<Triangle> &triangles) {

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
                                                                         indices.size() /3);
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


}

void Setup::errorHandler(void *userPtr, RTCError code, const char *str) {
    if (code == RTC_ERROR_NONE)
        return;
    std::cerr << "Error " << code << ": " << str << std::endl;
    exit(1); // Consider proper error handling here
}




