#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

namespace TR_exca {

    using Eigen::Matrix3d;
    using Eigen::Vector3d;


    struct twist
    {
        Vector3d w;
        Vector3d q;
        Vector3d v;
    };

    struct Param
    {
        /// link parameters 
        float L01; 
        float L12;
        float L23;
        float L34;
        float L45;
        float L56;
        float L67;
        float L78; 
        float L89;
        float L910;

        /// moblie base com position & orientaion
        float mb_x;
        float mb_y;
        float mb_z; 

        Matrix3d mb_ori;

        // base - manipulator relative pos & ori
        float mn_rel_x;
        float mn_rel_y;
        float mn_rel_z; 

        Matrix3d mn_rel_ori; 

        // manipulator joint values;
        float swing;
        float boom_angle;
        float arm_angle;
        float bkt_angle;
        float tilt;
        float rotate; 

        std::vector<twist> twist_col; 
        
    };
} // namespace TR_exca 