#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <config.hpp>


namespace TR_exca{

    using Eigen::Vector3d; 


    class initializer
    {

        private: 
            twist gen_twist(Vector3d input_w, Vector3d input_q)
            {
                twist output_twist;
                output_twist.w = input_w;
                output_twist.q = input_q;
                output_twist.v = -input_w.cross(input_q);
            }

        public :
            Param state_initialize()
            {
                Param output_state; 
                output_state.arm_angle = 0;
                output_state.bkt_angle = 0;
                output_state.boom_angle = 0;
                output_state.tilt = 0;
                output_state.rotate = 0; 
                output_state.swing = 0;

                output_state.mn_rel_x = 0;
                output_state.mn_rel_y = 0;
                output_state.mn_rel_z = 0;
                output_state.mn_rel_ori = Eigen::MatrixXd::Identity(3,3); 

                output_state.mb_x = 0;
                output_state.mb_y = 0;
                output_state.mb_z = 0;
                output_state.mn_rel_ori = Eigen::MatrixXd::Identity(3,3); 

                output_state.L01 = 1.19; 
                output_state.L12 = 0.05;
                output_state.L23 = 0.2;
                output_state.L34 = 0.93;
                output_state.L45 = 6.25;
                output_state.L56 = 3.05;
                output_state.L67 = 0.34;
                output_state.L78 = 0.41; 
                output_state.L89 = 0.485;
                output_state.L910 = 0.24;

                std::vector<Vector3d> w_col;
                Vector3d w1(0,0,1);
                Vector3d w2(0,-1,0);
                Vector3d w3(0,-1,0);
                Vector3d w4(0,-1,0);
                Vector3d w5(0,0,1);
                Vector3d w6(1,0,0);
                w_col.push_back(w1);
                w_col.push_back(w2);                
                w_col.push_back(w3);
                w_col.push_back(w4);
                w_col.push_back(w5);
                w_col.push_back(w6);

                std::vector<Vector3d> q_col;
                Vector3d q1(0,0,output_state.L01);
                Vector3d q2(output_state.L23,-output_state.L12,output_state.L01+output_state.L34);
                Vector3d q3(output_state.L23+output_state.L45,-output_state.L12,output_state.L01+output_state.L34);
                Vector3d q4(output_state.L23+output_state.L45+output_state.L56,-output_state.L12,output_state.L01+output_state.L34);
                Vector3d q5(output_state.L23+output_state.L45+output_state.L56+output_state.L67,-output_state.L12,output_state.L01+output_state.L34);
                Vector3d q6(output_state.L23+output_state.L45+output_state.L56+output_state.L67,-output_state.L12,output_state.L01+output_state.L34+output_state.L78);
                q_col.push_back(q1);
                q_col.push_back(q2);
                q_col.push_back(q3);
                q_col.push_back(q4);
                q_col.push_back(q5);
                q_col.push_back(q6);

                for (int i =0 ; i<6 ; i++)
                {
                    twist tmp_twist = gen_twist(w_col[i],q_col[i]);
                    output_state.twist_col.push_back(tmp_twist);
                }
                

                return output_state; 
            }


    };
}