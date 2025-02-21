#ifndef OV_CORE_CAM_RADTAN_H
#define OV_CORE_CAM_RADTAN_H

#include "CamBase.h"

class CamRadtan : public CamBase {
    public:
        CamRadtan(int width, int height):CamBase(width, height) {};
        ~CamRadtan() {}

        // 定义一个函数，用于对畸变点进行去畸变处理
        Eigen::Vector2f undistort_f(const Eigen::Vector2f& uv_dist) override {
            // 从类成员变量中获取相机的内参矩阵和畸变系数
            cv::Matx33d camK = camera_k_OPENCV;
            cv::Vec4d camD = camera_d_OPENCV;

            // 创建一个OpenCV矩阵，用于存储畸变点的坐标
            cv::mat(1,2,CV_32F);
            // 将输入的畸变点坐标赋值到OpenCV矩阵中
            mat.at<float>(0,0) = uv_dist(0);
            mat.at<float>(0,1) = uv_dist(1);

            // 将矩阵的形状调整为2行1列，以便后续处理
            mat =mat.reshape(2);
            // 再次将输入的畸变点坐标赋值到调整形状后的矩阵中
            mat.at<float>(0,0) = uv_dist(0);
            mat.at<float>(0,1) = uv_dist(1);

            // 使用OpenCV的undistortPoints函数对畸变点进行去畸变处理
            cv::undistortPoints(mat, mat, camK, camD);

            // 创建一个Eigen向量，用于存储去畸变后的点坐标
            Eigen::Vector2f pt_out;
            // 将去畸变后的点坐标从OpenCV矩阵转换回Eigen向量
            mat = mat.reshape(1);
            pt_out(0) = mat.at<float>(0,0);
            pt_out(1) = mat.at<float>(0,1);
            // 返回去畸变后的点坐标
            return pt_out;
        }
        
        
        // 对归一化相机坐标进行畸变处理
        Eigen::Vector2f distort_f(const Eigen::Vector2f& uv_norm) override {
            Eigen::MatrixXd cam_d = camera_values;

            // 应用径向与切向畸变模型
            double r_2 = uv_norm(0)*uv_norm(0) + uv_norm(1)*uv_norm(1);
            double r_4 = r_2 * r_2;
            double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 
                    2 * cam_d(6) * uv_norm(0) * uv_norm(1) + cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
            
            double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 
                    cam_d(6)* (r_2 + 2 * uv_norm(1) * uv_norm(1)) + 2 * cam_d(7) * uv_norm(0) * uv_norm(1);

            Eigen::Vector2f uv_dist;
            uv_dist(0) = (float)(cam_d(0) * x1 + cam_d(2));
            uv_dist(1) = (float)(cam_d(1) * y1 + cam_d(3));
            return uv_dist;
        };
        
        // 计算归一化相机坐标到畸变坐标的雅可比矩阵
        void compute_distort_jacobian(const Eigen::Vector2d &uv_norm, Eigen::MatrixXd &H_dz_dzn, Eigen::MatrixXd &H_dz_dzeta) override {

        // Get our camera parameters
        Eigen::MatrixXd cam_d = camera_values;

        // Calculate distorted coordinates for radial
        double r = std::sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
        double r_2 = r * r;
        double r_4 = r_2 * r_2;

        // Jacobian of distorted pixel to normalized pixel
        H_dz_dzn = Eigen::MatrixXd::Zero(2, 2);
        double x = uv_norm(0);
        double y = uv_norm(1);
        double x_2 = uv_norm(0) * uv_norm(0);
        double y_2 = uv_norm(1) * uv_norm(1);
        double x_y = uv_norm(0) * uv_norm(1);
        H_dz_dzn(0, 0) = cam_d(0) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * x_2 + 4 * cam_d(5) * x_2 * r_2) +
                                    2 * cam_d(6) * y + (2 * cam_d(7) * x + 4 * cam_d(7) * x));
        H_dz_dzn(0, 1) = cam_d(0) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
        H_dz_dzn(1, 0) = cam_d(1) * (2 * cam_d(4) * x_y + 4 * cam_d(5) * x_y * r_2 + 2 * cam_d(6) * x + 2 * cam_d(7) * y);
        H_dz_dzn(1, 1) = cam_d(1) * ((1 + cam_d(4) * r_2 + cam_d(5) * r_4) + (2 * cam_d(4) * y_2 + 4 * cam_d(5) * y_2 * r_2) +
                                    2 * cam_d(7) * x + (2 * cam_d(6) * y + 4 * cam_d(6) * y));

        // Calculate distorted coordinates for radtan
        double x1 = uv_norm(0) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + 2 * cam_d(6) * uv_norm(0) * uv_norm(1) +
                    cam_d(7) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
        double y1 = uv_norm(1) * (1 + cam_d(4) * r_2 + cam_d(5) * r_4) + cam_d(6) * (r_2 + 2 * uv_norm(1) * uv_norm(1)) +
                    2 * cam_d(7) * uv_norm(0) * uv_norm(1);

        // Compute the Jacobian in respect to the intrinsics
        H_dz_dzeta = Eigen::MatrixXd::Zero(2, 8);
        H_dz_dzeta(0, 0) = x1;
        H_dz_dzeta(0, 2) = 1;
        H_dz_dzeta(0, 4) = cam_d(0) * uv_norm(0) * r_2;
        H_dz_dzeta(0, 5) = cam_d(0) * uv_norm(0) * r_4;
        H_dz_dzeta(0, 6) = 2 * cam_d(0) * uv_norm(0) * uv_norm(1);
        H_dz_dzeta(0, 7) = cam_d(0) * (r_2 + 2 * uv_norm(0) * uv_norm(0));
        H_dz_dzeta(1, 1) = y1;
        H_dz_dzeta(1, 3) = 1;
        H_dz_dzeta(1, 4) = cam_d(1) * uv_norm(1) * r_2;
        H_dz_dzeta(1, 5) = cam_d(1) * uv_norm(1) * r_4;
        H_dz_dzeta(1, 6) = cam_d(1) * (r_2 + 2 * uv_norm(1) * uv_norm(1));
        H_dz_dzeta(1, 7) = 2 * cam_d(1) * uv_norm(0) * uv_norm(1);
    } else {
        
    }
}

#endif