#ifndef OV_CORE_CAM_BASE_H
#define OV_CORE_CAM_BASE_H
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

class CamBase {
    public:
        // 初始化相机的宽度 _width 和高度 _height，即相机拍摄图像的分辨率
        CamBase(int width, int height): _width(width), _height(height) {}
        virtual ~CamBase() {}

        
        // 相机内参设置
        virtual void set_value(const Eigen::MatrixXd& calib) {
            assert(calib.rows() == 8);
            camera_values = calib;

            cv::Matx33d tempK;
            tempK(0,0) = calib(0);
            tempK(0,1) = 0;
            tempK(0,2) = calib(2);

            tempK(1,0) = 0;
            tempK(1,1) = calib(1);
            tempK(1,2) = calib(3);

            tempK(2,0) = 0;
            tempK(2,1) = 0;
            tempK(2,2) = 1;


            camera_k_OPENCV = tempK;

            cv::Vec4d tempD;
            tempD[0] = calib(4);
            tempD[1] = calib(5);
            tempD[2] = calib(6);
            tempD[3] = calib(7);
            camera_d_OPENCV = tempD;
        }

        // 像素坐标去畸变
        virtual Eigen::Vector2f undistort_f(const Eigen::Vector2f &uv_dist) = 0;

        // 双精度版本
        Eigen::Vector2d undistort_d(const Eigen::Vector2d &uv_dist) {
            Eigen::Vector2f ept1, ept2;
            ept1 = uv_dist.cast<float>();
            ept2 = undistort_f(ept1);
            return ept2.cast<double>();
        }

        // opencv 版本去畸变
        cv::Point2f undistort_cv(const cv::Point2f &uv_dist) {
            Eigen::Vector2f ept1, ept2;
            ept1 << uv_dist.x, uv_dist.y;
            ept2 = undistort_f(ept1);
            cv::Point2f pt_out;
            pt_out.x = ept2.x();
            pt_out.y = ept2.y();
            return pt_out;
        }

        // 将归一化坐标映射回畸变的原始像素坐标
        virtual Eigen::Vector2f distort_f(const Eigen::Vector2f &uv_norm) = 0;

        Eigen::Vector2d distort_d(const Eigen::Vector2d &uv_norm) {
            Eigen::Vector2f ept1, ept2;
            ept1 = uv_norm.cast<float>();
            ept2 = distort_f(ept1);
            return ept2.cast<double>();
        }

        cv::Point2f distort_cv(const cv::Point2f &uv_norm) {
            Eigen::Vector2f ept1, ept2;
            ept1 << uv_norm.x, uv_norm.y;
            ept2 = distort_f(ept1);
            cv::Point2f pt_out;
            pt_out.x = ept2.x();
            pt_out.y = ept2.y();
            return pt_out;
        }

        //Todo 归一化坐标系计算jacobian
        // virtual Eigen::Matrix2d compute_distort_jacobian(const Eigen::Vector2f &uv_norm, Eigen::Ma) = 0;

        // 得到内参系数
        Eigen::MatrixXd get_value() {
            return camera_values;
        }
        // 得到内参矩阵
        cv::Matx33d get_K() {
            return camera_k_OPENCV;
        }

        // 得到畸变系数
        cv::Vec4d get_D() {
            return camera_d_OPENCV;
        }

        // 得到图像宽度
        int get_width() {
            return _width;
        }

        // 得到图像高度
        int get_height() {
            return _height;
        }
    private:
        CamBase() = default;
        Eigen::MatrixXd camera_values;
        cv::Matx33d camera_k_OPENCV;
        cv::Vec4d camera_d_OPENCV;
        int _width;
        int _height;
};
#endif