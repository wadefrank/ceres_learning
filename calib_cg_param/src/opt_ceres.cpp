#include <iostream>
#include <cmath>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>

// using namespace std;

// // cg参数
// struct CameraGroundParameter {
//     double height = 0.0;    // m
//     double roll = 0.0;      // rad
//     double pitch = 0.0;     // rad    
//     double yaw = 0.0;       // rad
// };

// // 定义旋转矩阵（输入为弧度）
// Eigen::Matrix3d rotationMatrix(double roll, double pitch, double yaw) {
//     // 绕X轴的旋转矩阵 (Roll)
//     Eigen::Matrix3d Rx;
//     Rx << 1, 0, 0,
//           0, cos(roll), -sin(roll),
//           0, sin(roll), cos(roll);

//     // 绕Y轴的旋转矩阵 (Pitch)
//     Eigen::Matrix3d Ry;
//     Ry << cos(pitch), 0, sin(pitch),
//           0, 1, 0,
//           -sin(pitch), 0, cos(pitch);

//     // 绕Z轴的旋转矩阵 (Yaw)
//     Eigen::Matrix3d Rz;
//     Rz << cos(yaw), -sin(yaw), 0,
//           sin(yaw), cos(yaw), 0,
//           0, 0, 1;

//     // 总旋转矩阵 R = Rz * Ry * Rx
//     Eigen::Matrix3d R = Rz * Ry * Rx;
//     return R;
// }

// // 将像素坐标转换为地面坐标
// void conver_pixel_to_gound(const Eigen::Vector2d& pixel, const Eigen::Matrix3d& K, const CameraGroundParameter& cg_para, Eigen::Vector3d& point_ground) {
    
//     // 像素坐标转齐次坐标
//     Eigen::Vector3d pixel_homogeneous;
//     pixel_homogeneous << pixel[0], pixel[1], 1;

//     // 求相机内参矩阵的逆
//     Eigen::Matrix3d K_inv = K.inverse();
    
//     // 将像素坐标转换为相机坐标系的归一化方向向量
//     Eigen::Vector3d direction_camera = K_inv * pixel_homogeneous;
//     direction_camera /= direction_camera[2];

//     // 获取旋转矩阵
//     Eigen::Matrix3d R_c_g = rotationMatrix(cg_para.roll, cg_para.pitch, cg_para.yaw);
//     Eigen::Matrix3d R_g_c = R_c_g.inverse();

//     // 方向向量从相机坐标系转换到地面坐标系
//     Eigen::Vector3d direction_ground = R_g_c * direction_camera;

//     // 计算地面上的点，注意使用height进行标定
//     point_ground = direction_ground * cg_para.height / direction_ground[1];
// }



// Ceres误差模型，计算车道宽度误差
class LaneWidthError {
public:

    LaneWidthError(const Eigen::Vector2d& pixel_0, const Eigen::Vector2d& pixel_1, const double& roll, const double& pitch, const Eigen::Matrix3d& K, const double& laneWidthPrior)
        : pixel_0_(pixel_0), pixel_1_(pixel_1),
          roll_(roll), pitch_(pitch),
          K_(K), laneWidthPrior_(laneWidthPrior) {}

    template <typename T>
    bool operator()(const T* const yaw, const T* const height, T* residual) const {
        // Eigen::Matrix<T, 3, 3> Rx;
        // Rx << T(1), T(0), T(0),
        //       T(0), ceres::cos(roll_), -ceres::sin(roll_),
        //       T(0), ceres::sin(roll_),  ceres::cos(roll_);
        Eigen::Matrix<T, 3, 3> Rx;
        Rx << T(1), T(0), T(0),
              T(0), T(cos(roll_)), T(-sin(roll_)),
              T(0), T(sin(roll_)), T(cos(roll_));

        // // 绕Y轴的旋转矩阵 (Pitch)
        // Eigen::Matrix<T, 3, 3> Ry;
        // Ry << ceres::cos(pitch_), T(0), ceres::sin(pitch_),
        //       T(0), T(1), T(0),
        //      -ceres::sin(pitch_), T(0), ceres::cos(pitch_);
        Eigen::Matrix<T, 3, 3> Ry;
        Ry << T(cos(pitch_)), T(0), T(sin(pitch_)),
              T(0), T(1), T(0),
             -T(sin(pitch_)), T(0), T(cos(pitch_));

        // 绕Z轴的旋转矩阵 (Yaw)
        Eigen::Matrix<T, 3, 3> Rz;
        Rz << ceres::cos(yaw[0]), -ceres::sin(yaw[0]), T(0),
              ceres::sin(yaw[0]), ceres::cos(yaw[0]), T(0),
              T(0), T(0), T(1);

        Eigen::Matrix<T, 3, 3> R_c_g = Rz * Ry * Rx;

        // 求相机内参矩阵的逆
        Eigen::Matrix<T, 3, 3> K_T;
        K_T << T(K_(0, 0)), T(K_(0, 1)), T(K_(0, 2)),
               T(K_(1, 0)), T(K_(1, 1)), T(K_(1, 2)),
               T(K_(2, 0)), T(K_(2, 1)), T(K_(2, 2));

        Eigen::Matrix<T, 3, 1> pixel_homogeneous_0;
        pixel_homogeneous_0[0] = T(pixel_0_[0]);
        pixel_homogeneous_0[1] = T(pixel_0_[1]);
        pixel_homogeneous_0[2] = T(1);

        Eigen::Matrix<T, 3, 1> direction_camera_0;
        direction_camera_0 = K_T.inverse() * pixel_homogeneous_0;
        direction_camera_0 /= direction_camera_0[2];
        Eigen::Matrix<T, 3, 1> direction_ground_0 = R_c_g.inverse() * direction_camera_0;
        Eigen::Matrix<T, 3, 1> point_ground_0 = direction_ground_0 * height[0] / direction_ground_0[1];


        Eigen::Matrix<T, 3, 1> pixel_homogeneous_1;
        pixel_homogeneous_1[0] = T(pixel_1_[0]);
        pixel_homogeneous_1[1] = T(pixel_1_[1]);
        pixel_homogeneous_1[2] = T(1);

        Eigen::Matrix<T, 3, 1> direction_camera_1;
        direction_camera_1 = K_T.inverse() * pixel_homogeneous_1;
        direction_camera_1 /= direction_camera_1[2];
        Eigen::Matrix<T, 3, 1> direction_ground_1 = R_c_g.inverse() * direction_camera_1;
        Eigen::Matrix<T, 3, 1> point_ground_1 = direction_ground_1 * height[0] / direction_ground_1[1];

        residual[0] = T(laneWidthPrior_) - (point_ground_1[0] - point_ground_0[0]);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& pixel_0, const Eigen::Vector2d& pixel_1,
                                       const double& roll, const double& pitch,
                                       const Eigen::Matrix3d& K, const double& laneWidthPrior)
    {
        return (new ceres::AutoDiffCostFunction<LaneWidthError, 1, 1, 1>(
            new LaneWidthError(pixel_0, pixel_1, roll, pitch, K, laneWidthPrior)));
    }

private:
    Eigen::Vector2d pixel_0_, pixel_1_;
    double roll_;
    double pitch_;
    Eigen::Matrix3d K_;
    double laneWidthPrior_;
};


int main() {
    double height = 1.5;  // 相机高度
    double roll = 8.8 / 180.0 * M_PI;
    double pitch = 0.0 / 180.0 * M_PI;
    double yaw = 0.0 / 180.0 * M_PI;
    
    // 相机内参矩阵
    Eigen::Matrix3d K;
    K << 1840.0215122388756, 0.0, 963.62819621821473,
         0.0, 1840.0215122388756, 557.52823657359090,
         0.0, 0.0, 1.0;

    std::vector<Eigen::Vector2d> lane_lines;
    Eigen::Vector2d pixel_0; pixel_0 << 8, 611;     lane_lines.push_back(pixel_0);
    Eigen::Vector2d pixel_1; pixel_1 << 140, 1077;  lane_lines.push_back(pixel_1);
    Eigen::Vector2d pixel_2; pixel_2 << 1839, 1078; lane_lines.push_back(pixel_2);
    Eigen::Vector2d pixel_3; pixel_3 << 1912, 542;  lane_lines.push_back(pixel_3);

    double laneWidthPrior = 3.75;  // 假设车道宽度先验为3.5米

    // 设置优化问题
    ceres::Problem problem;

    // 对每对车道线创建优化问题
    for (size_t i = 0; i < lane_lines.size() - 1; ++i) {
        ceres::CostFunction* cost_function = LaneWidthError::Create(lane_lines[i], lane_lines[i+1], roll, pitch, K, laneWidthPrior);
        problem.AddResidualBlock(cost_function, nullptr, &yaw, &height);
    }

    // 创建误差模型并将其添加到优化问题中
    // ceres::CostFunction* cost_function = LaneWidthError::Create(pixel_0, pixel_1, roll, pitch, K, laneWidthPrior);
    // problem.AddResidualBlock(cost_function, nullptr, &yaw, &height);

    // 设置求解器选项
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_type = ceres::TRUST_REGION;
    options.max_num_iterations = 100;

    // 求解问题
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 输出优化结果
    std::cout << summary.FullReport() << std::endl;
    std::cout << "Optimized Yaw: " << yaw * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Optimized Height: " << height << " meters" << std::endl;

    return 0;
}
