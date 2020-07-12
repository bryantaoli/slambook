#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>


// 使用sophus库之前先在用cmake命令编译
#include "sophus/so3.h"
#include "sophus/se3.h"


int main(int argc, char **argv) {
    // 沿Z轴转90度的旋转矩阵
    Eigen::AngleAxisd rotationVector(M_PI/2, Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R = rotationVector.toRotationMatrix();
    
    // 旋转矩阵构造
    Sophus::SO3 SO3_R(R);    
    // 旋转向量构造
    Sophus::SO3 SO3_v(0,0,M_PI/2);  // rot_x, rot_y, rot_z
    // 或者
    Sophus::SO3 SO3_v_1((rotationVector.axis()*rotationVector.angle())(0),(rotationVector.axis()*rotationVector.angle())(1),(rotationVector.axis()*rotationVector.angle())(2)); 
    // 四元数构造
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);
    
    // 以so(3)形式输出
    cout<<rotationVector.angle()<<endl<<rotationVector.axis()<<endl;
    cout<<"SO(3) from matrix:"<<SO3_R<<endl;
    cout<<"SO(3) from vector:"<<SO3_v<<'\t'<<SO3_v_1<<endl;
    cout<<"SO(3) from quaternoin:"<<SO3_q<<endl;
    
    // 对数映射
    Eigen::Vector3d so3 = SO3_R.log(); //SO3(R)->so3(φ)
    cout<<"so3 = "<<so3.transpose()<<endl;  // 转置输出
    // hat为向量到反对称矩阵的映射
    cout<<"so3_hat = "<<Sophus::SO3::hat(so3)<<endl;
    // vee为反对称矩阵到向量的映射
    cout<<"so3_hat_vee = "<<Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose()<<endl;
    
    //  增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4,0,0);
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
    cout<<"SO3_updated = "<<SO3_updated<<endl;
    
    // 对SE（3）的操作与上述基本类似
    // 沿X轴平移t
    Eigen::Vector3d t(1,0,0);   
    // R,t构造SE（3）
    Sophus::SE3 SE3_Rt(R,t); 
    // q,t构造SE（3）
    Sophus::SE3 SE3_qt(q,t);
    // 虽然SE3是4*4的矩阵，但是输出还是以六维向量的形式输出
    cout<<"SE3 from R,t = "<<endl<<SE3_Rt<<endl;  
    cout<<"SE3 from q,t = "<<endl<<SE3_qt<<endl;
    
    // 定义se(3)(六维向量形式）
    typedef Eigen::Matrix<double,6,1> Vector6d;   //Vector6d相当于一个新的数据类型
    Vector6d se3 = SE3_Rt.log();
    cout<<"se3 = "<<se3.transpose()<<endl;
    // hat
    cout<<"se3_hat"<<endl<<Sophus::SE3::hat(se3)<<endl;
    // vee
    cout<<"se3_hat_vee = "<<Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose()<<endl;
    
    // 更新
    Vector6d update_se3;
    update_se3.setZero();  // 初始化为0
    update_se3(0,0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    cout<<"SE3_updated = "<<endl<<SE3_updated.matrix()<<endl;
   
    return 0;
}