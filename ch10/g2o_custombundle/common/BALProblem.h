#ifndef BALPROBLEM_H
#define BALPROBLEM_H

#include <stdio.h>
#include <string>
#include <iostream>

// BALProblem类，存储相机和路标点之间的关联、相机和路标的初始值，数据导出为PLY文件
class BALProblem
{
public:
    explicit BALProblem(const std::string& filename, bool use_quaternions = false);
    ~BALProblem(){
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    void WriteToFile(const std::string& filename)const;
    void WriteToPLYFile(const std::string& filename)const;

    void Normalize();

    void Perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);
    
    // 相机参数个数:四元数10维，旋转向量9维
    // 9个参数：3 个表示旋转, 3 个表示平移, 1个焦距，2个径向畸变(二阶径向畸变系数，四阶径向畸变系数)
    int camera_block_size()             const{ return use_quaternions_? 10 : 9;  }
    // 路标点的维度3维
    int point_block_size()              const{ return 3;                         }             
    int num_cameras()                   const{ return num_cameras_;              }
    int num_points()                    const{ return num_points_;               }
    int num_observations()              const{ return num_observations_;         }
    int num_parameters()                const{ return num_parameters_;           }
    const int* point_index()            const{ return point_index_;              }
    const int* camera_index()           const{ return camera_index_;             }
    const double* observations()        const{ return observations_;             }

    // parameters存储待优化变量
    const double* parameters()          const{ return parameters_;               }
    const double* cameras()             const{ return parameters_;               }
    // 返回路标点数据列的开头位置
    const double* points()              const{ return parameters_ + camera_block_size() * num_cameras_; }
    double* mutable_cameras()                { return parameters_;               }
    double* mutable_points()                 { return parameters_ + camera_block_size() * num_cameras_; }

    // 返回第i个观测中的相机数据
    double* mutable_camera_for_observation(int i){
        return mutable_cameras() + camera_index_[i] * camera_block_size();
    }
    
    // 返回第i个观测中的路标数据
    double* mutable_point_for_observation(int i){
        return mutable_points() + point_index_[i] * point_block_size();
    }

    const double* camera_for_observation(int i)const {
        return cameras() + camera_index_[i] * camera_block_size();
    }

    const double* point_for_observation(int i)const {
        return points() + point_index_[i] * point_block_size();
    }


private:
    void CameraToAngelAxisAndCenter(const double* camera,
                                    double* angle_axis,
                                    double* center)const;

    void AngleAxisAndCenterToCamera(const double* angle_axis,
                                    const double* center,
                                    double* camera)const;

    // 这几个参数就是将txt数据分块读取和写入的承接变量和数组
    // 相机个数
    int num_cameras_;
    // 路标点个数
    int num_points_;
    // 观测个数
    int num_observations_;
    // 待优化变量的具体的参数的个数
    int num_parameters_; // num_parameters = 9 * num_cameras_ + 3 * num_points_
    bool use_quaternions_; 

    // 观测中的路标点数组
    int* point_index_;
    // 相机数组
    int* camera_index_;
    // 观测的索引述组
    double* observations_;
    // 优化参数值数组，其实这里就是个指针，指向txt的后半部分参数的开头
    double* parameters_; 

};

#endif // BALProblem.h