#include "BALProblem.h"

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>


#include "tools/random.h"
#include "tools/rotation.h"


typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;


// FscanfOrDie函数从fptr文件中读出一个format类型(如%d)的值,赋值给value,从开头开始找到合适的就停止
// 主要是给BALProblem()构造函数读取txt数据文件用
template<typename T>
void FscanfOrDie(FILE *fptr, const char *format, T *value){
        // fcanf位于头文件<stdio.h>中，原型为int fscanf(FILE *stream,const char* format,[argument...])
        // 其功能是根据数据格式format从输入流stream中读取数据(存储到argument)，遇到空格和换行结束
        int num_scanned = fscanf(fptr, format, value); 
        if(num_scanned != 1)
            std::cerr<< "Invalid UW data file. ";
}


// 给三维向量加噪声，方便后面Perturb函数对路标点、相机旋转、相机平移等三维向量加噪声
void PerturbPoint3(const double sigma, double* point)
{
  for(int i = 0; i < 3; ++i)
    point[i] += RandNormal()*sigma;
}

// 取一个数组的中位数，用于Normalize()函数
double Median(std::vector<double>* data){
  int n = data->size();
  std::vector<double>::iterator mid_point = data->begin() + n/2;
  // nth_elelment函数，位于<algorithm>，对个定范围内的元素排序，nth位置的元素放置第nth大的值
  std::nth_element(data->begin(),mid_point,data->end());
  return *mid_point;
}

// 构造函数，将优化数据读入程序
BALProblem::BALProblem(const std::string& filename, bool use_quaternions){
  // 只读模式打开名为filename的数据文件，并将文件地址赋值为文件指针fptr
  FILE* fptr = fopen(filename.c_str(), "r");

  // 文件为空报警
  if (fptr == NULL) {
    std::cerr << "Error: unable to open file " << filename;
    return;
  };


  // This wil die horribly on invalid files. Them's the breaks.
  // FscanfDie读取文件的前三个double类型的值(相机数、路标点、观测数)，直接读入到类成员中
  FscanfOrDie(fptr, "%d", &num_cameras_);   // fptr是指针，所以第三个参数value是地址
  FscanfOrDie(fptr, "%d", &num_points_);
  FscanfOrDie(fptr, "%d", &num_observations_);

  std::cout << "Header: " << num_cameras_
            << " " << num_points_
            << " " << num_observations_;

  // txt相机索引那一列，存在point_index数组，大小为观测个数
  point_index_ = new int[num_observations_];
  // txt路标点索引那一列，存在camera_index_数组，大小为观测个数
  camera_index_ = new int[num_observations_];
  // 每个观测是个二维坐标，大小为2倍
  observations_ = new double[2 * num_observations_];


  // 所有要优化的参数量，相机个数*9维，路标点个数*3维
  num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
  // parameters_数组存储优化变量的值
  parameters_ = new double[num_parameters_];

  // 此处对txt文件进行读取，按照第i个相机、第j个路标、两个像素坐标顺序读取
  for (int i = 0; i < num_observations_; ++i) {
    // FscanfOrDie读取txt中的一行
    FscanfOrDie(fptr, "%d", camera_index_ + i);
    FscanfOrDie(fptr, "%d", point_index_ + i);
    for (int j = 0; j < 2; ++j) {
      FscanfOrDie(fptr, "%lf", observations_ + 2*i + j); // 读取像素二维坐标 
    }
  }

  // 读取txt后半部分，就是所有优化变量具体值
  for (int i = 0; i < num_parameters_; ++i) {
    FscanfOrDie(fptr, "%lf", parameters_ + i);
  }

  fclose(fptr);

  use_quaternions_ = use_quaternions;

  // 读入四元数形式的数据
  if (use_quaternions) {
    // Switch the angle-axis rotations to quaternions.
    num_parameters_ = 10 * num_cameras_ + 3 * num_points_;
    double* quaternion_parameters = new double[num_parameters_];
    double* original_cursor = parameters_; // parameters_优化变量具体值复制到original_cursor中
    double* quaternion_cursor = quaternion_parameters;
    for (int i = 0; i < num_cameras_; ++i) {
      AngleAxisToQuaternion(original_cursor, quaternion_cursor);
      quaternion_cursor += 4; // 四元数是四位所以指针前进四位
      original_cursor += 3;   // 角轴三位
      for (int j = 4; j < 10; ++j) { 
       *quaternion_cursor++ = *original_cursor++; // 其余量直接赋值
      }
    }
    // Copy the rest of the points.
    // 空间点不需要变换
    for (int i = 0; i < 3 * num_points_; ++i) {
      *quaternion_cursor++ = *original_cursor++;
    }
    // Swap in the quaternion parameters.
    delete []parameters_;
    parameters_ = quaternion_parameters;
  }
}

// 类成员中存储的数据生成一个待优化数据txt
void BALProblem::WriteToFile(const std::string& filename)const{
  // c_str()返回指向字符串常量的指针，主要是为了兼容c
  // fopne需要指针参数，filename是string类型，需要用c_str()返回文件名字字符串的指针
  FILE* fptr = fopen(filename.c_str(),"w");
  
  if(fptr == NULL)
  {
    std::cerr<<"Error: unable to open file "<< filename;
    return;
  }

  fprintf(fptr, "%d %d %d %d\n", num_cameras_, num_cameras_, num_points_, num_observations_);
  // 每行循环，每行四部分：相机、路标、二维观测
  for(int i = 0; i < num_observations_; ++i){
    // 形式如(0,0,(u,v))
    fprintf(fptr, "%d %d", camera_index_[i], point_index_[i]);
    // 二维坐标观测
    for(int j = 0; j < 2; ++j){
      fprintf(fptr, " %g", observations_[2*i + j]);
    }
    fprintf(fptr,"\n");
  }

  // 输出9维的相机参数
  for(int i = 0; i < num_cameras(); ++i)
  {
    double angleaxis[9];
    if(use_quaternions_){
      //OutPut in angle-axis format.
      QuaternionToAngleAxis(parameters_ + 10 * i, angleaxis);
      memcpy(angleaxis + 3, parameters_ + 10 * i + 4, 6 * sizeof(double));
    }else{
      // memcpy()函数复制，长度为9，步长为9，将parameters_数组中的数据每次9个放到angleaxis中
      memcpy(angleaxis, parameters_ + 9 * i, 9 * sizeof(double));
    }
    for(int j = 0; j < 9; ++j)
    {
      fprintf(fptr, "%.16g\n",angleaxis[j]);
    }
  }

  // 输出三维路标点
  const double* points = parameters_ + camera_block_size() * num_cameras_;
  for(int i = 0; i < num_points(); ++i){
    const double* point = points + i * point_block_size();
    for(int j = 0; j < point_block_size(); ++j){
      fprintf(fptr,"%.16g\n",point[j]);
    }
  }

  fclose(fptr);
}

// Write the problem to a PLY file for inspection in Meshlab or CloudCompare
void BALProblem::WriteToPLYFile(const std::string& filename)const{
  std::ofstream of(filename.c_str());

  of<< "ply"
    << '\n' << "format ascii 1.0"
    << '\n' << "element vertex " << num_cameras_ + num_points_
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "end_header" << std::endl;

    // Export extrinsic data (i.e. camera centers) as green points.
    // 创建两个数组，用于承接CameraToAngelAxisAndCenter()解析出来的相机旋转姿态和相机位置中心
    double angle_axis[3];
    double center[3];
    // 循环写入，首先是相机中心点参数，个数为相机数据个数
    for(int i = 0; i < num_cameras(); ++i){
      const double* camera = cameras() + camera_block_size() * i;
      // 用CameraToAngleAxisAndCenter()函数从相机参数中解析处相机位姿
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      of << center[0] << ' ' << center[1] << ' ' << center[2]
         << "0 255 0" << '\n'; // 颜色
    }

    // Export the structure (i.e. 3D Points) as white points.
    const double* points = parameters_ + camera_block_size() * num_cameras_;
    for(int i = 0; i < num_points(); ++i){
      const double* point = points + i * point_block_size();
      for(int j = 0; j < point_block_size(); ++j){
        of << point[j] << ' ';
      }
      of << "255 255 255\n";
    }
    of.close();
}

/**
 * 由camera数据中的旋转向量和平移向量解析出相机世界坐标系下的姿态(依旧是旋转向量)和位置(世界坐标系下的xyz)，也是用于生成点云用的
 * @param camera 要解析的相机参数，前三维旋转，接着三维平移，这里只用到这6维
 * @param angle_axis 解析出的相机姿态承接数组，也是旋转向量形式
 * @param center 解析出来的相机原点在世界坐标系下的坐标承接数组，XYZ
 */
void BALProblem::CameraToAngelAxisAndCenter(const double* camera, 
                                            double* angle_axis,
                                            double* center) const{
    // 先把数组变成矩阵angle_axis_ref
    VectorRef angle_axis_ref(angle_axis,3);
    if(use_quaternions_){
      QuaternionToAngleAxis(camera, angle_axis);
    }else{
      angle_axis_ref = ConstVectorRef(camera,3);
    }

    // c = -R't c是世界坐标系下的相机原点坐标
    // PW_center*R+t = PC+center，PC_center = (0,0,0)，PW_center = -R^(-1)*t
    Eigen::VectorXd inverse_rotation = -angle_axis_ref; // 旋转向量求逆（取负数）
    // 计算R*p，结果存储在result中，此处为计算R^(-1)*t
    AngleAxisRotatePoint(inverse_rotation.data(),  // R^(-1)
                         camera + camera_block_size() - 6,  // 指针偏移获得平移量t，0-2是旋转，3-5是平移，6-8是相机内参
                         center);
    // map类型构造的是引用，可直接对原构造数组进行操作
    VectorRef(center,3) *= -1.0;
}

/**
 * 反向过程，由世界坐标系下的相机姿态和原点位置，生成一个camera数据
 * @param angle_axis 旋转向量数据
 * @param center 相机中心在世界坐标系下的位置坐标
 * @param camera 承接数据的camera数组，由于这里只是生成旋转和平移，所以是camera的前6维
 */
void BALProblem::AngleAxisAndCenterToCamera(const double* angle_axis,
                                            const double* center,
                                            double* camera) const{
    ConstVectorRef angle_axis_ref(angle_axis,3);
    if(use_quaternions_){
      AngleAxisToQuaternion(angle_axis,camera);
    }
    else{
      VectorRef(camera, 3) = angle_axis_ref;
    }

    // t = -R * c 
    AngleAxisRotatePoint(angle_axis,center,camera+camera_block_size() - 6);
    VectorRef(camera + camera_block_size() - 6,3) *= -1.0;
}

// 数据初始化
void BALProblem::Normalize(){
  // Compute the marginal median of the geometry
  std::vector<double> tmp(num_points_);
  Eigen::Vector3d median;
  double* points = mutable_points();
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < num_points_; ++j){
      tmp[j] = points[3 * j + i];      
    }
    // Median函数获得x、y、z的中值
    median(i) = Median(&tmp);
  }

  for(int i = 0; i < num_points_; ++i){
    VectorRef point(points + 3 * i, 3);
    tmp[i] = (point - median).lpNorm<1>(); // L1范数，向量各个元素绝对值之和
  }
  // tmp存储空间点相对于中值点的偏差的L1范数
  const double median_absolute_deviation = Median(&tmp);

  // Scale so that the median absolute deviation of the resulting
  // reconstruction is 100

  const double scale = 100.0 / median_absolute_deviation;

  // X = scale * (X - median)
  for(int i = 0; i < num_points_; ++i){
    VectorRef point(points + 3 * i, 3);
    point = scale * (point - median);
  }

  double* cameras = mutable_cameras();
  double angle_axis[3];
  double center[3];
  for(int i = 0; i < num_cameras_ ; ++i){
    double* camera = cameras + camera_block_size() * i;
    CameraToAngelAxisAndCenter(camera, angle_axis, center);
    // center = scale * (center - median)
    VectorRef(center,3) = scale * (VectorRef(center,3)-median);
    AngleAxisAndCenterToCamera(angle_axis, center,camera);
  }
}

// 添加噪声
void BALProblem::Perturb(const double rotation_sigma, 
                         const double translation_sigma,
                         const double point_sigma){
   assert(point_sigma >= 0.0);
   assert(rotation_sigma >= 0.0);
   assert(translation_sigma >= 0.0);

   double* points = mutable_points();
   if(point_sigma > 0){
     for(int i = 0; i < num_points_; ++i){
       PerturbPoint3(point_sigma, points + 3 * i);
     }
   }
  
  // 相机加噪声分为旋转和平移两部分
   for(int i = 0; i < num_cameras_; ++i){
     double* camera = mutable_cameras() + camera_block_size() * i;

     double angle_axis[3];
     double center[3];
     // Perturb in the rotation of the camera in the angle-axis
     // representation
     CameraToAngelAxisAndCenter(camera, angle_axis, center); // 从camera中取出三维的angle_axis
     if(rotation_sigma > 0.0){
       PerturbPoint3(rotation_sigma, angle_axis);
     }
     AngleAxisAndCenterToCamera(angle_axis, center,camera);

     if(translation_sigma > 0.0)
        PerturbPoint3(translation_sigma, camera + camera_block_size() - 6);
   }
}