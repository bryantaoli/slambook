#include <iostream>
#include <fstream>
using namespace std;
    
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
在矩阵运算时调用了 Eigen 库,为了提高运算速度,向量化必须要求向量是以 16 字节即 128bit 对齐的方式分配内存空间,
所以针对这个问题,容器需要使用 eigen 自己定义的内存分配器。对 Eigen 中的固定大小的类使用 STL 容器的时候,如果
直接使用就会出错。此时应加上头文件include<Eigen/StdVector>
*/
#include <Eigen/StdVector>
    
#include <octomap/octomap.h>    // for octomap 
    
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
    
int main( int argc, char** argv )
{
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d> > poses;         // 相机位姿,此处也应修改
    
    ifstream fin("./data/pose.txt");
    if (!fin)
    {
        cerr<<"cannot find pose file"<<endl;
        return 1;
    }
    
    for ( int i=0; i<5; i++ )
    {
        boost::format fmt( "./data/%s/%d.%s" ); //图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
        double data[7] = {0};
        for ( int i=0; i<7; i++ )
        {
            fin>>data[i];
        }
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d T(q);
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
        poses.push_back( T );
    }
    
    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为 Octomap ..."<<endl;
    
    // octomap tree 
    octomap::OcTree tree( 0.05 ); // 参数为分辨率
    for ( int i=0; i<5; i++ )
    {
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        
        octomap::Pointcloud cloud;  // the point cloud in octomap 
        
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 7000 ) continue; // 深度太大时不稳定，去掉
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
                // 将世界坐标系的点放入点云
                cloud.push_back( pointWorld[0], pointWorld[1], pointWorld[2] ); 
            }
            
        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        tree.insertPointCloud( cloud, octomap::point3d( T(0,3), T(1,3), T(2,3) ) );     
    }
    
    // 更新中间节点的占据信息并写入磁盘
    tree.updateInnerOccupancy();
    cout<<"saving octomap ... "<<endl;
    tree.writeBinary( "octomap.bt" );
    return 0;
}
// #include <iostream>
// #include <fstream>
// using namespace std;

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

// #include <octomap/octomap.h>    // for octomap 

// #include <Eigen/Geometry> 
// #include <boost/format.hpp>  // for formating strings

// int main( int argc, char** argv )
// {
//     vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
//     vector<Eigen::Isometry3d> poses;         // 相机位姿
    
//     ifstream fin("./data/pose.txt");
//     if (!fin)
//     {
//         cerr<<"cannot find pose file"<<endl;
//         return 1;
//     }
    
//     for ( int i=0; i<5; i++ )
//     {
//         boost::format fmt( "./data/%s/%d.%s" ); //图像文件格式
//         colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() ));
//         depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 使用-1读取原始图像
        
//         double data[7] = {0};
//         for ( int i=0; i<7; i++ )
//         {
//             fin>>data[i];
//         }
//         Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
//         Eigen::Isometry3d T(q);
//         T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] ));
//         poses.push_back( T );
//     }
    
//     // 计算点云并拼接
//     // 相机内参 
//     double cx = 325.5;
//     double cy = 253.5;
//     double fx = 518.0;
//     double fy = 519.0;
//     double depthScale = 1000.0;
    
//     cout<<"正在将图像转换为 Octomap ..."<<endl;
    
//     // octomap tree 
//     octomap::OcTree tree( 0.05 ); // 参数为分辨率
    
//     for ( int i=0; i<5; i++ )
//     {
//         cout<<"转换图像中: "<<i+1<<endl; 
//         cv::Mat color = colorImgs[i]; 
//         cv::Mat depth = depthImgs[i];
//         Eigen::Isometry3d T = poses[i];
        
//         octomap::Pointcloud cloud;  // the point cloud in octomap 
        
//         for ( int v=0; v<color.rows; v++ )
//             for ( int u=0; u<color.cols; u++ )
//             {
//                 unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
//                 if ( d==0 ) continue; // 为0表示没有测量到
//                 if ( d >= 7000 ) continue; // 深度太大时不稳定，去掉
//                 Eigen::Vector3d point; 
//                 point[2] = double(d)/depthScale; 
//                 point[0] = (u-cx)*point[2]/fx;
//                 point[1] = (v-cy)*point[2]/fy; 
//                 Eigen::Vector3d pointWorld = T*point;
//                 // 将世界坐标系的点放入点云
//                 cloud.push_back( pointWorld[0], pointWorld[1], pointWorld[2] ); 
//             }
            
//         // 将点云存入八叉树地图，给定原点，这样可以计算投射线
//         tree.insertPointCloud( cloud, octomap::point3d( T(0,3), T(1,3), T(2,3) ) );     
//     }
    
//     // 更新中间节点的占据信息并写入磁盘
//     tree.updateInnerOccupancy();
//     cout<<"saving octomap ... "<<endl;
//     tree.writeBinary( "octomap.bt" );
//     return 0;
// }