#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/base_vertex.h>

#include <sophus/se3.h>
#include <sophus/so3.h>
using namespace std;
using Sophus::SE3;
using Sophus::SO3;

typedef Eigen::Matrix<double,6,6> Matrix6d;

// 位姿图优化就是只优化位姿 不优化路标点
// 顶点应该相机的位姿
// 边是相邻两个位姿的变换
// error误差是观测的相邻相机的位姿变换的逆 * 待优化的相邻相机的位姿变换

// 给定误差求J_R^(-1)的近似
Matrix6d JRInv(SE3 e)
{
    Matrix6d J;
    // matrix.block(i,j,p,q) 提取块大小为(p,q)，起始于(i,j)
    J.block(0,0,3,3) = SO3::hat(e.so3().log());
    J.block(0,3,3,3) = SO3::hat(e.translation());
    J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
    J.block(3,3,3,3) = SO3::hat(e.so3().log());
    J = J*0.5 + Matrix6d::Identity();  // J^(-1) = I + [phi^ rho^;0 phi^]
    return J;
}

// 李代数顶点
typedef Eigen::Matrix<double,6,1> Vector6d;
class VertexSE3LieAlgebra: public g2o::BaseVertex<6,SE3>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool read(istream& is){
        double data[7];
        for(int i = 0;i < 7;i++)
            is>>data[i];
        setEstimate(SE3(   // 用四元数和平移向量初始化SE3
            Eigen::Quaterniond(data[6],data[3],data[4],data[5]),
            Eigen::Vector3d(data[0],data[1],data[2])
            ));
    }

    bool write(ostream& os) const{
        os<<id()<<" ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();  // unit_quaternion()获取单位四元数
        os<<_estimate.translation().transpose()<<" ";
        os<<q.coeffs()[0]<<" "<<q.coeffs()[1]<<" "<<q.coeffs()[2]<<" "<<q.coeffs()[3]<<endl; // 输出x,y,z,w
        return true;
    }

    virtual void setToOriginImpl(){
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double* update){
        Sophus::SE3 up(  // 也可以用Vector6d定义六位向量up，后面用SE3::exp(upd)更新位姿
            Sophus::SO3(update[3],update[4],update[5]),
            Eigen::Vector3d(update[0],update[1],update[2])
        );
        _estimate = up*_estimate;   // 更新位姿
    }

};

// 两个李代数顶点的边,边就是两个顶点之间的变换,即位姿之间的变换
class EdgeSE3LieAlgebra:public g2o::BaseBinaryEdge<6,SE3,VertexSE3LieAlgebra,VertexSE3LieAlgebra>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool read(istream& is){
        double data[7];
        for(int i = 0;i < 7;i++)
            is >> data[i];
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]); // 注意Eigen::Quaterniond的初始化是w在前
        q.normalize();
        setMeasurement(Sophus::SE3(q,Eigen::Vector3d(data[0],data[1],data[2])));  // 定义观测值

        for(int i = 0; i < information().rows() && is.good(); i++)  // 6*6的信息矩阵，信息矩阵是协方差矩阵的逆
            for(int j = i; j < information().cols() && is.good(); j++){
                is >> information()(i,j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        return true;
    }

    // 输出优化完的相机位姿
    bool write(ostream& os) const{
        VertexSE3LieAlgebra* v1 = static_cast<VertexSE3LieAlgebra*>(_vertices[0]); // _vertices存储顶点信息，存储顺序和调用setVertex(int,vertex) 是设定的int有关(0或1）
        VertexSE3LieAlgebra* v2 = static_cast<VertexSE3LieAlgebra*>(_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";  // 两个顶点的编号流入os
        SE3 m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        // 传入平移和四元数
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for ( int i=0; i<information().rows(); i++ )
            for ( int j=i; j<information().cols(); j++ ){
                os << information() ( i,j ) << "␣";
            }
        os<<endl;
        return true;
    }

    // 误差计算
    virtual void computeError(){
        Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
        Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
        _error = (_measurement.inverse()*v1.inverse()*v2).log(); // 李代数
    }

    virtual void linearizeOplus(){
        Sophus::SE3 v1 = (static_cast<VertexSE3LieAlgebra*>(_vertices[0]))->estimate();
        Sophus::SE3 v2 = (static_cast<VertexSE3LieAlgebra*>(_vertices[1]))->estimate();
        Matrix6d J = JRInv(SE3::exp(_error));
        // 尝试把J近似为I
        // 两个雅克比，一个是误差对相机i位姿的雅克比，一个是误差对相机j位姿的雅克比
        _jacobianOplusXi = -J*v2.inverse().Adj();
        _jacobianOplusXj = J*v2.inverse().Adj();
    } 
};


int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"Usage: pose_graph_g2o_SE3_lie sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin ( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;  // BlockSolver为6x6
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( std::unique_ptr<Block::LinearSolverType>(linearSolver) );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    // 试试G-N或Dogleg？
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( solver_ptr );
    
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm ( solver );  // 设置求解器

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    
    vector<VertexSE3LieAlgebra*> vectices;
    vector<EdgeSE3LieAlgebra*> edges;
    while ( !fin.eof() )
    {
        string name;
        fin>>name;
        if ( name == "VERTEX_SE3:QUAT" )
        {
            // 顶点
            VertexSE3LieAlgebra* v = new VertexSE3LieAlgebra();
            int index = 0;
            fin>>index;
            v->setId( index );
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            vectices.push_back(v);
            if ( index==0 )
                v->setFixed(true);
        }
        else if ( name=="EDGE_SE3:QUAT" )
        {
            // SE3-SE3 边
            EdgeSE3LieAlgebra* e = new EdgeSE3LieAlgebra();
            int idx1, idx2;     // 关联的两个顶点
            fin>>idx1>>idx2;
            e->setId( edgeCnt++ );
            e->setVertex( 0, optimizer.vertices()[idx1] );
            e->setVertex( 1, optimizer.vertices()[idx2] );
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if ( !fin.good() ) break;
    }

    cout<<"read total "<<vertexCnt<<" vertices, "<<edgeCnt<<" edges."<<endl;

    cout<<"prepare optimizing ..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(30);

    cout<<"saving optimization results ..."<<endl;
    // 因为用了自定义顶点且没有向g2o注册，这里保存自己来实现
    // 伪装成 SE3 顶点和边，让 g2o_viewer 可以认出
    ofstream fout("result_lie.g2o");
    for ( VertexSE3LieAlgebra* v:vectices )
    {
        fout<<"VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for ( EdgeSE3LieAlgebra* e:edges )
    {
        fout<<"EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}