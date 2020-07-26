#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
using namespace std;

// 曲线模型的顶点,模板参数:优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>  // 顶点类型 a b c三维变量
{
public:
    // 类成员有Eigen变量，定义一个宏，每次new一个对象返回一个对齐的指针
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
    virtual void setToOriginImpl()  // 顶点重置
    {
      _estimate<<0,0,0;   // 初始化定点  优化变量值初始化
    }
    virtual void oplusImpl(const double* update) // 顶点更新
    {
      _estimate += Eigen::Vector3d(update);  // 更新X(k+1)=X(k)+x;
    }
    // 存盘和读盘：留空
    virtual bool read(istream& in){}
    virtual bool write(ostream& out) const{}
};

// 误差模型模板参数:观测值维度,类型,连接顶点类型
// 一元边 BaseUnaryEdge<1,double,CurveFittingVertex> 
// 二元边 BaseBinaryEdge<2,double,CurveFittingVertex>
// 多元边 BaseMultiEdge<>
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {} //初始化函数   直接赋值 _x = x
    // 边的误差计算函数
    void computeError()
    {
      const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
      const Eigen::Vector3d abc = v->estimate();
      _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x; // x 值,y 值为_measurement
};


int main(int argc, char **argv) {
    double a=1.0, b=2.0, c=1.0; // 真实参数值
    int N=100; // 数据点
    double w_sigma=1.0; // 噪声值Sigma
    cv::RNG rng; // 随机数产生器OpenCV
    double abc[3] = {0,0,0}; // 参数的估计值abc
    
    vector<double> x_data, y_data; // 数据
    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
    double x = i/100.0;
    x_data.push_back ( x );
    y_data.push_back (
    exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
    );
    cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }
    
    // 构建图优化,先设定g2o
    // 矩阵块:每个误差项优化变量维度为3 ,误差值维度为1
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block; // 每个误差项优化变量维度为3，误差值维度为1
    // 线性方程求解器:稠密的增量方程H * Δx = −b
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();// 线性方程求解器
    // Block* solver_ptr = new Block( linearSolver ); // 矩阵块求解器,用于求解 雅克比J ( 得到右边 b = e' *  Ω * J ) 和  海塞矩阵 H  左边 H = J'* Ω * J
    // 梯度下降方法,从GN, LM, DogLeg 中选
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    
    // 注意新版的g2o库中使用了智能指针，需要进行指针类型转换
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm( solver ); // 设置求解器
    optimizer.setVerbose( true ); // 打开调试输出
    
    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex(); // //曲线拟合 新建 顶点类型
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
      CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);  // 新建边 带入观测数据
      edge->setId(i);
      edge->setVertex( 0, v ); // 设置连接的顶点
      edge->setMeasurement( y_data[i] ); // 观测数值
      // 信息矩阵:协方差矩阵之逆
      edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) );
      optimizer.addEdge( edge ); // 添加边
    }
    
    // 执行优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);  // 优化次数
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds "<<endl;
    
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout<<"estimated␣model:␣"<<abc_estimate.transpose()<<endl;
    
    return 0;
}
