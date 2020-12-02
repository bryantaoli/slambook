#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
using namespace std;

/********************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工合成的一个Pose graph
 * 尽管可以直接通过load 函数读取整个图,但我们还是自己来实现读取代码,以期获得更深刻的理解
 * 这里使用g2o/types/slam3d/ 中的SE3 表示位姿,它实质上是四元数而非李代数
 * *******************************************/


int main(int argc, char** argv)
{
    if(argc != 2){
        cout<<"Usage: pose_graph_g2o_SE3 sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if(!fin){
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }

    // 开始g2o的配置求解
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block; // 边两边顶点都是SE36维
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器

    int vertexCnt = 0,edgeCnt = 0; // 顶点和边的个数
    while(!fin.eof()){
        string name;
        fin>>name;
        if(name=="VERTEX_SE3:QUAT"){
            // SE3顶点
            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin>>index;
            v->setId(index); 
            v->read(fin); // 读入位姿
            optimizer.addVertex(v);
            vertexCnt++;
            if(index==0)
                v->setFixed(true); // 第一个顶点不用优化,所有顶点插入vset中然后遍历
        }
        else if(name=="EDGE_SE3:QUAT"){
            // SE3-SE3边
            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1,idx2;  // 关联的两个顶点
            fin>>idx1>>idx2;  // 读入
            e->setId(edgeCnt++);
            e->setVertex(0,optimizer.vertices()[idx1]);
            e->setVertex(1,optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if(!fin.good())
            break;
    }

    cout<<"read total"<<vertexCnt<<" vertices, "<<edgeCnt<<"edges."<<endl;

    cout<<"prepare optimizing"<<endl;
    optimizer.setVerbose(true);  // 打开调试输出
    optimizer.initializeOptimization(); // 将所有顶点插入到vset中，遍历vset组合，取出每个顶点的边，判断连接的顶点是否有效，形成有效点与有效边
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(30);

    cout<<"saving optimization results"<<endl;
    optimizer.save("result.g2o");

    return 0;

}