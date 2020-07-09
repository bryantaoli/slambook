#include <iostream>
using namespace std;

//Eigen part
#include <Eigen/Core>
#include <Eigen/Dense>

//basic usage of Eigen
int main(int argc, char **argv) {
    // Eigen use matrix as the basic data unit
    // Declare a 2*3 matrix
    Eigen::Matrix<float,2,3> matrix_23;
    // Declare a 3*1 vector
    Eigen::Vector3d v_3d;
    // Declare a 3*3 matrix
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();  // Initialize to setZero
    // Declare a dynamic matrix
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x;
    
    // Operation on matrix
    matrix_23 << 1,2,3,4,5,6;  // Input data
    cout<<"matrix_23 = "<<endl;
    cout<<matrix_23<<endl;     // Output data
    
    // Access matrix elements with ()
    cout<<"The elements of matrix_23"<<endl;
    for(int i = 0; i<1; i++)
      for(int j = 0; j<2; j++)
	cout<<matrix_23(i,j)<<endl;
    
    v_3d << 3,2,1;
    
    // Matrix and vector multiplication
    // Note:Data types must be consistent
    Eigen::Matrix<double,2,1> result = matrix_23.cast<double>()*v_3d;
    cout<<result<<endl;
    
    // Some matrix operation
    matrix_33 = Eigen::Matrix3d::Random();
    cout<<matrix_33<<endl<<endl;
    
    cout<<matrix_33.transpose()<<endl;  // Transpose
    cout<<matrix_33.sum()<<endl;        // Sum of elements
    cout<<matrix_33.trace()<<endl;      // Trace
    cout<<10*matrix_33<<endl;           // Number multiplication
    cout<<matrix_33.inverse()<<endl;    // Inverse of matrix
    cout<<matrix_33.determinant()<<endl;// Determinant
    
    // Calculate eigenvalue of matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33);  // Define an object
    cout<<"Eigenvalue = "<<eigen_solver.eigenvalues()<<endl;
    cout<<"Eigenvectors: "<<eigen_solver.eigenvectors()<<endl;
    
    // Solve equation
    Eigen::Vector3d x = matrix_33.inverse()*v_3d;
    cout<<"x="<<x<<endl;
    // QR decomposition
    x = matrix_33.colPivHouseholderQr().solve(v_3d);
    cout<<"x= "<<x<<endl;
    
    return 0;
}
