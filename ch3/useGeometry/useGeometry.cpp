#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char **argv) {
    // 3D rotation
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // rotation vector:angle, direction
    Eigen::AngleAxisd rotation_vector( M_PI/2, Eigen::Vector3d(0,0,1)); // Rotate 90 degrees around the z-AngleAxisd_rotation_vector
    cout<<"rotaion matrix =\n"<<rotation_vector.matrix()<<endl;  // transform to matrix
    rotation_matrix = rotation_vector.toRotationMatrix();
    
    // coordinate transformation
    Eigen::Vector3d v(1,0,0);   // vector (1,0,0) pointing to x-aixs direction
    Eigen::Vector3d v_rotated = rotation_vector*v;
    cout<<"(1,0,0) after rotation\n"<<v_rotated<<endl;
    // or use rotation matrix
    v_rotated = rotation_matrix*v;
    cout<<"(1,0,0) after ratation\n"<<v_rotated<<endl;
    
    // roatation matrix to Euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0,1,2); //x-y-z
    cout<<"roll pitch yaw = \n"<<euler_angles<<endl;
    
    // Isometry usage:Euclidean transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);  // rotation part assignment:R
    T.pretranslate(Eigen::Vector3d(1,0,0)); // translation part assignment:t
    cout<<"Transform matrix = \n"<<T.matrix()<<endl;
    
    // coordinate transformation using transformation matrix
    Eigen::Vector3d v_transformed = T*v;
    cout<<"v_transformed = \n"<<v_transformed<<endl;
    
    
    // Quaternion 
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector); //Quaterniond means double precision
    cout<<"quaternion = \n"<<q.coeffs()<<endl;    // (x,y,z,w),x y z are real part and w is imaginary part
    // assignment using ratation matrix
    q = Eigen::Quaterniond(rotation_matrix);
    cout<<"quaternion = \n "<<q.coeffs()<<endl;
    // Rotate the vector using quaternion
    v_rotated = q*v;
    cout<<"(1,0,0) after rotation = \n"<<v_rotated<<endl;
    
    
    return 0;
}
