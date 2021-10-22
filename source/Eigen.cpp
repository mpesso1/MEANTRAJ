#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

Eigen::Vector3f run(float x, float y, float z) {
    Eigen::Vector3f j;
    j(0) = x;
    j(1) = y;
    j(2) = z;
    return j;
}


int main() {
    bool a = false;
    int const d = 8;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> trying;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> w;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> s;


    Eigen::Matrix<float,1,8> bM;
    Eigen::Vector3f l;
    l(0) = 2;
    l(1) = 4;
    l(2) = 3;

    trying.resize(3,3);
    w = Eigen::Matrix<float,3,3>::Identity();
    s = w*2;
    if (a) {
        std::cout << "did it" << std::endl;
    }
    Eigen::Matrix3f l2 = Eigen::Matrix3f::Ones();
    trying.resize(8,8);
    std::vector<int> arr;
    std::vector<int> b;

    b.push_back(1);
    b.push_back(2);
    b.push_back(5);
    arr.push_back(1);
    arr.push_back(2);
    arr.push_back(5);
    trying(0) = 100;
    trying(1) = 100;
    std::cout << w*s << std::endl;
    
    return 0;
}