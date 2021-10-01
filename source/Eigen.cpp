#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

int main() {
    int const d = 8;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> trying;
    Eigen::Matrix<float,1,8> bM;
    Eigen::Matrix3f l = Eigen::Matrix3f::Ones();
    l(0,0) = 2;
    l(0,1) = 4;
    l(1,2) = 3;

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
    std::cout << arr << std::endl;
    
    return 0;
}