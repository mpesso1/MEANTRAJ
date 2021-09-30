#include <iostream>
#include <Eigen/Dense>
#include <vector>


int main() {
    int const d = 8;
    Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> trying;
    trying.resize(8,8);
    std::vector<int> arr;
    arr.push_back(1);
    arr.push_back(2);
    arr.push_back(5);
    trying(0) = 100;
    trying(1) = 100;
    std::cout << trying << std::endl;
    std::cout << arr.size() << std::endl;
    
    return 0;
}