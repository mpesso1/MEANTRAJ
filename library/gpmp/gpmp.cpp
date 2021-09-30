#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gpmp.h"
#include <math.h>

root::MeanTraj::MeanTraj(int DOF, int steps) {
    DsOF = DOF;
    init_parameters.resize(4,DsOF);
    step = step;
}

root::MeanTraj::~MeanTraj(){ 
    std::cout << "DOF ENDED" << std::endl;
} // Define destructor in cpp

void root::MeanTraj::add_DOF(float a, float v0, float p0, float pf, int idx) {
    init_parameters(0,idx) = a;
    init_parameters(1,idx) = v0;
    init_parameters(2,idx) = p0;
    init_parameters(3,idx) = pf;

    if (a == 0) {
        final_times[idx] = (pf-p0)/v0;
    }
    else {
        float tplus = (-v0 + sqrt(pow(v0,2)-4*(a/2)*(p0-pf)))/a;
        float tminus = (-v0 - sqrt(pow(v0,2)-4*(a/2)*(p0-pf)))/a;
        if (tplus > tminus) {
            final_times.push_back(tplus);
        }
        else {
            final_times.push_back(tminus);
        }
    }
    if (final_times[idx] > root_time) {
        root_time = final_times[idx];
    }
}
