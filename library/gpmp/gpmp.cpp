#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gpmp.h"
#include <math.h>

root::MeanTraj::MeanTraj(int DOF, int num_of_steps, float Q_c) {
    DsOF = DOF;
    init_parameters.resize(4,DsOF);
    steps = num_of_steps;
    mean_state.resize(DOF*2,steps+1);
    state_time.resize(1,num_of_steps+1);
    Qc = Q_c;
    B.resize(steps,steps-1);
    Q.resize(steps,steps);
    Ka_inv.resize(steps,steps);
    Kv.resize(steps,steps);
    Kp.resize(steps,steps);
    define_B();
}

root::MeanTraj::~MeanTraj(){ 
    std::cout << "DOF ENDED" << std::endl;
} // Define destructor in cpp

void root::MeanTraj::add_DOF(float a, float v0, float p0, float pf, int idx) {
    init_parameters(0,idx) = a;
    init_parameters(1,idx) = v0;
    init_parameters(2,idx) = p0;
    init_parameters(3,idx) = pf;

    mean_state(idx,0) = p0;
    mean_state(idx+1,0) = v0;

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

    if (idx+1 == DsOF) {
        define_step();
        // calculate the step with the max time defined and the steps specified by user
        define_states();
        define_Q();
        define_Ka_inv();
        define_kernals();
    }

    if (idx+1 > DsOF) {
        std::cout << "TOO MANY DOF SPECIFIED" << std::endl;
    }
}

void root::MeanTraj::define_step() {
    step = root_time/steps;
}

void root::MeanTraj::define_states() {
    float add = 0;
    int i = 0;
    while(add <= root_time+.01) { // had errors occure were the last column defining the mean was coming out as 0.  This was due to the add being greater than the root_time before it was supposed to.  This error is beleived to occure because of a truncation/rounding error by computer.
        state_time(0,i) = add;
        for (int j = 0; j<=DsOF-1; j++) {
            if (add>=final_times[j]) {
                mean_state(2*j,i) = init_parameters(3,j);
                mean_state(2*j+1,i) = init_parameters(0,j)*root_time + init_parameters(1,j);
            }
            else {
                mean_state(2*j,i) = init_parameters(0,j)*add*add/2 + init_parameters(1,j)*add + init_parameters(2,j);
                mean_state(2*j+1,i) = init_parameters(0,j)*add + init_parameters(1,j);
            }
        }
        add+=step;
        i+=1;
    }
}

void root::MeanTraj::define_B() {
    for (int i = 0; i<B.cols();i++) {
        B(i,i) = 1;
        B(i+1,i) = -1;
    }
}

void root::MeanTraj::define_Q() {
    for (int i=0; i<Q.cols(); i++) {
        Q(i,i) = Qc*step;
    }
}

void root::MeanTraj::define_Ka_inv() {
    Ka_inv = B.transpose()*Q.inverse()*B;
}

void root::MeanTraj::define_kernals() {
    Kv = Ka_inv.inverse()*(step*step/2);
    Kp = Kv*(step*step/2);
}

void root::MeanTraj::get_something() {
    std::cout << step << std::endl;
    std::cout << Kv << std::endl;
    std::cout << Ka_inv.rows() << "  " << Ka_inv.cols() << std::endl;
    std::cout << Q.rows() << "  " << Q.cols() << std::endl;
}
