#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gpmp.h"
#include <math.h>

root::MeanTraj::MeanTraj(int DOF, int num_of_steps, int ocv) {

    // -- Prior Variables

    DsOF = DOF;
    init_parameters.resize(4,DOF);
    steps = num_of_steps;
    mean_state.resize(DOF*2,steps+1); // all state defining varibales and time variables must include a +1 in demension defining ste size
    // the x2 is so that the mean position and velocity can be stored within the same matrix
    state_time.resize(1,num_of_steps+1);
    B.resize(steps+2,steps+1); // The is supposed to be an unsquare marix!
    Q.resize(steps+2,steps+2);
    Ka_inv.resize(steps+2,steps+2);
    Kv_inv.resize(steps+2,steps+2);
    Kv.resize(steps+2,steps+2);
    Kp_inv.resize(steps+2,steps+2);
    Kp.resize(steps+2,steps+2);
    define_B();

    // -- Optimization Variabels

    ocv_size = ocv;
    state.resize(DOF*2,steps+1); // may not need this variable as we used a different one for same pupose, dont forget to delete in other places
    mag_ocv.resize(1,steps+1);
    J.resize(steps+1,ocv_size);
    unit_prime.resize(ocv_size,steps+1);
    unit_dubprime.resize(ocv_size,steps+1);
    del_c.resize(ocv_size,1);

    final_pos.resize(steps+1,ocv_size); // have to break these into two sepperate matrices so algebraic manipulation can be done on them speratly
    mean_pos.resize(steps+1,ocv_size);
    final_vel.resize(steps+1,ocv_size);
    mean_vel.resize(steps+1,ocv_size);

    I = Eigen::Matrix<float,3,3>::Identity();
    IJ = I;

    g_pos.resize(ocv,steps+1);
    g_vel.resize(ocv,steps+1);

    obs.resize(ocv,ocv);
}

// ************ within code I will point out the demensions of some variables --> (steps+1) = (s1) ; (steps+1,steps+1) = (s1,s1) ; (steps) = (s) ***********

// Prior ----

root::MeanTraj::~MeanTraj(){ 
    std::cout << "DOF ENDED" << std::endl;
} // Define destructor in cpp

void root::MeanTraj::add_DOF(float a, float v0, float p0, float pf, int idx, bool ocv) {
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

    if (ocv) {
        add_ocv(idx);
    }

    if (idx+1 == DsOF) {
        if (ocv_idx.size() != ocv_size) {
            std::cout << "Object concerning varibales (ocv) does not equal the amount of ocv variables initiated" << std::endl;
            std::cout << "make sure you declare boolean true for each DOF that is a ocv and make sure it equals the ocv_size you initiate in the last paramater of class instance" << std::endl;
        }
        define_step(); // size of step
        define_states(); // updates prior states -- adjusts for timeing inconsistencies -- stores in "mean_state ()
        define_Q(); // Matrix used to solve for prior kernals -- cannot be defined in the constructor because it requires sensitivity variables -- however if Q wouldnt change it would be nice to define it only once.
        define_Ka_inv(); // basis kernal used to solve for position and velocity kernal
        define_kernals(); // solve for position and veloity kernal

        declare_optimization(); // create variables that need to be ready for optimization
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
                mean_state(2*j+1,i) = init_parameters(0,j)*root_time + init_parameters(1,j); // need to make a decision if we should set acceleration to 0 once this occurs, and how that would affect the results in optimization
            }
            else {
                mean_state(2*j,i) = init_parameters(0,j)*add*add/2 + init_parameters(1,j)*add + init_parameters(2,j);
                mean_state(2*j+1,i) = init_parameters(0,j)*add + init_parameters(1,j);
            }
        }
        add+=step;
        i+=1;
    }
    state = mean_state;
}

void root::MeanTraj::define_B() {
    for (int i = 0; i<B.cols();i++) {
        B(i,i) = 1;
        B(i+1,i) = -1;
    }
}

void root::MeanTraj::define_Q() {
    for (int i=0; i<Q.cols(); i++) {
        Q(i,i) = sensitivity[2]*step;
    }
}

void root::MeanTraj::define_Ka_inv() {
    Ka_inv = B.transpose()*Q.inverse()*B;
}

void root::MeanTraj::define_kernals() {
    Kv_inv = Ka_inv*(step*step/2);
    Kv = Ka_inv.inverse()*(step*step/2);
    Kp_inv = Kv_inv*(step*step/2);
    Kp = Kv*(step*step/2);
}


// Optimization ----

void root::MeanTraj::set_sensitivity(float whole, float ability, float power) {
    sensitivity.push_back(whole);
    sensitivity.push_back(ability);
    sensitivity.push_back(power); // for prior
}

void root::MeanTraj::add_ocv(int index) {
    ocv_idx.push_back(index);
}


void root::MeanTraj::declare_optimization() {
    for (int i=0;i<=steps;i++) {
        mag_ocv(0,i) = sqrt(pow(mean_state(ocv_idx[0]*2+1,i),2)+pow(mean_state(ocv_idx[1]*2+1,i),2)+pow(mean_state(ocv_idx[2]*2+1,i),2)); // (1,s1)
        for (int j=0;j<ocv_size;j++) {
            J(i,j) = mean_state(ocv_idx[j]*2+1,i); // (s1,ocv)
            unit_prime(j,i) = mean_state(ocv_idx[j]*2+1,i)/mag_ocv(i); // (ocv,s1)
            unit_dubprime(j,i) = init_parameters(0,ocv_idx[j]); // (ocv,s1)
            final_pos(i,j) = mean_state(ocv_idx[j]*2,i); // (ocv,s1)
            mean_pos(i,j) = final_pos(i,j); // (ocv,s1)
            final_vel(i,j) = mean_state(ocv_idx[j]*2+1,i); // (ocv,s1)
            mean_vel(i,j) = final_vel(i,j); // (ocv,s1)
        }
    }
}

void root::MeanTraj::update_optimizaion() {
    for (int i=0;i<=steps;i++) {
        mag_ocv(i) = sqrt(pow(final_vel(i,0),2)+pow(final_vel(i,1),2)+pow(final_vel(i,2),2)); // (1,s1)
        mean_pos.row(i) = final_pos.row(i);
        mean_vel.row(i) = final_vel.row(i);
        J.row(i) = final_vel.row(i);
        unit_prime(0,i) = final_vel(i,0)/mag_ocv(i);
        unit_prime(1,i) = final_vel(i,1)/mag_ocv(i);
        unit_prime(2,i) = final_vel(i,2)/mag_ocv(i);
    }
}

void root::MeanTraj::optimize(float obj_x,float obj_y,float obj_z) {
    obs(0) = obj_x;
    obs(1) = obj_y;
    obs(2) = obj_z;

    update_g();
    for (int i=0;i<100;i++) {
        final_pos = final_pos - (1/sensitivity[1])*Kp*(sensitivity[0]*Kp_inv*(final_pos-mean_pos) + g_pos.transpose());
        final_vel = final_vel - (1/sensitivity[1])*Kv*(sensitivity[0]*Kv_inv*(final_vel-mean_vel) + g_vel.transpose());
        update_optimizaion();
    }
}

void root::MeanTraj::update_g() { // the way you initialize your ocv must be in the same order that you input your object detection cordinates ********
    for (int i=0;i<=steps;i++) {
        update_J(i);
        for (int j=0;j<ocv_size;j++) {
            del_c(j) = obs(j) - final_pos(i,j); // vector computed distance between object and robot state(i).  Both vectors are taken from the robots initial position
            if (abs(del_c(j)) < .5) {
                del_c(j) = 1;
            }
            else {
                del_c(j) = 0;
            }
        }
        std::cout << del_c << std::endl;
        c = sqrt(pow(del_c(0),2) + pow(del_c(1),2) + pow(del_c(2),2));  /// <----- ** hard coded indexes, ultimitly you wouldnt know how many indexes**************
        g_pos.col(i) = IJ.transpose()*mag_ocv(i)*(I - unit_prime.col(i)*unit_prime.col(i).transpose())*del_c - c*pow(mag_ocv(i),-2)*(I - unit_prime.col(i)*unit_prime.col(i).transpose())*unit_dubprime.col(i);
        g_vel.col(i) = IJ.transpose()*c*unit_prime.col(i);
    }
}

void root::MeanTraj::update_J(int step) {
    for (int j=0;j<ocv_size;j++) {
        IJ(j,j) = J(step,j); // (ocv,ocv)
    }
}


// Display Stuff ----
void root::MeanTraj::get_something() {

    //std::cout << Kp_inv << std::endl;
    //std::cout << Kp_inv.rows() << "," << Kp_inv.cols() << std::endl;
    //td::cout << "  " << std::endl;

    //std::cout << Kp << std::endl;
    //std::cout << Kp.rows() << "," << Kp_inv.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << Kv << std::endl;
    //std::cout << Kv.rows() << "," << Kp_inv.cols() << std::endl;    
    //std::cout << "  " << std::endl;

    //std::cout << Kv_inv << std::endl;
    //std::cout << Kv_inv.rows() << "," << Kp_inv.cols() << std::endl;

    //std::cout << "  " << std::endl;

    std::cout << final_pos.transpose() << std::endl;
    std::cout << final_pos.rows() << "," << final_pos.cols() << std::endl;
    std::cout << "  " << std::endl;

    //std::cout << del_c << std::endl;

    //std::cout << mean_pos << std::endl;
    //std::cout << mean_pos.rows() << "," << mean_pos.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << g_pos << std::endl;
    //std::cout << g_pos.rows() << "," << g_pos.cols() << std::endl;
    //std::cout << "  " << std::endl;

    //std::cout << mean_state << std::endl;
    //std::cout << mean_state.rows() << "," << mean_state.cols() << std::endl;
    //std::cout << "  " << std::endl;


    //std::cout << Q.rows() << "  " << Q.cols() << std::endl;
}

