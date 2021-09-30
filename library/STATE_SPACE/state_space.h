#inlcude <iostream>
#include <Eigen/Dense>

#ifndef STATE_SPACE_H
#define STATE_SPACE_H

namespace sp{

class SimulateSystem {
public:
    SimulateSystem();
    // default constructor 
    // sets all of the variables to a 1x1 dimensional matrix and sets all variables to 0.


    SimulateSystem(MatrixXd Amatrix, MatrixXd Bmatrix, MatrixXd Cmatrix, MatrixXd initialState, MatrixXd inputSequenceMatrix);
    // overloaded constructor

    ~SimulateSystem();
    // default destructor

    std::tuple<MatrixXd,MatrixXd,MatrixXd> getStateOutputTime();

    void runSimultion();
    // runs discrete state space model


private:
    MatrixXd A,B,C; // A,B,C matrices
    MatrixXd x0; // initial state
    MatrixXd inputSequence; // input sequence, demension
    MatrixXd simulatedStateSequence; // simulated state sequence, demensions
    MatrixXd simulatedOutputSequence; // simulated output sequence
    MatrixXd timeRowVector; //time row vector
    int m,n,r,timeSamples; // m - input demension, n - state demension, r - output demension, timesamples - number of time samples
};

}

#endif // STATE_SPACE_H