#include <iostream>
#include "state_space.h"

using namespace std;
using namespace sp;

void SimulateSystem::SimulateSystem() {
    m = 0; n = 0; r = 0;
    A.resize(1,1); A.setZero();
    B.resize(1,1); B.setZero();
    C.resize(1,1); C.setZero();
    x0.resize(1,1); x0.setZero();
    inputSequence.resize(1,1); inputSequence.setZero();
    simulatedStateSequence.resize(1,1); simulatedStateSequence.setZero();
    simulatedOutputSequence.resize(1,1); simulatedOutputSequence.setZero();
    timeRowVector.resize(1,1); timeRowVector.setZero();
}

SimulateSystem::SimulateSystem(MatrixXd Amatrix, MatrixXd Bmatrix, MatrixXd Cmatrix, MatrixXd initialState, MatrixXd inputSequenceMatrix) {
    A = Amatrix; B = Bmatrix; C = Cmatrix; x0 = initialState; inputSequence = inputSequenceMatrix;
    n = A.rows();
    m = B.cols();
    r = C.rows();
    timeSamples = inputSequence.cols();

    simulatedOutputSequence.resize(r,timeSamples); simulatedOutputSequence.setZero();
    simulatedStateSequence.resize(n,timeSamples); simulatedStateSequence.setZero();

    timeRowVector.resize(1,timeSamples);

    for(int i = 0; i < timeSamples; i++) {
        timeRowVector(0,i) = i + 1;
    }
}

void SimulateSystem::runSimultion() {
    for(int j = 0; j<timeSamples; j++) {
        if (j ==0) {
            simulatedStateSequence.col(j) = x0;
            simulatedOutputSequence.col(j) = C*x0;
        }
        else {
            simulatedStateSequence.col(j) = A*simulatedStateSequence.col(j-1) + B*inputSequence.col(j-1);
            simulatedOutputSequence.col(j) = C*simulatedStateSequence.col(j);
        }
    }
}