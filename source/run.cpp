#include "gpmp.h"
#include <iostream>
//access token == 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

using namespace std;
using namespace root;

int main() {
    MeanTraj BlueRov(6,20,3.0); // # of DOF , steps, Qc
    BlueRov.add_DOF(.1,2.,0.,4.,0); // acceleration, init velocity, init poition, final position, indx
    BlueRov.add_DOF(.1,2.,0.,7.,1);
    BlueRov.add_DOF(.1,2.,0.,12.,2);
    BlueRov.add_DOF(.1,2.,0.,4.,3);
    BlueRov.add_DOF(.1,2.,0.,4.,4);
    BlueRov.add_DOF(.1,2.,0.,5.,5);
    BlueRov.get_something();
    
    
    return 0;
}