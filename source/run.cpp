#include "gpmp.h"
#include <iostream>
//access token == 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

using namespace std;
using namespace root;

int main() {
    MeanTraj BlueRov(6,20,3); // # of DOF , steps, # of ocv
    BlueRov.set_sensitivity(1.0,1.0,1.0); // whole, ability, power
    BlueRov.add_DOF(.1,2.,0.,4.,0,true); // acceleration, init velocity, init poition, final position, indx, boolian defining ocv
    BlueRov.add_DOF(.1,2.,0.,7.,1,true);
    BlueRov.add_DOF(.1,2.,0.,12.,2,true);
    BlueRov.add_DOF(.1,2.,0.,4.,3,false);
    BlueRov.add_DOF(.1,2.,0.,4.,4,false);
    BlueRov.add_DOF(.1,2.,0.,5.,5,false);
    BlueRov.get_something();
    
    
    return 0;
}