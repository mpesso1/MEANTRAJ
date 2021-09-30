#include "gpmp.h"
#include <iostream>

using namespace std;
using namespace root;

int main() {
    MeanTraj BlueRov(6);
    BlueRov.add_DOF(.1,2.,0.,6.,0);
    
    return 0;
}