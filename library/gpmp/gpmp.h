#ifndef GPMP_H
#define GPMP_H
#include <Eigen/Dense>
#include <vector>

namespace root {

    class MeanTraj {
        private:
            std::vector<float> final_times; // indexed based off of the order each DOF is initialized
            int DsOF; // counts the number of DOF that the system pocesses
            float root_time = 0; // largest time calculated from the inputs of each DOF --> Used to set the size of each DOF mean vector and kernal
            Eigen::Matrix<float,4,Eigen::Dynamic> init_parameters; // matrix that stores all the initial paramiters that define the prior
            int step;
        public:
            MeanTraj(int,int); // Constructor, allocates space in memory for class

            ~MeanTraj(); // Destructor, removes space in memory that class obtained once code falls out of scope that the class was initialized

            void add_DOF(float,float,float,float,int); // (must clarify the odered number in which this DOF is initialized --> input DOF with desired constant acceleration, initial velocity, initial position, and final position.  Values get stored in a matrix with all other values.
    };
}
#endif // GPMP_H