#ifndef GPMP_H
#define GPMP_H
#include <Eigen/Dense>
#include <vector>
#include <iostream>

namespace root {

    class MeanTraj {
        private:
            std::vector<float> final_times; // final times calculated for each DOF -- indexed based off of the order each DOF is initialized

            Eigen::Matrix<float,1,Eigen::Dynamic> state_time; // time at each step taken 
            //-- specified by max time calculated and the amount of steps specified

            int DsOF; // counts the number of DOF that the system pocesses

            float root_time = 0; // largest time calculated from the inputs of each DOF --> Used to set the size of each DOF mean vector and kernal

            Eigen::Matrix<float,4,Eigen::Dynamic> init_parameters; // matrix that stores all the initial paramiters that define the prior

            int steps; // defines how many steps there are total between initial and final positions -- including the defined starting and final positions
            
            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> mean_state;
            // Stores the mean of the position and the velocity at each given step for each DOF

            float step; // tf / steps --> t0 is always = 0

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> B;

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Q;

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Ka_inv;

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kv;

            Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> Kp;


            float Qc;


        public:
            MeanTraj(int,int,float); 
            // Constructor, allocates space in memory for class

            ~MeanTraj(); 
            // Destructor, removes space in memory that class obtained once code falls out of scope that the class was initialized

            void add_DOF(float,float,float,float,int); 
            // (must clarify the odered number in which this DOF is initialized --> input DOF with desired constant acceleration, 
            // initial velocity, initial position, and final position.  Values get stored in a matrix with all other values. and they are 
            // indexed by the order in which you input the DOF. This is why you must input the oder when initializing function.

            void define_step();
            // calculates the step based off of the max time calculated and the steps desired

            void define_states();
            // fills the state_time vector with each steps time

            void define_B();

            void define_Q();

            void define_Ka_inv();

            void define_kernals();
            void get_something();
            // displays the state_time vector to the consol

    
    };
}
#endif // GPMP_H