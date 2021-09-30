
Description of GPMP code:

Goal:
    Produce a trajectory for an (n) DOF robot.

Procedure:
    Step 1.) Build initial path that can be easily be manipulated and interpolated -- the prior
    Step 2.) Modify the prior trajectory by treating it as an optimization problem -- produces update rule


(Step 1)
    1a.) Use the gaussian process to establish a method for generating a trajectory based on a few states of interest
    1b.) Seperate each DOF, and additionally seperate each varibale defining the DOF -- (position, velocity, and acceleration)
    1c.) The way we will solve the Gaussian Process model is by using the equations of motion (eom) to build off one of the computed mean trajectories and kernals

        ... Starting with the acceleration is a good choice because it must be a constant for the eom. The gauss-markov derivation will be used to define the acceleration kernal

        ... after the aceleration kernal is defined, and because the derivation was markovian it can be shown that the velocity and position kernal will just be an easily computed integration w.r.t time for the acceleration kernal

        ... the mean velocity and mean position trajectories will be defined by the eom which will require an initial velocity, initial position, acceleration trajectory, and time span.

        ... However, instead of defining the time span and recieving the final state, we would like the reverse.  So in order to do that we will define the final state and solve for the time span with Newton Rasphson
        
        ... Drawback: We need a set of states for the gaussian process to run off of that will conicide with each seperate DOF.  

        ... The amount of states only sets the size of the kernals and not the actual values in them.  There for we are concered with the size of the kernal all being the same and the amount of mean states

        ... to solve this we will need to have a method within the algorithm that can depict the largest time frame produced and set this as the timeframe for all the DOF, but at the same time keeping the mean of that DOF constant once the derived time frame has been surpassed

        ... So in order to produce the mean trajectories we will need: ** Each DOF has the ability to set its own internal variables as long as the acceleration is a constant.  Because the Kernals will all be identical.  This is the beauty.
** Must contain a method that allows the user to input each DOFs input.  This is so the user can help the code by indexing the DOF for the time array as he/she initializes everything
                Input:
                    - Desired constant acceleration
                    - Initial velocity
                    - Initial position
                    - Final position
                    - Step
                Solution:
                    - recieve all the time spans produced and save them with their specific mean trajectories DOF
                    - locate max time span
                    - Calculate each DOFs trajectory at each step defined
                        > Check if the time span specific to the DOF was passed
                        > if so, set the DOF at that step and every following step equal to the desired value -- ignore the fact that the acceleration would go to zero
                        { so a DOF is defined by its initial state, its final position, and its time span}
                Output:
                    - A trajectory prior for each DOFs acceleration, velocity, and position
                
        ... We can now construct a single kernal for acceleration, velocity, and position for each DOF. Which mean only one kernal will contain an inverse and and only two will be mutiplied by constants

        ... Producing the kernals:
                Input:
                    - Hyperparameter Qc
                    - The time step size
                    - Max time_span
                    - Desired acceleration for the system
                Solution:
                    - Matrix B --> Defined by Gauss Markov derivation which is a matrix composed of 1s across the diagonal and 1s across the the lower band diagonal considering that our acceleration is constant. 
                                    This matrix is also concatinated with an n deminsional row vector making the matrix unspare but also defining that the final state position is know
                    - Matrix Q --> a matrix construced of Qi along its diagonal where Qi is defined by a hyperparameter Qc integrated across the size of each time step == Qc*delta(t)
                Output:
                    - Matrix Ka (acceleration kernal) --> (B.transpose()*Q.inverse()*B).inverse() derived from the Gauss Markov derivation
                    - Matrix Kv (velocity kernal) --> Ka*delta(t)/2
                    - Matrix Kp (position kernal) --> Ka*delta(t)^2/4

                



