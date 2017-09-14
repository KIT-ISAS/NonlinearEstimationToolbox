
/* Compile MEX file with
 *
 *      compileMex('targetSystemEquation.cpp')
 *
 * or with
 *
 *      compileMexOpenMP('targetSystemEquation.cpp')
 *
 * to enable OpenMP parallelization.
 */

#include <Mex/Mex.h>

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        // Check for proper number of arguments
        if (numInputs != 3) {
            throw std::invalid_argument("Three inputs are required.");
        }
        
        if (numOutputs != 1) {
            throw std::invalid_argument("One output is required.");
        }
        
        // First parameter contains the state samples
        Mex::ConstMatrix<double, 5, Eigen::Dynamic> stateSamples(inputs[0]);
        
        // Second parameter contains the noise samples
        Mex::ConstMatrix<double, 2, Eigen::Dynamic> noiseSamples(inputs[1]);
        
        // Check for the same number of state samples and noise samples
        if (stateSamples.cols() != noiseSamples.cols()) {
            throw std::invalid_argument("Different number of samples.");
        }
        
        // Third parameter is the discrete time step
        const double deltaT = mxGetScalar(inputs[2]);
        
        // Allocate memory for predicted state samples
        const int numSamples = stateSamples.cols();
        
        Mex::OutputMatrix<double, 5, Eigen::Dynamic> predictedStates(5, numSamples);
        
#if 1   // Change to #if 0 to switch to the other implementation
        
        // Compute a predicted state sample in each iteration
        #pragma omp parallel for
        for (int i = 0; i < numSamples; ++i) {
            const double px       = stateSamples(0, i);
            const double py       = stateSamples(1, i);
            const double dir      = stateSamples(2, i);
            const double speed    = stateSamples(3, i);
            const double turnRate = stateSamples(4, i);
            
            const double speedNoise    = noiseSamples(0, i);
            const double turnRateNoise = noiseSamples(1, i);
            
            const double predSpeed    = speed + speedNoise;
            const double predTurnRate = turnRate + turnRateNoise;
            const double predDir      = dir + deltaT * predTurnRate;
            
            predictedStates(0, i) = px + std::cos(predDir) * deltaT * predSpeed;
            predictedStates(1, i) = py + std::sin(predDir) * deltaT * predSpeed;
            predictedStates(2, i) = predDir;
            predictedStates(3, i) = predSpeed;
            predictedStates(4, i) = predTurnRate;
        }
        
#else
        
        // Another implementation using Eigen's capabilities could look like this:
        
        // Predicted speed
        predictedStates.row(3) = stateSamples.row(3) + noiseSamples.row(0);
        
        // Predicted turn rate
        predictedStates.row(4) = stateSamples.row(4) + noiseSamples.row(1);
        
        // Predicted direction
        predictedStates.row(2) = stateSamples.row(2) + deltaT * predictedStates.row(4);
        
        // Note that we need the array() cast to perform element-wise cos()/sin() and element-wise multiplication
        
        // Predicted x position
        predictedStates.row(0) = stateSamples.row(0) + deltaT * (predictedStates.row(2).array().cos() *
                                                                 predictedStates.row(3).array()).matrix();
        
        // Predicted y position
        predictedStates.row(1) = stateSamples.row(1) + deltaT * (predictedStates.row(2).array().sin() *
                                                                 predictedStates.row(3).array()).matrix();
        
#endif
        
        // Return the computed predicted state samples back to MATLAB
        outputs[0] = predictedStates;
    } catch (std::exception& ex) {
        mexErrMsgTxt(ex.what());
    }
}
