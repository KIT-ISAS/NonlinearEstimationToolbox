
/* Compile MEX file with
 *
 *      compileMex('polarMeasLogLikelihood.cpp')
 *
 * or with
 *
 *      compileMexOpenMP('polarMeasLogLikelihood.cpp')
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
        
        // Second parameter contains the inverse noise covariance matrix
        Mex::ConstMatrix<double, 2, 2> invNoiseCov(inputs[1]);
        
        // Thrid parameter contains the actual measurement
        Mex::ConstMatrix<double, 2, 1> measurement(inputs[2]);
        
        // Allocate memory for log-likelihood values
        const int numSamples = stateSamples.cols();
        
        Mex::OutputMatrix<double, 1, Eigen::Dynamic> logValues(1, numSamples);
        
        // Compute a log-likelihood value in each iteration
        #pragma omp parallel for
        for (int i = 0; i < numSamples; ++i) {
            const double px = stateSamples(0, i);
            const double py = stateSamples(1, i);
            
            Eigen::Vector2d vec(std::sqrt(px * px + py * py),
                                std::atan2(py, px));
            
            vec -= measurement;
            
            const double logExpVal = -0.5 * vec.transpose() * invNoiseCov * vec;
            
            // Note that the computed likelihood values are only correct up to proportionality
            // as we omit the normalization part of the Gaussian PDF!
            logValues(i) = logExpVal;
        }
        
        // Return the computed log-likelihood values back to MATLAB
        outputs[0] = logValues;
    } catch (std::exception& ex) {
        mexErrMsgTxt(ex.what());
    }
}
