
/* Compile MEX file with
 *
 *      compileMex('polarMeasurementEquation.cpp')
 *
 * or with
 *
 *      compileMexOpenMP('polarMeasurementEquation.cpp')
 *
 * to enable OpenMP parallelization.
 */

#include <Mex/Mex.h>

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        // Check for proper number of arguments
        if (numInputs != 1) {
            throw std::invalid_argument("One input is required.");
        }
        
        if (numOutputs != 1) {
            throw std::invalid_argument("One output is required.");
        }
        
        // First parameter contains the state samples
        Mex::ConstMatrix<double, 5, Eigen::Dynamic> stateSamples(inputs[0]);
        
        // Allocate memory for measurement samples
        const int numSamples = stateSamples.cols();
        
        Mex::OutputMatrix<double, 2, Eigen::Dynamic> measurements(2, numSamples);
        
        // Compute a measurement sample in each iteration
        #pragma omp parallel for
        for (int i = 0; i < numSamples; ++i) {
            const double px = stateSamples(0, i);
            const double py = stateSamples(1, i);
            
            measurements.col(i) = Eigen::Vector2d(std::sqrt(px * px + py * py),
                                                  std::atan2(py, px));
        }
        
        // Return the computed measurement samples back to MATLAB
        outputs[0] = measurements;
    } catch (std::exception& ex) {
        mexErrMsgTxt(ex.what());
    }
}
