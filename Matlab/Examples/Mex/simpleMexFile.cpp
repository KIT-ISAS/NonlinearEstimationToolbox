
/* Compile this example MEX file with
 *
 *      compileMex('simpleMexFile.cpp')
 *
 * and execute it with, e.g.,
 *
 *      result = simpleMexFile(3 * ones(3, 10), -2 * ones(3, 1))
 */

#include <Mex/Mex.h>

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        /* Check for proper number of arguments */
        if (numInputs != 2) {
            throw std::invalid_argument("Two inputs are required.");
        }
        
        if (numOutputs != 1) {
            throw std::invalid_argument("One output is required.");
        }
        
        /* Extract first input parameter
         * Here, we require that the first input parameter is a 3xN matrix.
         *
         * The Mex::ConstMatrix class is designed to make the readonly
         * input mxArrays accessible for the Eigen framework. As we now that
         * the data is readonly, compilation will fail if we (accidental)
         * try to modify its content. Moreover, as we only now the number
         * of rows at compile time, the number of columns template parameter
         * is set to accept any possible value.
         */
        Mex::ConstMatrix<double, 3, Eigen::Dynamic> mat(inputs[0]);
        
        /* Extract second input parameter
         * Here, we require the second input parameter to be a 3D vector.
         *
         * As we know at compile time that the second input is a 3D vector
         * we totally avoid Eigen::Dynamic for performance reasons and
         * allow static code analysis for code correctness during compilation.
         */
        Mex::ConstMatrix<double, 3, 1> vec(inputs[1]);
        
        /* Compute the result
         *
         * The Mex::OutputMatrix is designed to return the compuation result
         * of the MEX file. That is, it doesn't free its internal memory to
         * make it accessible to MATLAB after the MEX file execution.
         *
         * Do something useful with the powerful Eigen library.
         * Here, we subtract the second input (the 3D vector) column-wise
         * from the first input (the 3xN matrix), and finally multiply
         * the resulting matrix by 3.
         */
        Mex::OutputMatrix<double, 3, Eigen::Dynamic> output = mat.colwise() - vec;
        
        output *= 3;
        
        /* Return the computed result back to MATLAB
         *
         * The Mex::OutputMatrix is capable of casting an instance
         * to its internal mxArray pointer. Here, we use this functionality
         * to pass our computation result back to MATLAB.
         */
        outputs[0] = output;
    } catch (std::exception& ex) {
        /* In case of any exception, issue a MATLAB exception with
         * the text of the C++ exception. This terminates the MEX
         * file without returning any data back to MATLAB.
         */
        mexErrMsgTxt(ex.what());
    }
}
