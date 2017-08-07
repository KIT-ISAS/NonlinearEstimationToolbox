
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <GLCD/Computation.h>
#include <GLCD/Misc.h>
#include <Mex/Mex.h>
#include <stdexcept>
#include <sstream>

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        /* At least dimension and number of samples are required. */
        if (numInputs < 2) {
            std::stringstream ss;
            
            ss << "Compute optimally placed and equally weighted multivariate standard normal distributed samples\n\n"
               
               << "  Uses the Localized Cumulative Distribution (LCD) in combination with a\n"
               << "  modified Cramer-von Mises (mCvM) distance as optimality measure.\n\n"
               
               << "  This is free software; see the source for copying conditions. There is NO\n"
               << "  warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n\n"
               
               << "  Copyright (C) 2017  Jannik Steinbring <jannik.steinbring@kit.edu>\n"
               << "                      Martin Pander <martin.pander@kit.edu>\n\n"
               
               << "  Institute for Anthropomatics and Robotics\n"
               << "  Chair for Intelligent Sensor-Actuator-Systems (ISAS)\n"
               << "  Karlsruhe Institute of Technology (KIT), Germany\n"
               << "  Prof. Dr.-Ing. Uwe D. Hanebeck\n\n"
               
               << "  http://isas.uka.de\n\n"
               
               << "  Input parameters:\n"
               << "    * Dimension (required)\n"
               << "        The dimension of the considered standard normal distribution.\n\n"
               
               << "    * Number of samples (required)\n"
               << "        The desired number of samples.\n\n"
               
               << "    * Symmetric sampling scheme flag (optional)\n"
               << "        If true, the symmetric sampling scheme is used for determining the\n"
               << "        sample positions. Otherwise, the asymmetric sampling scheme is used.\n"
               << "        The symmetric sampling scheme is to be preferred as it ensures correct\n"
               << "        odd moments and is much faster to compute.\n\n"
               
               << "        Default: true\n\n"
               
               << "    * bMax (optional)\n"
               << "        Provide the upper bound for the numerical integration required to\n"
               << "        compute the mCvM distance and its derivatives. The larger the specified\n"
               << "        dimension the larger bMax has to be to generate meaningful sample positions.\n\n"
               
               << "        Default: Dimension ==    1:   5\n"
               << "                 Dimension <=   10:  10\n"
               << "                 Dimension <=  100:  50\n"
               << "                 Dimension <= 1000: 100\n"
               << "                 Dimension >  1000: 200\n\n"
               
               << "    * Initial sample positions (optional)\n"
               << "        Provide column-wise arranged initial sample positions for the\n"
               << "        optimization procedure. The provided sample positions have to\n"
               << "        fit in with the specified dimension, number of samples, and\n"
               << "        slected asymmetric / symmetric sampling scheme. For example,\n"
               << "        when computing 13 2D samples with the asymmetric sampling scheme\n"
               << "        a 5x13 matrix is required. If instead the symmetric sampling\n"
               << "        scheme is selected, a 5x6 matrix is required.\n\n"
               
               << "        Default: Random standard normal distributed sample positions of\n"
               << "                 approriate dimension and amount.\n\n"
               
               << "  Output parameters:\n"
               << "    * Optimized sample positions\n"
               << "        Column-wise arranged optimized sample positions.\n\n"
               
               << "    * Sample weight\n"
               << "        The corresponding sample weight for all samples.\n\n"
               
               << "    * Used initial sample positions\n"
               << "        Column-wise arranged initial sample positions used by the optimization procedure.\n"
               << "        If initial sample positions were provided by the user, these will be returned.\n\n"
               
               << "    * Modified Cramer-von Mises distance\n"
               << "        The mCvM distnace corresponding to the computed optimal sample positions.\n\n"
               
               << "    * Normalized sample covariance error\n"
               << "        A normalized error of sample covariance matrix\n\n"
               
               << "              ||Cov - I|| / D^2  ,\n\n"
               
               << "        where ||.|| denotes the Frobenius norm, Cov the sample covariance matrix,\n"
               << "        D the specified dimension, and I the DxD identity matrix.\n\n"
               
               << "  Examples:\n"
               << "    * Generate 51 two-dimensional standard normal distributed samples using the\n"
               << "      symmetric sampling scheme:\n\n"
               
               << "        samples = GLCD(2, 51);\n"
               << "        hold on;\n"
               << "        axis equal;\n"
               << "        plot(samples(1, :), samples(2, :), 'bo', 'LineWidth', 3, 'MarkerSize', 3);\n\n"
               
               << "    * Generate 25 two-dimensional standard normal distributed samples using the\n"
               << "      asymmetric sampling scheme:\n\n"
               
               << "        samples = GLCD(2, 25, false);\n"
               << "        hold on;\n"
               << "        axis equal;\n"
               << "        plot(samples(1, :), samples(2, :), 'bo', 'LineWidth', 3, 'MarkerSize', 3);\n\n";
            
            mexPrintf("%s", ss.str().c_str());
            
            return;
        }
        
        /* Parse inputs */
        GLCD::Computation computation;
        Eigen::MatrixXd initialParameters;
        int dimension;
        int numSamples;
        
        /* Dimension */
        if (mxGetNumberOfElements(inputs[0]) != 1) {
            throw std::invalid_argument("Dimension must be a scalar.");
        }
        
        dimension = (int) *mxGetPr(inputs[0]);
        
        if (dimension <= 0) {
            throw std::invalid_argument("Dimension must be greater than zero.");
        }
        
        /* Number of samples */
        if (mxGetNumberOfElements(inputs[1]) != 1) {
            throw std::invalid_argument("Number of samples must be a scalar.");
        }
        
        numSamples = (int) *mxGetPr(inputs[1]);
        
        /* Symmetric sampling scheme flag */
        if (numInputs >= 3) {
            if (!mxIsLogicalScalar(inputs[2])) {
                throw std::invalid_argument("Symmetric sampling scheme flag must be a boolean scalar.");
            }
            
            const bool symmetric = *mxGetLogicals(inputs[2]);
            
            computation.setSymmetric(symmetric);
        }
        
        /* bMax */
        if (numInputs >= 4) {
            if (mxGetNumberOfElements(inputs[3]) != 1) {
                throw std::invalid_argument("bMax must be a scalar.");
            }
            
            const double bMax = *mxGetPr(inputs[3]);
            
            computation.setBMax(bMax);
        }
        
        /* Initial sample positions */
        if (numInputs >= 5) {
            try {
                initialParameters = Mex::ConstMatrixXd(inputs[4]);
            } catch (std::exception& ex) {
                throw std::invalid_argument(std::string("Invalid initial sample positions: ") + ex.what());
            }
        }
        
        /* Perform sample computation */
        Eigen::MatrixXd samples;
        double distCorrectedSamples;
        
        computation(dimension, numSamples,
                    initialParameters, samples,
                    &distCorrectedSamples);
        
        if (numOutputs >= 1) {
            /* Return computed samples back to MATLAB */
            outputs[0] = Mex::OutputMatrixXd(samples);
        }
        
        if (numOutputs >= 2) {
            /* Return sample weight back to MATLAB */
            const double sampleWeight = 1.0 / numSamples;
            
            outputs[1] = mxCreateDoubleScalar(sampleWeight);
        }
        
        if (numOutputs >= 3) {
            /* Return initial samples back to MATLAB */
            outputs[2] = Mex::OutputMatrixXd(initialParameters);
        }
        
        if (numOutputs >= 4) {
            /* Return mCvM distance back to MATLAB */
            outputs[3] = mxCreateDoubleScalar(distCorrectedSamples);
        }
        
        if (numOutputs >= 5) {
            /* Return normalized covariance error back to MATLAB */
            const double covError = GLCD::normalizedCovError(samples);
            
            outputs[4] = mxCreateDoubleScalar(covError);
        }
    } catch (std::exception& ex) {
        std::stringstream ss;
        
        ss << ex.what();
        
        mexErrMsgIdAndTxt("GLCD:ComputationFailed", ss.str().c_str());
    }
}
