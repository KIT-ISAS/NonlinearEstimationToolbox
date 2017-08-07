
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
#include <iostream>

int main()
{
    try {
        GLCD::Computation computation;
        const int dimension  = 3;
        const int numSamples = 21;
        Eigen::MatrixXd samples;
        Eigen::MatrixXd initialParameters;
        
        Optimization::Result result = computation(dimension, numSamples,
                                                  initialParameters, samples);
        
        std::cout << ">> Samples approximating a three-dimensional standard normal distribution:\n"
                  << samples << std::endl << std::endl;
        
        const Eigen::VectorXd mean = samples.rowwise().sum() / numSamples;
        
        std::cout << ">> Sample mean:\n"
                  << mean << std::endl << std::endl;
        
        const Eigen::MatrixXd diffs = samples.colwise() - mean;
        
        std::cout << ">> Sample covariance matrix:\n"
                  << (diffs * diffs.transpose()) / numSamples << std::endl << std::endl;
        
        std::cout << ">> The optimization was initialized with these parameters:\n"
                  << initialParameters << std::endl << std::endl;
        
        std::cout << ">> Information about the optimization:\n\n"
                  << result << std::endl;
    } catch (std::exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
        
        return 1;
    }
    
    return 0;
}

