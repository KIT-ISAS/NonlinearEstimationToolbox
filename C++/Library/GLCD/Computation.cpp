
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
 *                             Martin Pander
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
#include <GLCD/MCvMDistanceAsym.h>
#include <GLCD/MCvMDistanceSymEven.h>
#include <GLCD/MCvMDistanceSymOdd.h>
#include <GLCD/Misc.h>
#include <stdexcept>

namespace GLCD {

Computation::Computation()
{
    // By default, use varying bMax
    setBMax(-1);
    
    // By default, use point symmetric sampling
    setSymmetric(true);
}

Computation::~Computation()
{

}

void Computation::setSymmetric(bool useSymmetric)
{
    this->useSymmetric = useSymmetric;
}

void Computation::setBMax(double bMax)
{
    this->forceBMax = bMax;
}

Optimization::Result Computation::operator()(int dimension,
                                             int numSamples,
                                             Eigen::MatrixXd& initialParameters,
                                             Eigen::MatrixXd& samples,
                                             double* distCorrectedSamples)
{
    MCvMDistance::Ptr distance;
    
    if (dimension <= 0) {
        throw std::invalid_argument("The dimension must be greater than zero.");
    }
    
    if (useSymmetric) {
        if (numSamples < 2 * dimension) {
            throw std::invalid_argument("The number of samples must be at least twice the dimension.");
        }
        
        const unsigned int numHalfSamples = numSamples / 2;
        const bool isEven = (numSamples % 2) == 0;
        
        if (isEven) {
            distance = std::make_shared<GLCD::MCvMDistanceSymEven>(dimension, numHalfSamples);
        } else {
            distance = std::make_shared<GLCD::MCvMDistanceSymOdd>(dimension, numHalfSamples);
        }
        
        if (initialParameters.size() == 0) {
            stdNormalRndMatrix(dimension, numHalfSamples, initialParameters);
        }
    } else {
        if (numSamples <= dimension) {
            throw std::invalid_argument("The number of samples must be greater than the dimension.");
        }
        
        distance = std::make_shared<GLCD::MCvMDistanceAsym>(dimension, numSamples);
        
        if (initialParameters.size() == 0) {
            stdNormalRndMatrix(dimension, numSamples, initialParameters);
        }
    }
    
    double bMax;
    
    if (forceBMax < 0) {
        // Use different bMax for different dimensions
        if (dimension == 1) {
            bMax =   5;
        } else if (dimension <= 10) {
            bMax =  10;
        } else if (dimension <= 100) {
            bMax =  50;
        } else if (dimension <= 1000) {
            bMax = 100;
        } else {
            bMax = 200;
        }
    } else {
        bMax = forceBMax;
    }
    
    distance->setBMax(bMax);
    
    MCvMDistanceOptimizer optimizer;
    
    Optimization::Result result;
    
    try {
        optimizer(distance, initialParameters, result);
        
        if (result.exitFlag == Optimization::MaxNumIterations) {
            throw std::runtime_error("Maximum number of allowed iterations reached.");
        }
        
        if (result.exitFlag == Optimization::LineSearchFailed) {
            throw std::runtime_error("Line search failed.");
        }
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Quasi-Newton optimization failed.\n") + ex.what());
    }
    
    if (useSymmetric) {
        covarianceCorrection(distance->getSamples(), samples);
    } else {
        meanCorrection(distance->getSamples(), samples);
        covarianceCorrection(samples, samples);
    }
    
    if (!samples.allFinite()) {
        throw std::runtime_error("Computed invalid sample positions.");
    }
    
    if (distCorrectedSamples) {
        GLCD::MCvMDistance::Ptr dist = std::make_shared<GLCD::MCvMDistanceAsym>(dimension, numSamples);
        
        dist->setBMax(bMax);
        
        dist->setParameters(samples);
        
        dist->compute(*distCorrectedSamples);
    }
    
    return result;
}

}   // namespace GLCD

