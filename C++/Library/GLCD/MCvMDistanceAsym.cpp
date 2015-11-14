
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
 *                        Martin Pander <martin.pander@student.kit.edu>
 *
 *                        Institute for Anthropomatics and Robotics
 *                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
 *                        Karlsruhe Institute of Technology (KIT), Germany
 *
 *                        http://isas.uka.de
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

#include <GLCD/MCvMDistanceAsym.h>
#include <stdexcept>
#include <cmath>

namespace GLCD {

MCvMDistanceAsym::MCvMDistanceAsym(unsigned int d,
                                   unsigned int n) :
    MCvMDistance        (d),
    numSamples          (n),
    sampleWeight        (1.0 / numSamples),
    sampleWeightSquared (sampleWeight * sampleWeight),
    coeffD2             (sampleWeight),
    coeffD3             (sampleWeightSquared),
    coeffGrad1          (2.0 * sampleWeight),
    coeffGrad2          (0.5 * sampleWeightSquared)
{
    /* Allocate memory */
    squaredNorms = RowArrayXd(numSamples);
    
    quadD2    = std::bind(&MCvMDistanceAsym::computeQuadD2, this, std::placeholders::_1);
    quadGrad1 = std::bind(&MCvMDistanceAsym::computeQuadGrad1, this, std::placeholders::_1);
}

MCvMDistanceAsym::~MCvMDistanceAsym()
{
    
}

void MCvMDistanceAsym::setParameters(const Eigen::MatrixXd& parameters)
{
    if (!checkParameters(parameters)) {
        throw std::invalid_argument("Invalid asymmetric Gaussian mCvM distance parameters.");
    }
    
    samples = parameters;
    
    squaredNorms = samples.colwise().squaredNorm();
}

Eigen::MatrixXd MCvMDistanceAsym::getSamples() const
{
    return samples;
}

bool MCvMDistanceAsym::checkParameters(const Eigen::MatrixXd& parameters) const
{
    if (parameters.rows() != dim || parameters.cols() != numSamples) {
        return false;
    } else {
        return true;
    }
}

void MCvMDistanceAsym::computeD2(double& D2)
{
    double quad;
    
    try {
        quad = integrate(quadD2);
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Numerical integration failed.\n") + ex.what());
    }
    
    D2 = coeffD2 * quad;
}

void MCvMDistanceAsym::computeD3(double& D3)
{
    double D3a = 0;
    double D3b = 0;
    
    #pragma omp parallel for reduction(+:D3a, D3b)
    for (int i = 0; i < numSamples; i++) {
        for (int j = 0; j < numSamples; j++) {
            const double snMinus  = (samples.col(i) - samples.col(j)).squaredNorm();
            const double csnMinus = coeffSquaredNorm * snMinus;
            
            D3a += std::exp(csnMinus);
            D3b += snMinus * expInt(csnMinus);
        }
    }
    
    D3 = coeffD3 * (0.5 * bMaxSquared * D3a + 0.125 * D3b);
}

void MCvMDistanceAsym::computeGrad1(Eigen::MatrixXd& grad1)
{ 
    grad1.resize(dim, numSamples);
    
    /* The numeric integration needs to be done for every sample individually */
    for (int i = 0; i < numSamples; i++) {
        double quad;
        
        grad1SquaredNorm = squaredNorms(i);
        
        try {
            quad = integrate(quadGrad1);
        } catch (std::exception& ex) {
            throw std::runtime_error(std::string("Numerical integration failed.\n") + ex.what());
        }
        
        grad1.col(i) = samples.col(i) * quad;
    }
    
    grad1 *= coeffGrad1;
}

void MCvMDistanceAsym::computeGrad2(Eigen::MatrixXd& grad2)
{
    grad2.resize(dim, numSamples);
    
    Eigen::VectorXd minus;
    
    grad2.setZero();
    
    #pragma omp parallel for private(minus)
    for (int e = 0; e < numSamples; e++) {
        for (int i = 0; i < numSamples; i++) {
            minus = samples.col(e) - samples.col(i);
            
            const double csnMinus = coeffSquaredNorm * minus.squaredNorm();
            
            grad2.col(e) += minus * expInt(csnMinus);
        }
    }
    
    grad2 *= coeffGrad2;
}

double MCvMDistanceAsym::computeQuadD2(double b) const
{
    const double bSquared    = 2.0 * b * b;
    const double bSquaredInv = 1.0 / (1.0 + bSquared);
    const double coeff       = std::pow(bSquared * bSquaredInv, halfDim) * b;
    
    return coeff * ((-0.5 * bSquaredInv) * squaredNorms).exp().sum();
}

double MCvMDistanceAsym::computeQuadGrad1(double b) const
{
    const double bSquared    = 2.0 * b * b;
    const double bSquaredInv = 1.0 / (1.0 + bSquared);
    const double coeff       = std::pow(bSquared * bSquaredInv, halfDim) * b * bSquaredInv;
    
    return coeff * std::exp((-0.5 * bSquaredInv) * grad1SquaredNorm);
}

}   // namespace GLCD
