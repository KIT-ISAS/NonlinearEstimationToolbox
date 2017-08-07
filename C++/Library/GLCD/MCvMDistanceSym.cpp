
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

#include <GLCD/MCvMDistanceSym.h>
#include <stdexcept>
#include <cmath>

namespace GLCD {

MCvMDistanceSym::MCvMDistanceSym(unsigned int d,
                                 unsigned int n,
                                 double w) :
    MCvMDistance        (d),
    numHalfSamples      (n),
    sampleWeight        (w),
    sampleWeightSquared (sampleWeight * sampleWeight),
    coeffD2             (2.0 * sampleWeight),
    coeffD3             (2.0 * sampleWeightSquared),
    coeffGrad1          (4.0 * sampleWeight),
    coeffGrad2          (sampleWeightSquared)
{
    /* Allocate memory */
    squaredNorms = RowArrayXd(numHalfSamples);
    
    quadD2    = std::bind(&MCvMDistanceSym::computeQuadD2, this, std::placeholders::_1);
    quadGrad1 = std::bind(&MCvMDistanceSym::computeQuadGrad1, this, std::placeholders::_1);
}

MCvMDistanceSym::~MCvMDistanceSym()
{
    
}

void MCvMDistanceSym::setParameters(const Eigen::MatrixXd& parameters)
{
    if (!checkParameters(parameters)) {
        throw std::invalid_argument("Invalid symmetric Gaussian mCvM distance parameters.");
    }
    
    halfSamples = parameters;
    
    squaredNorms = halfSamples.colwise().squaredNorm();
}

bool MCvMDistanceSym::checkParameters(const Eigen::MatrixXd& parameters) const
{
    if (parameters.rows() != dim || parameters.cols() != numHalfSamples) {
        return false;
    } else {
        return true;
    }
}

void MCvMDistanceSym::computeD2Base(double& D2)
{
    double quad;
    
    try {
        quad = integrate(quadD2);
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Numerical integration failed.\n") + ex.what());
    }
    
    D2 = coeffD2 * quad;
}

void MCvMDistanceSym::computeGrad1(Eigen::MatrixXd& grad1)
{
    grad1.resize(dim, numHalfSamples);
    
    /* The numeric integration needs to be done for every sample individually */
    for (int i = 0; i < numHalfSamples; i++) {
        double quad;
        
        grad1SquaredNorm = squaredNorms(i);
        
        try {
            quad = integrate(quadGrad1);
        } catch (std::exception& ex) {
            throw std::runtime_error(std::string("Numerical integration failed.\n") + ex.what());
        }
        
        grad1.col(i) = halfSamples.col(i) * quad;
    }
    
    grad1 *= coeffGrad1;
}

void MCvMDistanceSym::computeD3Base(double& D3)
{
    double D3a = 0;
    double D3b = 0;
    
    #pragma omp parallel for reduction(+:D3a, D3b)
    for (int i = 0; i < numHalfSamples; i++) {
        for (int j = 0; j < numHalfSamples; j++) {
            const double snMinus  = (halfSamples.col(i) - halfSamples.col(j)).squaredNorm();
            const double snPlus   = (halfSamples.col(i) + halfSamples.col(j)).squaredNorm();
            
            const double csnMinus = coeffSquaredNorm * snMinus;
            const double csnPlus  = coeffSquaredNorm * snPlus;
            
            D3a += std::exp(csnMinus) + std::exp(csnPlus);
            D3b += snMinus * expInt(csnMinus) + snPlus * expInt(csnPlus);
        }
    }
    
    D3 = coeffD3 * (0.5 * bMaxSquared * D3a + 0.125 * D3b);
}

void MCvMDistanceSym::computeGrad2Base(Eigen::MatrixXd& grad2)
{
    grad2.resize(dim, numHalfSamples);
    
    Eigen::VectorXd minus;
    Eigen::VectorXd plus;
    
    grad2.setZero();
    
    #pragma omp parallel for private(minus, plus)
    for (int e = 0; e < numHalfSamples; e++) {
        for (int i = 0; i < numHalfSamples; i++) {
            minus = halfSamples.col(e) - halfSamples.col(i);
            plus  = halfSamples.col(e) + halfSamples.col(i);
            
            const double csnMinus = coeffSquaredNorm * minus.squaredNorm();
            const double csnPlus  = coeffSquaredNorm * plus.squaredNorm();
            
            grad2.col(e) += minus * expInt(csnMinus) + plus * expInt(csnPlus);
        }
    }
    
    grad2 *= coeffGrad2;
}

double MCvMDistanceSym::computeQuadD2(double b) const
{
    const double bSquared    = 2.0 * b * b;
    const double bSquaredInv = 1.0 / (1.0 + bSquared);
    const double coeff       = std::pow(bSquared * bSquaredInv, halfDim) * b;
    
    return coeff * ((-0.5 * bSquaredInv) * squaredNorms).exp().sum();
}

double MCvMDistanceSym::computeQuadGrad1(double b) const
{
    const double bSquared    = 2.0 * b * b;
    const double bSquaredInv = 1.0 / (1.0 + bSquared);
    const double coeff       = std::pow(bSquared * bSquaredInv, halfDim) * b * bSquaredInv;
    
    return coeff * std::exp((-0.5 * bSquaredInv) * grad1SquaredNorm);
}

}   // namespace GLCD
