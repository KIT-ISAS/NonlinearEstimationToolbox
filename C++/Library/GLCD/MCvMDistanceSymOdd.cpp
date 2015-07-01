
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

#include <GLCD/MCvMDistanceSymOdd.h>
#include <cmath>

namespace GLCD {

MCvMDistanceSymOdd::MCvMDistanceSymOdd(unsigned int d,
                                       unsigned int n) :
    MCvMDistanceSym (d, n, 1.0 / (2.0 * n + 1.0))
{
    tmpSquaredNorms    = RowArrayXd(numHalfSamples);
    expSquaredNorms    = RowArrayXd(numHalfSamples);
    expIntSquaredNorms = RowArrayXd(numHalfSamples);
}

MCvMDistanceSymOdd::~MCvMDistanceSymOdd()
{
    
}

void MCvMDistanceSymOdd::setBMax(double bMax)
{
    MCvMDistanceSym::setBMax(bMax);
    
    Quadrature::Function quadConstD2 = std::bind(&MCvMDistanceSymOdd::computeQuadConstD2,
                                                 this, std::placeholders::_1);
    double quad;
    
    try {
        quad = integrate(quadConstD2);
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Computing constant part of D2 failed.\n") + ex.what());
    }
    
    constD2 = sampleWeight * quad;
    
    constD3 = sampleWeightSquared * 0.5 * bMaxSquared;
}

void MCvMDistanceSymOdd::setParameters(const Eigen::MatrixXd& parameters)
{
    MCvMDistanceSym::setParameters(parameters);
    
    tmpSquaredNorms    = coeffSquaredNorm * squaredNorms;
    expSquaredNorms    = tmpSquaredNorms.exp();
    expIntSquaredNorms = tmpSquaredNorms.unaryExpr(std::ptr_fun(MCvMDistance::expInt));
}

Eigen::MatrixXd MCvMDistanceSymOdd::getSamples() const
{
    Eigen::MatrixXd samples(dim, 2 * numHalfSamples + 1);
    
    unsigned int j = 0;
    
    for (unsigned int i = 0; i < numHalfSamples; ++i) {
        samples.col(j++) =  halfSamples.col(i);
        samples.col(j++) = -halfSamples.col(i);
    }
    
    samples.col(j).setZero();
    
    return samples;
}

void MCvMDistanceSymOdd::computeD2(double& D2)
{
    MCvMDistanceSym::computeD2Base(D2);
    
    D2 += constD2;
}

void MCvMDistanceSymOdd::computeD3(double& D3)
{
    MCvMDistanceSym::computeD3Base(D3);
    
    D3 += 4.0 * sampleWeightSquared * (0.5 * bMaxSquared * expSquaredNorms + 
                                       0.125 * squaredNorms * expIntSquaredNorms).sum();
    
    D3 += constD3;
}

void MCvMDistanceSymOdd::computeGrad2(Eigen::MatrixXd& grad2)
{
    MCvMDistanceSym::computeGrad2Base(grad2);
    
    grad2 += sampleWeightSquared * (halfSamples.array().rowwise() * expIntSquaredNorms).matrix();
}

double MCvMDistanceSymOdd::computeQuadConstD2(double b) const
{
    const double bSquared = 2.0 * b * b;
    
    return std::pow(bSquared / (1.0 + bSquared), halfDim) * b;
}

}   // namespace GLCD

