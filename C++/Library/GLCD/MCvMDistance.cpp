
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

#include <GLCD/MCvMDistance.h>
#include <GLCD/ExpInt.h>
#include <Quadrature/QAG.h>
#include <stdexcept>
#include <cmath>

namespace GLCD {

MCvMDistance::MCvMDistance(unsigned int d) :
    dim        (d),
 	halfDim    (d * 0.5),
    quadrature (1000)
{
    quadrature.setAbsTol(0);
    
    setQuadratureTol(1e-10);
    setQuadratureRule(Quadrature::GaussKronrod::Rule31);
    
    setBMax(50);
}

MCvMDistance::~MCvMDistance()
{

}

void MCvMDistance::setBMax(double b)
{
    if (b <= 0.0) {
        throw std::invalid_argument("bMax must be greater than zero.");
    }
    
    if (!std::isfinite(b)) {
        throw std::invalid_argument("bMax is NaN or Inf.");
    }
    
    bMax             = b;
    bMaxSquared      = bMax * bMax;
    coeffSquaredNorm = -1.0 / (4.0 * bMaxSquared);
    
    computeD1();
}

double MCvMDistance::getBMax() const
{
    return bMax;
}

void MCvMDistance::setQuadratureTol(double quadTol)
{
    if (quadTol <= 0.0) {
        throw std::invalid_argument("Quadrature tolerance must be greater than zero.");
    }
    
    quadrature.setRelTol(quadTol);
}

void MCvMDistance::setQuadratureRule(Quadrature::GaussKronrod::Rule rule)
{
    quadrature.setRule(rule);
}

void MCvMDistance::compute(double& distance)
{
    try {
        double D2, D3;
        
        computeD2(D2);
        
        computeD3(D3);
        
        distance = D1 - 2.0 * D2 + D3;
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Computing mCvM distance failed.\n") + ex.what());
    }
}

double MCvMDistance::integrate(Quadrature::Function& function)
{
    double result;
    double error;
    
    quadrature(function, 0, bMax, result, error);
    
    return result;
}

double MCvMDistance::expInt(double x)
{
    if (x != 0) {
        return Ei(x);
    } else {
        return 0;
    }
}

void MCvMDistance::computeD1()
{
    Quadrature::Function quadD1 = std::bind(&MCvMDistance::computeQuadD1,
                                            this, std::placeholders::_1);
    
    try {
        D1 = integrate(quadD1);
    } catch (std::exception& ex) {
        throw std::runtime_error(std::string("Computing D1 failed.\n") + ex.what());
    }
}

double MCvMDistance::computeQuadD1(double b) const
{
    const double bSquared = b * b; 
    
    return std::pow(bSquared / (1.0 + bSquared), halfDim) * b;   
}

}   // namespace GLCD

