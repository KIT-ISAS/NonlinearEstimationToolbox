
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

#include <Optimization/BFGS.h>

namespace Optimization {

BFGS::BFGS()
{
    
}

BFGS::~BFGS()
{

}

void BFGS::operator()(const Function&           function,
                      const Eigen::VectorXd&    initialParameters,
                      Result&                   result)
{
    numParameters = initialParameters.size();
    
    // Call actual quasi Newton optimization
    QuasiNewton::operator()(function, initialParameters, result);
}

void BFGS::initialDirection(const Eigen::VectorXd&  gradient,
                            Eigen::VectorXd&        direction)
{
    // Compute initial hessian H_0 = 1 / ||gradient|| * I
    inverseHessian = Eigen::MatrixXd::Identity(numParameters, numParameters) / gradient.norm();
    
    // Compute initial direction
    direction = -inverseHessian * gradient;
}

void BFGS::updateDirection(const Eigen::VectorXd& parameters,
                           const Eigen::VectorXd& gradient,
                           const Eigen::VectorXd& lastParameters,
                           const Eigen::VectorXd& lastGradient,
                           unsigned int           numIterations,
                           Eigen::VectorXd&       direction)
{
    /* Implements the BFGS algorithm from
     *   Jorge Nocedal and Stephen J. Wright,
     *   Numerical Optimization,
     *   Springer Series in Operations Research and Financial Engineering,
     *   2nd edition, Springer, 2006, page 177
     */
    
    // Update approximative inverse Hessian
    s = parameters - lastParameters;
    y = gradient - lastGradient;
    
    double ysInner = y.dot(s);
    
    ysOuter = y * s.transpose();
    ssOuter = s * s.transpose();
    
    A           = Eigen::MatrixXd::Identity(numParameters, numParameters) - ysOuter / ysInner;
    B.noalias() = A.transpose() * inverseHessian * A;
    
    inverseHessian = B + ssOuter / ysInner;
    
    // Compute new direction
    direction = -inverseHessian * gradient;
}

}   // namespace Optimization
