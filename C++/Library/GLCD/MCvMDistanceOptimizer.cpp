
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
#include <GLCD/MCvMDistanceOptimizer.h>
#include <stdexcept>

namespace GLCD {

MCvMDistanceOptimizer::MCvMDistanceOptimizer()
{
    setHistorySize(10);
    
    setMaxNumIterations(10000);
    
    setGradientTol(1e-8);
    
    setRelativeTol(1e-6);
    
    objFunc = std::bind(&MCvMDistanceOptimizer::objectiveFunction, this,
                        std::placeholders::_1,
                        std::placeholders::_2,
                        std::placeholders::_3);
}

MCvMDistanceOptimizer::~MCvMDistanceOptimizer()
{

}

void MCvMDistanceOptimizer::operator()(MCvMDistance::Ptr distance,
                                       const Eigen::MatrixXd& initialParameters,
                                       Optimization::Result& result)
{
    if (!distance) {
        throw std::invalid_argument("Invalid mCvM distance pointer.");
    }
    
    this->distance = distance;
    this->rows     = initialParameters.rows();
    this->cols     = initialParameters.cols();
    
    /* Start optimization */
    lbfgs(objFunc, createVectorMap(initialParameters), result);
}

void MCvMDistanceOptimizer::objectiveFunction(const Eigen::VectorXd& para,
                                              double& funcValue,
                                              Eigen::VectorXd& grad)
{
    const ConstMatrixMap parameters = createMatrixMap(para);
    MatrixMap gradient = createMatrixMap(grad);
    
    distance->setParameters(parameters);
    
    distance->compute(funcValue);
    
    distance->computeGradient(gradient);
}

void MCvMDistanceOptimizer::setHistorySize(unsigned int historySize)
{
    lbfgs.setHistorySize(historySize);
}

unsigned int MCvMDistanceOptimizer::getHistorySize() const
{
    return lbfgs.getHistorySize();
}

void MCvMDistanceOptimizer::setMaxNumIterations(unsigned int numIterations)
{
    lbfgs.setMaxNumIterations(numIterations);
}

unsigned int MCvMDistanceOptimizer::getMaxNumIterations() const
{
    return lbfgs.getMaxNumIterations();
}

void MCvMDistanceOptimizer::setGradientTol(double gradTol)
{
    lbfgs.setGradientTol(gradTol);
}

double MCvMDistanceOptimizer::getGradientTol() const
{
    return lbfgs.getGradientTol();
}

void MCvMDistanceOptimizer::setRelativeTol(double relTol)
{
    lbfgs.setRelativeTol(relTol);
}

double MCvMDistanceOptimizer::getRelativeTol() const
{
    return lbfgs.getRelativeTol();
}

}   // namespace GLCD
