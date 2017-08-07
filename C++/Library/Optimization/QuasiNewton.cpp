
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
 *
 *    The original L-BFGS C code was taken from libLBFGS version 1.10,
 *    <http://www.chokkan.org/software/liblbfgs/>, licensed under the MIT license.
 *
 *    Copyright (C) 2007-2010 Naoaki Okazaki
 *    Copyright (C) 1990 Jorge Nocedal
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

#include <Optimization/QuasiNewton.h>
#include <stdexcept>

namespace Optimization {

QuasiNewton::QuasiNewton()
{
    setMaxNumIterations(10000);
    
    setGradientTol(1e-9);
    
    setRelativeTol(1e-9);
    
    setLineSearch(std::make_shared<LineSearchNocedal>());
    
    evalObjFunc = std::bind(&QuasiNewton::evaluateObjectiveFunction, this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            std::placeholders::_3);
}

QuasiNewton::~QuasiNewton()
{

}

void QuasiNewton::operator()(const Function&        function,
                             const Eigen::VectorXd& initialParameters,
                             Result&                result)
{
    const Eigen::VectorXd::Index numParameters = initialParameters.size();
    
    Eigen::VectorXd parameters = initialParameters;
    Eigen::VectorXd gradient(numParameters);
    Eigen::VectorXd direction(numParameters);
    Eigen::VectorXd lastParameters(numParameters);
    Eigen::VectorXd lastGradient(numParameters);
    double funcValue;
    double lastFuncValue;
    double firstOrderOpt;
    double lastFirstOrderOpt;
    unsigned int numIterations = 0;
    
    objFunc = function;
    numFuncEvaluations = 0;
    
    // Evaluate the function and its gradient.
    evalObjFunc(parameters, funcValue, gradient);
    
    // Ensure that the initial parameters are not a minimizer.
    firstOrderOpt = computeFirstOrderOpt(gradient);
    
    if (firstOrderOpt <= gradTol) {
        // Stop optimization.
        result.exitFlag             = Gradient;
        result.optParameters        = parameters;
        result.optFuncValue         = funcValue;
        result.firstOrderOptimality = firstOrderOpt;
        result.numIterations        = numIterations;
        result.numFuncEvaluations   = numFuncEvaluations;
        
        return;
    }
    
    // Compute the initial direction.
    initialDirection(gradient, direction);
    
    while (true) {
        ++numIterations;
        
        // Store the current parameter and gradient vectors.
        lastParameters    = parameters;
        lastGradient      = gradient;
        lastFuncValue     = funcValue;
        lastFirstOrderOpt = firstOrderOpt;
        
        /* Search for an optimal step length */
        
        // Try a step length of 1.0 first.
        double stepLength = 1.0;
        
        const bool ret = lineSearch->search(evalObjFunc, lastParameters,
                                            lastGradient, direction,
                                            parameters, funcValue,
                                            gradient, stepLength);
        
        if (!ret) {
            // Stop optimization with parameters from the last iteration
            result.exitFlag             = LineSearchFailed;
            result.optParameters        = lastParameters;
            result.optFuncValue         = lastFuncValue;
            result.firstOrderOptimality = lastFirstOrderOpt;
            result.numIterations        = numIterations;
            result.numFuncEvaluations   = numFuncEvaluations;
            
            return;
        }
        
        /* Gradient convergence test. */
        firstOrderOpt = computeFirstOrderOpt(gradient);
        
        if (firstOrderOpt <= gradTol) {
            // Stop optimization.
            result.exitFlag             = Gradient;
            result.optParameters        = parameters;
            result.optFuncValue         = funcValue;
            result.firstOrderOptimality = firstOrderOpt;
            result.numIterations        = numIterations;
            result.numFuncEvaluations   = numFuncEvaluations;
            
            return;
        }
        
        /* Relative convergence test. */
        if (std::fabs(funcValue - lastFuncValue) <= relTol * std::fabs(funcValue)) {
            // Stop optimization.
            result.exitFlag             = Relative;
            result.optParameters        = parameters;
            result.optFuncValue         = funcValue;
            result.firstOrderOptimality = firstOrderOpt;
            result.numIterations        = numIterations;
            result.numFuncEvaluations   = numFuncEvaluations;
            
            return;
        }
        
        /* Check for maximum number of allowed iterations. */
        if (numIterations >= maxNumIterations) {
            // Stop optimization.
            result.exitFlag             = MaxNumIterations;
            result.optParameters        = parameters;
            result.optFuncValue         = funcValue;
            result.firstOrderOptimality = firstOrderOpt;
            result.numIterations        = numIterations;
            result.numFuncEvaluations   = numFuncEvaluations;

            return;
        }
        
        /* Compute new direction */
        updateDirection(parameters, gradient,
                        lastParameters, lastGradient,
                        numIterations, direction);
    }
}

void QuasiNewton::setLineSearch(LineSearch::Ptr lineSearch)
{
    if (!lineSearch) {
        throw std::invalid_argument("Invalid line search pointer.");
    }
    
    this->lineSearch = lineSearch;
}

LineSearch::Ptr QuasiNewton::getLineSearch() const
{
    return lineSearch;
}
 
void QuasiNewton::setMaxNumIterations(unsigned int maxNumIterations)
{
    if (maxNumIterations < 1) {
        throw std::invalid_argument("Maximum number of allowed iterations must be greater than zero.");
    }
    
    this->maxNumIterations = maxNumIterations;
}

unsigned int QuasiNewton::getMaxNumIterations() const
{
    return maxNumIterations;
}

void QuasiNewton::setGradientTol(double gradTol)
{
    if (gradTol < 0.0) {
        throw std::invalid_argument("Gradient tolerance must be greater than or equal to zero.");
    }
    
    this->gradTol = gradTol;
}

double QuasiNewton::getGradientTol() const
{
    return gradTol;
}

void QuasiNewton::setRelativeTol(double relTol)
{
    if (relTol < 0.0) {
        throw std::invalid_argument("Relative tolerance must be greater than or equal to zero.");
    }
    
    this->relTol = relTol;
}

double QuasiNewton::getRelativeTol() const
{
    return relTol;
}

void QuasiNewton::evaluateObjectiveFunction(const Eigen::VectorXd&  parameters,
                                            double&                 funcValue,
                                            Eigen::VectorXd&        gradient)
{
    ++numFuncEvaluations;
    
    objFunc(parameters, funcValue, gradient);
}

std::ostream& operator<<(std::ostream& os,
                         const Result& result)
{
    os << "---------------- Quasi-Newton Optimization Summary ----------------\n";
    os << "Exit flag                     : ";
    
    if (result.exitFlag == Gradient) {
        os << "Reached gradient tolerance\n";
    } else if (result.exitFlag == Relative) {
        os << "Reached relative tolerance\n";
    } else if (result.exitFlag == MaxNumIterations) {
        os << "Reached maximum number of allowed iterations\n";
    } else if (result.exitFlag == LineSearchFailed) {
        os << "Line search failed\n";
    } else {
        os << "Unknown exit flag\n";
    }
    
    os << "Function value                : " << result.optFuncValue << std::endl;
    os << "First-order optimality        : " << result.firstOrderOptimality << std::endl;
    os << "Number of iterations          : " << result.numIterations << std::endl;
    os << "Number of function evaluations: " << result.numFuncEvaluations << std::endl;
    os << "-------------------------------------------------------------------\n";
    
    return os;
}

}   // namespace Optimization
