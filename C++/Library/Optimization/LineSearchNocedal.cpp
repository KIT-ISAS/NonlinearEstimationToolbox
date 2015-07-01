
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
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

#include <Optimization/LineSearch.h>
#include <stdexcept>

namespace Optimization {

LineSearchNocedal::LineSearchNocedal()
{
    setMaxNumIterations(1000);
    
    setCoefficients(1e-4, 0.9);
}

LineSearchNocedal::~LineSearchNocedal()
{

}

bool LineSearchNocedal::search(Function&                function,
                               const Eigen::VectorXd&   initialParameters, 
                               const Eigen::VectorXd&   initialGradient,
                               const Eigen::VectorXd&   direction,
                               Eigen::VectorXd&         parameters,
                               double&                  funcValue,
                               Eigen::VectorXd&         gradient,
                               double&                  stepLength)
{
    /* Implements line search algorithm 3.5 from
     *   Jorge Nocedal and Stephen J. Wright,
     *   Numerical Optimization,
     *   Springer Series in Operations Research and Financial Engineering,
     *   2nd edition, Springer, 2006, page 60
     */
    
    // Step length has to be positive.
    if (stepLength <= 0) {
        throw std::invalid_argument("Initial step length must be greater than zero.");
    }
    
    const double initGradDotDir = initialGradient.dot(direction);
    
    // Ensure that the initial direction points to a descent direction.
    if (0 < initGradDotDir) {
        throw std::invalid_argument("Direction doesn't point to a descent direction.");
    }
    
    this->function       = &function;
    this->initParameters = &initialParameters;
    this->direction      = &direction;
    this->initFuncValue  = funcValue;
    this->testArmijo     = armijoCoeff * initGradDotDir;
    this->testWolfe      = wolfeCoeff * initGradDotDir;
    this->numIterations  = 0;
    
    double lastStepLength = 0.0;
    double lastFuncValue  = initFuncValue;
    double lastGradDotDir = initGradDotDir;
    
    while (true) {
        ++numIterations;
        
        // Evaluate the function and its gradient values.
        const double gradDotDir = evaluate(stepLength, parameters, funcValue, gradient);
        
        if (!checkArmijo(stepLength, funcValue) || funcValue >= lastFuncValue) {
            return zoom(lastStepLength, lastFuncValue, lastGradDotDir,
                        stepLength, funcValue, gradDotDir,
                        parameters, funcValue, gradient, stepLength);
        }
        
        if (checkWolfe(gradDotDir)) {
            // Line search was successful.
            return true;
        }
        
        if (gradDotDir >= 0.0) {
            return zoom(stepLength, funcValue, gradDotDir,
                        lastStepLength, lastFuncValue, lastGradDotDir,
                        parameters, funcValue, gradient, stepLength);
        }
        
        if (numIterations > maxNumIterations) {
            // Reached maximum number of allowed iteration.
            return false;
        }
        
        lastStepLength = stepLength;
        lastFuncValue  = funcValue;
        lastGradDotDir = gradDotDir;
        
        // Extrapolate step length in exponential fashion.
        stepLength = 2.0 * stepLength;
        
        if (std::isinf(stepLength)) {
            // Reached maximum possible step length.
            return false;
        }
    }
}

bool LineSearchNocedal::zoom(double           stepLengthLo, ////
                             double           funcValueLo,  // Smaller function value, not smaller step length!
                             double           gradDotDirLo, ////
                             double           stepLengthHi, ////
                             double           funcValueHi,  // Larger function value, not larger step length!
                             double           gradDotDirHi, ////
                             Eigen::VectorXd& parameters,
                             double&          funcValue,
                             Eigen::VectorXd& gradient,
                             double&          stepLength)
{
    /* Implements line search zoom algorithm 3.6 from
     *   Jorge Nocedal and Stephen J. Wright,
     *   Numerical Optimization,
     *   Springer Series in Operations Research and Financial Engineering,
     *   2nd edition, Springer, 2006, page 61
     */
    
    while (true) {
        ++numIterations;
        
        if (std::fabs(stepLengthHi - stepLengthLo) < DBL_EPSILON) {
            // Current step length interval too small.
            return false;
        }
        
        // Bisect current step length interval.
        stepLength = 0.5 * (stepLengthLo + stepLengthHi);
        
        // Evaluate the function and gradient values.
        const double gradDotDir = evaluate(stepLength, parameters, funcValue, gradient);
        
        if (!checkArmijo(stepLength, funcValue) || funcValue >= funcValueLo) {
            // Change upper bound.
            stepLengthHi = stepLength;
            funcValueHi  = funcValue;
            gradDotDirHi = gradDotDir;
        } else {
            if (checkWolfe(gradDotDir)) {
                // Line search was successful.
                return true;
            }
            
            if (gradDotDir * (stepLengthHi - stepLengthLo) >= 0) {
                // Change upper bound.
                stepLengthHi = stepLengthLo;
                funcValueHi  = funcValueLo;
                gradDotDirHi = gradDotDirLo;
            }
            
            // Change lower bound.
            stepLengthLo = stepLength;
            funcValueLo  = funcValue;
            gradDotDirLo = gradDotDir;
        }
        
        if (numIterations > maxNumIterations) {
            // Reached maximum number of allowed iteration.
            return false;
        }
    }
}

void LineSearchNocedal::setMaxNumIterations(unsigned int numIterations)
{
    if (numIterations < 1) {
        throw std::invalid_argument("Maximum number of iterations must be greater than zero.");
    }
    
    maxNumIterations = numIterations;
}

unsigned int LineSearchNocedal::getMaxNumIterations() const
{
    return maxNumIterations;
}

void LineSearchNocedal::setCoefficients(double armijoCoeff,
                                        double wolfeCoeff)
{
    if (armijoCoeff <= 0.0 || armijoCoeff >= 1.0) {
        throw std::invalid_argument("The Armijo coefficient must be in (0, 1).");
    }
    
    if (wolfeCoeff <= armijoCoeff || wolfeCoeff >= 1.0) {
        throw std::invalid_argument("The Wolfe coefficient must be in (armijoCoeff, 1).");
    }
    
    this->armijoCoeff = armijoCoeff;
    this->wolfeCoeff  = wolfeCoeff;
}

double LineSearchNocedal::getArmijoCoeff() const
{
    return armijoCoeff;
}

double LineSearchNocedal::getWolfeCoeff() const
{
    return wolfeCoeff;
}

}   // namespace Optimization
