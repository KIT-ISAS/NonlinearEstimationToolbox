
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

#include <Optimization/LBFGS.h>
#include <stdexcept>

namespace Optimization {

LBFGS::LBFGS()
{
    setHistorySize(100);
}

LBFGS::~LBFGS()
{

}

void LBFGS::initialDirection(const Eigen::VectorXd& gradient,
                             Eigen::VectorXd& direction)
{
    // Compute the initial direction = -H_0 * gradient,
    // with initial Hessian H_0 = 1 / ||gradient|| * I
    direction = -gradient / gradient.norm();
}

void LBFGS::operator()(const Function&          function,
                       const Eigen::VectorXd&   initialParameters,
                       Result&                  result)
{
    // Reset history insertion position
    insertPos = 0;
    
    // Call actual quasi Newton direction optimization
    QuasiNewton::operator()(function, initialParameters, result);
}

void LBFGS::updateDirection(const Eigen::VectorXd& parameters,
                            const Eigen::VectorXd& gradient,
                            const Eigen::VectorXd& lastParameters,
                            const Eigen::VectorXd& lastGradient,
                            unsigned int           numIterations,
                            Eigen::VectorXd&       direction)
{
    /* Implements the L-BFGS algorithm 7.4 from
     *   Jorge Nocedal and Stephen J. Wright,
     *   Numerical Optimization,
     *   Springer Series in Operations Research and Financial Engineering,
     *   2nd edition, Springer, 2006, page 178
     */
    
    const unsigned int historySize = history.capacity();
    
    // Update history.
    Entry& entry = history[insertPos];
    
    entry.s = parameters - lastParameters;
    entry.y = gradient - lastGradient;
    
    entry.ys  = entry.y.dot(entry.s);
    entry.rho = 1.0 / entry.ys;
    
    const unsigned int bound = (numIterations <= historySize) ? numIterations : historySize;
    
    direction = gradient;
    
    unsigned int j = insertPos;
    
    for (unsigned int i = 0; i < bound; ++i) {
        Entry& entry = history[j];
        
        entry.alpha = entry.rho * entry.s.dot(direction);
        
        direction -= entry.alpha * entry.y;
        
        j = (j - 1 + historySize) % historySize;
    }
    
    // direction = H^0_k * direction, with H^0_k = gamma * I
    const double gamma = entry.ys / entry.y.squaredNorm();
    
    direction *= gamma;
    
    for (unsigned int i = 0; i < bound; ++i) {
        j = (j + 1) % historySize;
        
        const Entry& entry = history[j];
        
        const double beta = entry.rho * entry.y.dot(direction);
        
        direction += (entry.alpha - beta) * entry.s;
    }
    
    direction = -direction;
    
    // Set the next history insertion position.
    insertPos = (insertPos + 1) % historySize;
}

void LBFGS::setHistorySize(unsigned int historySize)
{
    if (historySize < 1) {
        throw std::invalid_argument("History size must be greater than zero.");
    }
    
    history.resize(historySize);
}

unsigned int LBFGS::getHistorySize() const
{
    return history.capacity();
}

}   // namespace Optimization
