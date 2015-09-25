
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

#ifndef _OPTIMIZATION_FUNCTION_H_
#define _OPTIMIZATION_FUNCTION_H_

#include <Eigen/Dense>
#include <functional>
#include <cfloat>

namespace Optimization {

typedef std::function<void(const Eigen::VectorXd&   parameters,
                           double&                  funcValue,
                           Eigen::VectorXd&         gradient)> Function;

class ApproxDerivative {
    public:
        ApproxDerivative() { }
        
        virtual ~ApproxDerivative() { }
        
        void operator()(const Eigen::VectorXd&  parameters,
                        double&                 funcValue,
                        Eigen::VectorXd&        gradient) {
            const Eigen::VectorXd::Index numParameters = parameters.size();
            const double epsilon    = std::sqrt(DBL_EPSILON);
            const double invEpsilon = 1.0 / epsilon;
            funcValue = objectiveFunction(parameters);
            
            gradParameters = parameters;
            
            for (Eigen::VectorXd::Index i = 0;  i < numParameters; ++i) {
                // Add epsilon to ith parameter.
                gradParameters(i) += epsilon;
                
                const double value = objectiveFunction(gradParameters);
                
                gradient(i) = (value - funcValue) * invEpsilon;
                
                // Restore original parameter.
                gradParameters(i) = parameters(i);
            }
        }
        
    protected:
        virtual double objectiveFunction(const Eigen::VectorXd& parameters) = 0;
        
    private:
        Eigen::VectorXd gradParameters;
        
};

}   // namespace Optimization

#endif
