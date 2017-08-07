
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
#include <Optimization/LBFGS.h>
#include <iostream>

static double objFunc(const Eigen::VectorXd& parameters)
{
    return std::exp(std::pow( 3 - parameters(0), 2.0) +
                    std::pow(-2 - parameters(1), 2.0));
}

static void objFuncAnalyticDerivative(const Eigen::VectorXd&   parameters,
                                      double&                  funcValue,
                                      Eigen::VectorXd&         gradient)
{
    funcValue = objFunc(parameters);
    
    gradient(0) = objFunc(parameters) * 2 * ( 3 - parameters(0)) * (-1);
    gradient(1) = objFunc(parameters) * 2 * (-2 - parameters(1)) * (-1);             
}

class ObjFuncApproxDerivative : public Optimization::ApproxDerivative {
    public:
        double objectiveFunction(const Eigen::VectorXd& parameters) override {
            return objFunc(parameters);
        }
        
};

int main()
{
    try {
        const Eigen::Vector2d initialParameters(-5, 10);
        ObjFuncApproxDerivative objFuncApproxDerivative;
        Optimization::BFGS bfgs;
        Optimization::LBFGS lbfgs;
        Optimization::Result result;
        
        // Using anyltic derivative & BFGS
        bfgs(objFuncAnalyticDerivative, initialParameters, result);
        
        std::cout << result << std::endl;
        
        std::cout << "Optimal parameters: " << result.optParameters.transpose() << std::endl << std::endl;
        
        // Using approximative derivative & BFGS
        bfgs(objFuncApproxDerivative, initialParameters, result);
        
        std::cout << result << std::endl;
        
        std::cout << "Optimal parameters: " << result.optParameters.transpose() << std::endl << std::endl;
        
        // Using anyltic derivative & L-BFGS
        lbfgs(objFuncAnalyticDerivative, initialParameters, result);
        
        std::cout << result << std::endl;
        
        std::cout << "Optimal parameters: " << result.optParameters.transpose() << std::endl << std::endl;
        
        // Using approximative derivative & L-BFGS
        lbfgs(objFuncApproxDerivative, initialParameters, result);
        
        std::cout << result << std::endl;
        
        std::cout << "Optimal parameters: " << result.optParameters.transpose() << std::endl << std::endl;
    } catch (std::exception& ex) {
        std::cout << "Error: " << ex.what() << std::endl;
        
        return 1;
    }
    
    return 0;
}

