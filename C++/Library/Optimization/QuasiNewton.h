
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

#ifndef _OPTIMIZATION_QUASI_NEWTON_H_
#define _OPTIMIZATION_QUASI_NEWTON_H_

#include <Optimization/LineSearch.h>

namespace Optimization {

enum ExitFlag {
     Gradient,
     Relative,
     LineSearchFailed,
     MaxNumIterations
};

struct Result {
    ExitFlag            exitFlag;
    Eigen::VectorXd     optParameters;
    double              optFuncValue;
    double              firstOrderOptimality;
    unsigned int        numIterations;
    unsigned int        numFuncEvaluations;
};

class QuasiNewton {
    public:
        QuasiNewton();
        
        virtual ~QuasiNewton();
        
        virtual void operator()(const Function&         function,
                                const Eigen::VectorXd&  initialParameters,
                                Result&                 result);
        
        void setLineSearch(LineSearch::Ptr lineSearch);
        
        LineSearch::Ptr getLineSearch() const;
        
        void setMaxNumIterations(unsigned int maxNumIterations);
        
        unsigned int getMaxNumIterations() const;
        
        void setGradientTol(double gradTol);
        
        double getGradientTol() const;
        
        void setRelativeTol(double relTol);
        
        double getRelativeTol() const;
        
    private:
        virtual void initialDirection(const Eigen::VectorXd&    gradient,
                                      Eigen::VectorXd&          direction) = 0;
        
        virtual void updateDirection(const Eigen::VectorXd& parameters,
                                     const Eigen::VectorXd& gradient,
                                     const Eigen::VectorXd& lastParameters,
                                     const Eigen::VectorXd& lastGradient,
                                     unsigned int           numIterations,
                                     Eigen::VectorXd&       direction) = 0;
        
        void evaluateObjectiveFunction(const Eigen::VectorXd&   parameters,
                                       double&                  funcValue,
                                       Eigen::VectorXd&         gradient);
        
        static inline double computeFirstOrderOpt(const Eigen::VectorXd& gradient) {
            return gradient.lpNorm<Eigen::Infinity>();
        }
        
    private:
        LineSearch::Ptr     lineSearch;
        unsigned int        maxNumIterations;
        double              gradTol;
        double              relTol;
        
        Function            objFunc;
        Function            evalObjFunc;
        unsigned int        numFuncEvaluations;
        
};

std::ostream& operator<<(std::ostream& os,
                         const Result& result);

}   // namespace Optimization

#endif
