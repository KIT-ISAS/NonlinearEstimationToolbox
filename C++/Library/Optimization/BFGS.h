
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

#ifndef _OPTIMIZATION_BFGS_H_
#define _OPTIMIZATION_BFGS_H_

#include <Optimization/QuasiNewton.h>

namespace Optimization {

class BFGS : public QuasiNewton {
    public:
        BFGS();
        
        ~BFGS();
        
        void operator()(const Function&         function,
                        const Eigen::VectorXd&  initialParameters,
                        Result&                 result) override;
        
    private:
        void initialDirection(const Eigen::VectorXd&    gradient,
                              Eigen::VectorXd&          direction) override;
        
        void updateDirection(const Eigen::VectorXd&     parameters,
                             const Eigen::VectorXd&     gradient,
                             const Eigen::VectorXd&     lastParameters,
                             const Eigen::VectorXd&     lastGradient,
                             unsigned int               numIterations,
                             Eigen::VectorXd&           direction) override;
        
    private:
        unsigned int        numParameters;
        
        Eigen::MatrixXd     inverseHessian;
        Eigen::VectorXd     s;
        Eigen::VectorXd     y;
        Eigen::MatrixXd     ysOuter;
        Eigen::MatrixXd     ssOuter;
        Eigen::MatrixXd     A;
        Eigen::MatrixXd     B;
        
};

}   // namespace Optimization

#endif
