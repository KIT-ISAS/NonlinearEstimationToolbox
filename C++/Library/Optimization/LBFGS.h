
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

#ifndef _OPTIMIZATION_LBFGS_H_
#define _OPTIMIZATION_LBFGS_H_

#include <Optimization/QuasiNewton.h>
#include <vector>

namespace Optimization {

class LBFGS : public QuasiNewton {
    public:
        LBFGS();
        
        ~LBFGS();
        
        void setHistorySize(unsigned int historySize);
        
        unsigned int getHistorySize() const {
            return history.size();
        }
        
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
        struct Entry {
            Eigen::VectorXd s;
            Eigen::VectorXd y;
            double          ys;
            double          rho;
            double          alpha;
        };
        
        typedef std::vector<Entry> History;
        
    private:
        History         history;
        unsigned int    insertPos;
        
};

}   // namespace Optimization

#endif
