
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

#ifndef _GLCD_COMPUTATION_H_
#define _GLCD_COMPUTATION_H_

#include <GLCD/MCvMDistanceOptimizer.h>

namespace GLCD {

class Computation {
    public:
        Computation();
        
        ~Computation();
        
        void setSymmetric(bool useSymmetric);
        
        void setBMax(double bMax);
        
        Optimization::Result operator()(int dimension,
                                        int numSamples,
                                        Eigen::MatrixXd& initialParameters,
                                        Eigen::MatrixXd& samples,
                                        double* distCorrectedSamples = nullptr);
        
    private:
        bool    useSymmetric;
        double  forceBMax;
        
};

}   // namespace GLCD

#endif

