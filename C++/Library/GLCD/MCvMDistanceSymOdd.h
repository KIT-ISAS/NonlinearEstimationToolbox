
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

#ifndef _GLCD_MCVM_DISTANCE_SYMMETRIC_ODD_H_
#define _GLCD_MCVM_DISTANCE_SYMMETRIC_ODD_H_

#include <GLCD/MCvMDistanceSym.h>

namespace GLCD {

class MCvMDistanceSymOdd : public MCvMDistanceSym {
    public:
        MCvMDistanceSymOdd(unsigned int dim,
                           unsigned int numHalfSamples);
        
        ~MCvMDistanceSymOdd();
        
        void setBMax(double bMax) override;
        
        void setParameters(const Eigen::MatrixXd& parameters) override;
        
        Eigen::MatrixXd getSamples() const override;
        
    protected:
        void computeD2(double& D2) override;
        
        void computeD3(double& D3) override;
        
        void computeGrad2(Eigen::MatrixXd& grad2) override;
        
    private:
        double computeQuadConstD2(double b) const;
        
    private:
        RowArrayXd      tmpSquaredNorms;
        RowArrayXd      expSquaredNorms;
        RowArrayXd      expIntSquaredNorms;
        
        double          constD2;
        double          constD3;
        
};

}  // namespace GLCD

#endif
