
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

#ifndef _GLCD_MCVM_DISTANCE_OPTIMIZER_H_
#define _GLCD_MCVM_DISTANCE_OPTIMIZER_H_

#include <Optimization/LBFGS.h>
#include <GLCD/MCvMDistance.h>

namespace GLCD {

class MCvMDistanceOptimizer {
    public:
        MCvMDistanceOptimizer();
        
        ~MCvMDistanceOptimizer();
        
        void operator()(MCvMDistance::Ptr distance,
                        const Eigen::MatrixXd& initialParameters,
                        Optimization::Result& result);
        
        void setHistorySize(unsigned int historySize);
        
        unsigned int getHistorySize() const;
        
        void setMaxNumIterations(unsigned int numIterations);
        
        unsigned int getMaxNumIterations() const;
        
        void setGradientTol(double gradTol);
        
        double getGradientTol() const;
        
        void setRelativeTol(double relTol);
        
        double getRelativeTol() const;
        
    private:
        typedef Eigen::Map<const Eigen::VectorXd>   ConstVectorMap;
        typedef Eigen::Map<const Eigen::MatrixXd>   ConstMatrixMap;
        typedef Eigen::Map<Eigen::MatrixXd>         MatrixMap;
        
        void objectiveFunction(const Eigen::VectorXd& para,
                               double& funcValue,
                               Eigen::VectorXd& grad);
        
        ConstVectorMap createVectorMap(const Eigen::MatrixXd& mat) const {
            return ConstVectorMap(mat.data(), rows * cols, 1);
        }
        
        ConstMatrixMap createMatrixMap(const Eigen::VectorXd& vec) const {
            return ConstMatrixMap(vec.data(), rows, cols);
        }
        
        MatrixMap createMatrixMap(Eigen::VectorXd& vec) const {
            return MatrixMap(vec.data(), rows, cols);
        }
        
    private:
        Optimization::LBFGS     lbfgs;
        Optimization::Function  objFunc;
        MCvMDistance::Ptr       distance;
        Eigen::MatrixXd::Index  rows;
        Eigen::MatrixXd::Index  cols;
        
};

}   // namespace GLCD

#endif
