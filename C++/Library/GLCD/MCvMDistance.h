
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
 *                        Martin Pander <martin.pander@student.kit.edu>
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

#ifndef _GLCD_MCVM_DISTANCE_H_
#define _GLCD_MCVM_DISTANCE_H_

#include <Quadrature/QAG.h>
#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

namespace GLCD {

class MCvMDistance {
    public:
        typedef std::shared_ptr<MCvMDistance> Ptr;
        
    public:
        MCvMDistance(unsigned int dim);
        
        virtual ~MCvMDistance();
        
        virtual void setBMax(double bMax);
        
        double getBMax() const;
        
        virtual void setParameters(const Eigen::MatrixXd& parameters) = 0;
        
        virtual Eigen::MatrixXd getSamples() const = 0;
        
        void setQuadratureTol(double quadTol);
        
        void setQuadratureRule(Quadrature::GaussKronrod::Rule rule);
        
        virtual void compute(double& distance);
        
        template<typename Derived>
        void computeGradient(Eigen::MatrixBase<Derived>& gradient) {
            try {
                computeGrad1(grad1);
                
                computeGrad2(grad2);
                
                gradient = grad1 + grad2;
            } catch (std::exception& ex) {
                throw std::runtime_error(std::string("-> Computing mCvM distance gradient failed.\n") + ex.what());
            }
        }
        
    protected:
        virtual bool checkParameters(const Eigen::MatrixXd& parameters) const = 0;
        
        virtual void computeD2(double& D2) = 0;
        
        virtual void computeD3(double& D3) = 0;
        
        virtual void computeGrad1(Eigen::MatrixXd& grad1) = 0;
        
        virtual void computeGrad2(Eigen::MatrixXd& grad2) = 0;
        
        double integrate(Quadrature::Function& function);
        
        static double expInt(double x);
    
    private:
        void computeD1();
        
        double computeQuadD1(double b) const;
        
    protected:
        typedef Eigen::Array<double, 1, Eigen::Dynamic> RowArrayXd;
        
    protected:
        const unsigned int  dim;
        const double        halfDim;
        
        double              bMax;
        double              bMaxSquared;
        double              coeffSquaredNorm;
        
    private:
        Eigen::MatrixXd     grad1;
        Eigen::MatrixXd     grad2;
        double              D1;  
        Quadrature::QAG     quadrature;
        
};

}   // namespace GLCD

#endif

