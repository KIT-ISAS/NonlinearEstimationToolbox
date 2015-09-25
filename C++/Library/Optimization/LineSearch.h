
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

#ifndef _OPTIMIZATION_LINE_SEARCH_H_
#define _OPTIMIZATION_LINE_SEARCH_H_

#include <Optimization/Function.h>
#include <Eigen/Dense>
#include <memory>

namespace Optimization {

class LineSearch {
    public:
        typedef std::shared_ptr<LineSearch> Ptr;
        
    public:
        LineSearch() { }
        
        virtual ~LineSearch() { }
        
        virtual bool search(Function&               function,
                            const Eigen::VectorXd&  lastParameters, 
                            const Eigen::VectorXd&  lastGradient,
                            const Eigen::VectorXd&  direction,
                            Eigen::VectorXd&        parameters,
                            double&                 funcValue,
                            Eigen::VectorXd&        gradient,
                            double&                 stepLength) = 0;
        
};

class LineSearchNocedal : public LineSearch {
    public:
        LineSearchNocedal();
        
        ~LineSearchNocedal();
        
        bool search(Function&               function,
                    const Eigen::VectorXd&  lastParameters, 
                    const Eigen::VectorXd&  lastGradient,
                    const Eigen::VectorXd&  direction,
                    Eigen::VectorXd&        parameters,
                    double&                 funcValue,
                    Eigen::VectorXd&        gradient,
                    double&                 stepLength);
        
        /* The maximum number of allowed line search iterations.
         *  The default value is 1,000.
         */
        void setMaxNumIterations(unsigned int numIterations);
        
        unsigned int getMaxNumIterations() const;
        
        /* Set the coefficients for the Armijo and Wolfe conditions.
         *  The armijoCoeff must be in (0, 1).
         *  The default value is 1e-4.
         *  The wolfeCoeff must be in (armijoCoeff, 1).
         *  The default value is 0.9.
         */
        void setCoefficients(double armijoCoeff,
                             double wolfeCoeff);
        
        double getArmijoCoeff() const;
        
        double getWolfeCoeff() const;
        
    private:
        bool zoom(double           stepLengthLo, 
                  double           funcValueLo,
                  double           gradDotDirLo,
                  double           stepLengthHi,
                  double           funcValueHi,
                  double           gradDotDirHi,
                  Eigen::VectorXd& parameters,
                  double&          funcValue,
                  Eigen::VectorXd& gradient,
                  double&          stepLength);
        
        inline double evaluate(double           stepLength,
                               Eigen::VectorXd& parameters,
                               double&          funcValue,
                               Eigen::VectorXd& gradient) const {
            parameters = (*initParameters) + stepLength * (*direction);
            
            (*function)(parameters, funcValue, gradient);
            
            const double gradDotDir = gradient.dot(*direction);
            
            return gradDotDir;
        }
        
        inline bool checkArmijo(double stepLength,
                                double funcValue) const {
            // Check the Armijo condition (sufficient decrease condition).
            return funcValue <= (initFuncValue + stepLength * testArmijo);
        }
        
        inline bool checkWolfe(double gradDotDir) const {
            // Check the Wolfe condition (curvature condition).
            return gradDotDir >= testWolfe;
        }
        
    private:
        unsigned int            maxNumIterations;
        double                  armijoCoeff;
        double                  wolfeCoeff;
        
        Function*               function;
        const Eigen::VectorXd*  initParameters;
        const Eigen::VectorXd*  direction;        
        double                  initFuncValue;
        unsigned int            numIterations;
        double                  testArmijo;
        double                  testWolfe;
        
};

}   // namespace Optimization

#endif
