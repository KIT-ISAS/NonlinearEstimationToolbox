
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
 *    The original quadrature code was taken from the GNU Scientific Library
 *    (GSL) version 1.16, <http://www.gnu.org/software/gsl/>.
 *
 *    Copyright (C) 1996, 1997, 1998, 1999, 2000, 2007 Brian Gough
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

#ifndef _QUADRATURE_QAG_H_
#define _QUADRATURE_QAG_H_

#include <Quadrature/Workspace.h>
#include <Quadrature/GaussKronrod.h>

namespace Quadrature {

class QAG {
    public:
        QAG(unsigned int maxNumSubintervals);
        
        void setAbsTol(double absTol);
        
        void setRelTol(double relTol);
        
        void setRule(GaussKronrod::Rule rule);
        
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr);
        
    private:
        bool isSubintervalTooSmall(double a1,
                                   double a2,
                                   double b2) const;
        
    private:
        Workspace           workspace;
        GaussKronrod::Ptr   gaussKronrod;
        double              epsabs;
        double              epsrel;
        
};

}   // namespace Quadrature

#endif

