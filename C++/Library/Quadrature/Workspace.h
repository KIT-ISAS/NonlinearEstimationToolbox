
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
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

#ifndef _QUADRATURE_WORKSPACE_H_
#define _QUADRATURE_WORKSPACE_H_

#include <cstdlib>

namespace Quadrature {

class Workspace {
    public:
        Workspace(unsigned int maxNumSubintervals);
        
        ~Workspace();
        
        void initialize(double a,
                        double b);
        
        void setInitialResult(double result,
                              double error);
        
        void update(double a1,
                    double b1,
                    double area1,
                    double error1,
                    double a2,
                    double b2,
                    double area2,
                    double error2);
        
        void retrieve(double& a,
                      double& b,
                      double& r,
                      double& e) const;
              
        void qpsrt();
        
        double sumResults() const;
        
        unsigned int getLimit() const {
            return limit;
        }
        
    private:
        unsigned int    limit;
        unsigned int    size;
        unsigned int    nrmax;
        unsigned int    i;
        unsigned int    maximum_level;
        double*         alist;
        double*         blist;
        double*         rlist;
        double*         elist;
        unsigned int*   order;
        unsigned int*   level;
        
};

}   // namespace Quadrature

#endif

