
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

#include "Workspace.h"
#include <stdexcept>

namespace Quadrature {

Workspace::Workspace(unsigned int maxNumSubintervals)
{
    if (maxNumSubintervals == 0) {
        throw std::invalid_argument("Workspace length must be positive.");
    }
    
    alist = new double[maxNumSubintervals];
    blist = new double[maxNumSubintervals];
    rlist = new double[maxNumSubintervals];
    elist = new double[maxNumSubintervals];
    order = new unsigned int[maxNumSubintervals];
    level = new unsigned int[maxNumSubintervals];
    
    size          = 0;
    limit         = maxNumSubintervals;
    maximum_level = 0;
}

Workspace::~Workspace()
{
    delete[] alist;
    delete[] blist;
    delete[] rlist;
    delete[] elist;
    delete[] order;
    delete[] level;
}

void Workspace::initialize(double a,
                           double b)
{
    size          = 0;
    nrmax         = 0;
    i             = 0;
    alist[0]      = a;
    blist[0]      = b;
    rlist[0]      = 0.0;
    elist[0]      = 0.0;
    order[0]      = 0;
    level[0]      = 0;
    maximum_level = 0;
}

void Workspace::setInitialResult(double result,
                                 double error)
{
    size     = 1;
    rlist[0] = result;
    elist[0] = error;
}

void Workspace::update(double a1,
                       double b1,
                       double area1,
                       double error1,
                       double a2,
                       double b2,
                       double area2,
                       double error2)
{
    const unsigned int i_max     = i;
    const unsigned int i_new     = size;
    const unsigned int new_level = level[i_max] + 1;
    
    /* append the newly-created intervals to the list */
    
    if (error2 > error1) {
        alist[i_max] = a2;        /* blist[maxerr] is already == b2 */
        rlist[i_max] = area2;
        elist[i_max] = error2;
        level[i_max] = new_level;

        alist[i_new] = a1;
        blist[i_new] = b1;
        rlist[i_new] = area1;
        elist[i_new] = error1;
        level[i_new] = new_level;
    } else {
        blist[i_max] = b1;        /* alist[maxerr] is already == a1 */
        rlist[i_max] = area1;
        elist[i_max] = error1;
        level[i_max] = new_level;

        alist[i_new] = a2;
        blist[i_new] = b2;
        rlist[i_new] = area2;
        elist[i_new] = error2;
        level[i_new] = new_level;
    }
    
    size++;
    
    if (new_level > maximum_level) {
        maximum_level = new_level;
    }
    
    qpsrt();
}

void Workspace::retrieve(double& a,
                         double& b,
                         double& r,
                         double& e) const
{
    a = alist[i];
    b = blist[i];
    r = rlist[i];
    e = elist[i];
}

void Workspace::qpsrt()
{
    const unsigned int last = size - 1;
    
    unsigned int i_nrmax  = nrmax;
    unsigned int i_maxerr = order[i_nrmax];
    
    /* Check whether the list contains more than two error estimates */
    
    if (last < 2) {
        order[0] = 0;
        order[1] = 1;
        i        = i_maxerr;
        return;
    }
    
    const double errmax = elist[i_maxerr];
    
    /* This part of the routine is only executed if, due to a difficult
       integrand, subdivision increased the error estimate. In the normal
       case the insert procedure should start after the nrmax-th largest
       error estimate. */
    
    while (i_nrmax > 0 && errmax > elist[order[i_nrmax - 1]]) {
        order[i_nrmax] = order[i_nrmax - 1];
        i_nrmax--;
    } 
    
    /* Compute the number of elements in the list to be maintained in
       descending order. This number depends on the number of
       subdivisions still allowed. */
    unsigned int top;
    
    if (last < (limit / 2 + 2)) {
        top = last;
    } else {
        top = limit - last + 1;
    }
    
    /* Insert errmax by traversing the list top-down, starting
       comparison from the element elist(order(i_nrmax+1)). */
    
    unsigned int l = i_nrmax + 1;

    /* The order of the tests in the following line is important to
       prevent a segmentation fault */

    while (l < top && errmax < elist[order[l]]) {
        order[l - 1] = order[l];
        l++;
    }

    order[l - 1] = i_maxerr;

    /* Insert errmin by traversing the list bottom-up */
    
    const double errmin = elist[last];
    
    unsigned int k = top - 1;
    
    while (k > l - 2 && errmin >= elist[order[k]]) {
        order[k + 1] = order[k];
        k--;
    }
    
    order[k + 1] = last;
    
    i_maxerr = order[i_nrmax];
    
    i     = i_maxerr;
    nrmax = i_nrmax;
}

double Workspace::sumResults() const
{
    double result_sum = 0;
    
    for (unsigned int k = 0; k < size; k++) {
        result_sum += rlist[k];
    }
    
    return result_sum;
}

}   // namespace Quadrature

