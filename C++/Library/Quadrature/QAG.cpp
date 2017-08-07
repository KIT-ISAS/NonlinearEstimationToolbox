
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

#include "QAG.h"
#include <cfloat>
#include <cmath>
#include <stdexcept>

namespace Quadrature {

QAG::QAG(unsigned int maxNumSubintervals)
    : workspace(maxNumSubintervals)
{
    setAbsTol(0);
    setRelTol(1e-10);
    
    setRule(GaussKronrod::Rule15);
}

void QAG::setAbsTol(double absTol)
{
    if (absTol < 0.0) {
        throw std::invalid_argument("Absolute tolerance must be greater than or equal to zero.");
    }
    
    this->epsabs = absTol;
}

void QAG::setRelTol(double relTol)
{
    if (relTol < 0.0) {
        throw std::invalid_argument("Relative tolerance must be greater than or equal to zero.");
    }
    
    this->epsrel = relTol;
}

void QAG::setRule(GaussKronrod::Rule rule)
{
    switch(rule) {
        case GaussKronrod::Rule15:
            gaussKronrod = std::make_shared<GaussKronrod15>();
            break;
        case GaussKronrod::Rule21:
            gaussKronrod = std::make_shared<GaussKronrod21>();
            break;
        case GaussKronrod::Rule31:
            gaussKronrod = std::make_shared<GaussKronrod31>();
            break;
        case GaussKronrod::Rule41:
            gaussKronrod = std::make_shared<GaussKronrod41>();
            break;
        case GaussKronrod::Rule51:
            gaussKronrod = std::make_shared<GaussKronrod51>();
            break;
        case GaussKronrod::Rule61:
            gaussKronrod = std::make_shared<GaussKronrod61>();
            break;
        default:
            throw std::invalid_argument("Invalid Gauss Kronrod rule.");
    }
}

void QAG::operator()(Function function,
                     const double a,
                     const double b,
                     double& result,
                     double& abserr)
{
    const size_t limit = workspace.getLimit();
    
    /* Initialize results */
    
    workspace.initialize(a, b);
    
    result = 0;
    abserr = 0;
    
    if (epsabs <= 0 && (epsrel < 50 * DBL_EPSILON || epsrel < 0.5e-28)) {
        throw std::runtime_error("Tolerance cannot be acheived with given absolute and relative error limits.");
    }
    
    /* perform the first integration */
    double result0;
    double abserr0;
    double resabs0;
    double resasc0;
    
    (*gaussKronrod)(function, a, b, result0, abserr0, resabs0, resasc0);
    
    workspace.setInitialResult(result0, abserr0);
    
    /* Test on accuracy */
    
    double tolerance = std::fmax(epsabs, epsrel * std::fabs(result0));
       
    /* need IEEE rounding here to match original quadpack behavior */
    
    const double round_off = 50 * DBL_EPSILON * resabs0;
    
    if (abserr0 <= round_off && abserr0 > tolerance) {
        result = result0;
        abserr = abserr0;
        
        throw std::runtime_error("Cannot reach tolerance because of roundoff error on first attempt.");
    } else if ((abserr0 <= tolerance && abserr0 != resasc0) || abserr0 == 0.0) {
        result = result0;
        abserr = abserr0;
        
        return;
    } else if (limit == 1) {
        result = result0;
        abserr = abserr0;
        
        throw std::runtime_error("A maximum of one iteration was insufficient.");
    }
    
    double area   = result0;
    double errsum = abserr0;
    
    int roundoff_type1 = 0;
    int roundoff_type2 = 0;
    int error_type     = 0;
        
    size_t iteration = 1;
    
    do {
        double a_i, b_i, r_i, e_i;
        double area1 = 0, area2 = 0;
        double error1 = 0, error2 = 0;
        double resasc1, resasc2;
        double resabs1, resabs2;
        
        /* Bisect the subinterval with the largest error estimate */
        
        workspace.retrieve(a_i, b_i, r_i, e_i);
        
        const double a1 = a_i;
        const double b1 = 0.5 * (a_i + b_i);
        const double a2 = b1;
        const double b2 = b_i;
        
        (*gaussKronrod)(function, a1, b1, area1, error1, resabs1, resasc1);
        (*gaussKronrod)(function, a2, b2, area2, error2, resabs2, resasc2);
        
        const double area12  = area1 + area2;
        const double error12 = error1 + error2;
        
        errsum += (error12 - e_i);
        area   += area12 - r_i;
        
        if (resasc1 != error1 && resasc2 != error2) {
            const double delta = r_i - area12;
            
            if (std::fabs(delta) <= 1.0e-5 * std::fabs(area12) && error12 >= 0.99 * e_i) {
                roundoff_type1++;
            }
            
            if (iteration >= 10 && error12 > e_i) {
                roundoff_type2++;
            }
        }
        
        tolerance = std::fmax(epsabs, epsrel * std::fabs(area));
        
        if (errsum > tolerance) {
            if (roundoff_type1 >= 6 || roundoff_type2 >= 20) {
                error_type = 1;   /* round off error */
            }
            
            /* set error flag in the case of bad integrand behaviour at
               a point of the integration range */
            if (isSubintervalTooSmall(a1, a2, b2)) {
                error_type = 2;
            }
        }
        
        workspace.update(a1, b1, area1, error1, a2, b2, area2, error2);
        
        workspace.retrieve(a_i, b_i, r_i, e_i);
        
        iteration++;
    } while (iteration < limit && !error_type && errsum > tolerance);
    
    result = workspace.sumResults();
    abserr = errsum;
    
    if (errsum <= tolerance) {
        return;
    } else if (error_type == 1) {
        throw std::runtime_error("Roundoff error prevents tolerance from being achieved.");
    } else if (error_type == 2) {
        throw std::runtime_error("Bad integrand behavior found in the integration interval.");
    } else if (iteration == limit) {
        throw std::runtime_error("Maximum number of subdivisions reached.");
    } else {
        throw std::runtime_error("Could not integrate function.");
    }
}

bool QAG::isSubintervalTooSmall(double a1,
                                double a2,
                                double b2) const
{
    const double e = DBL_EPSILON;
    const double u = DBL_MIN;
    
    const double tmp = (1 + 100 * e) * (std::fabs(a2) + 1000 * u);
    
    const bool status = std::fabs(a1) <= tmp && std::fabs(b2) <= tmp;
    
    return status;
}

}   // namespace Quadrature

