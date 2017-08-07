
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

#ifndef _QUADRATURE_GAUSS_KRONROD_H_
#define _QUADRATURE_GAUSS_KRONROD_H_

#include <functional>
#include <memory>

namespace Quadrature {

typedef std::function<double(double)> Function;

class GaussKronrod
{
    public:
        typedef std::shared_ptr<GaussKronrod> Ptr;
        
        enum Rule {
            Rule15,
            Rule21,
            Rule31,
            Rule41,
            Rule51,
            Rule61
        };
        
    public:
        virtual void operator()(Function function,
                                const double a,
                                const double b,
                                double& result,
                                double& abserr,
                                double& resabs,
                                double& resasc) = 0;
        
    protected:
        void evaluate(const int n, 
                      const double xgk[],
                      const double wg[],
                      const double wgk[],
                      Function function,
                      const double a,
                      const double b,
                      double fv1[],
                      double fv2[],
                      double& result,
                      double& abserr,
                      double& resabs,
                      double& resasc) const;
        
    private:
        double rescale_error(double err,
                             const double result_abs,
                             const double result_asc) const;
        
};

class GaussKronrod15 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

class GaussKronrod21 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

class GaussKronrod31 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

class GaussKronrod41 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

class GaussKronrod51 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

class GaussKronrod61 : public GaussKronrod
{
    public:
        void operator()(Function function,
                        const double a,
                        const double b,
                        double& result,
                        double& abserr,
                        double& resabs,
                        double& resasc);
        
    private:
        static const double xgk[];
        static const double wg[];
        static const double wgk[];
        
};

}   // namespace Quadrature

#endif
