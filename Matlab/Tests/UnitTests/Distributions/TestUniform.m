
classdef TestUniform < matlab.unittest.TestCase
    % Provides unit tests for the Uniform class.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    methods (Test)
        function testConstructorDefault(obj)
            u = Uniform();
            
            obj.verifyResetUniform(u);
        end
        
        function testConstructorScalar(obj)
            a       = -1;
            b       = 2;
            dim     = 1;
            mean    = 0.5;
            cov     = 9 / 12;
            covSqrt = sqrt(cov);
            
            u = Uniform(a, b);
            
            obj.verifyUniform(u, dim, a, b, mean, cov, covSqrt);
        end
        
        function testConstructorMultivariate(obj)
            a       = [-1, 3]';
            b       = [-0.5, 5]';
            dim     = 2;
            mean    = [-0.75, 4]';
            cov     = diag([0.25 4] / 12);
            covSqrt = chol(cov)';
            
            u = Uniform(a, b);
            
            obj.verifyUniform(u,  dim, a, b,mean, cov, covSqrt);
        end
        
        function testConstructorInvalidA(obj)
            obj.verifyError(@() Uniform(eye(2), 2), ...
                            'Uniform:InvalidMinimum');
            
            obj.verifyError(@() Uniform('A', 2), ...
                            'Uniform:InvalidMinimum');
        end
        
        function testConstructorInvalidB(obj)
            obj.verifyError(@() Uniform(1, eye(2)), ...
                            'Uniform:InvalidMaximum');
            
            obj.verifyError(@() Uniform(1, 'B'), ...
                            'Uniform:InvalidMaximum');
        end
        
        function testConstructorAGreaterEqualB(obj)
            obj.verifyError(@() Uniform(2, 2), ...
                            'Uniform:InvalidMinimum');
            
            obj.verifyError(@() Uniform(2, -1), ...
                            'Uniform:InvalidMinimum');
            
            obj.verifyError(@() Uniform([-1 2], [-2 3]), ...
                            'Uniform:InvalidMinimum');
        end
        
        
        function testSetScalar(obj)
            a       = -1;
            b       = 2;
            dim     = 1;
            mean    = 0.5;
            cov     = 9 / 12;
            covSqrt = sqrt(cov);
            
            u = Uniform();
            
            u.set(a, b);
            
            obj.verifyUniform(u, dim, a, b, mean, cov, covSqrt);
        end
        
        function testSetMultivariate(obj)
            a       = [-1, 3]';
            b       = [-0.5, 5]';
            dim     = 2;
            mean    = [-0.75, 4]';
            cov     = diag([0.25 4] / 12);
            covSqrt = chol(cov)';
            
            u = Uniform();
            
            u.set(a, b);
            
            obj.verifyUniform(u,  dim, a, b,mean, cov, covSqrt);
        end
        
        function testSetInvalidA(obj)
            u = Uniform();
            obj.verifyError(@() u.set(eye(2), 2), ...
                            'Uniform:InvalidMinimum');
            obj.verifyResetUniform(u);
            
            u = Uniform();
            obj.verifyError(@() u.set('A', 2), ...
                            'Uniform:InvalidMinimum');
            obj.verifyResetUniform(u);
        end
        
        function testSetInvalidB(obj)
            u = Uniform();
            obj.verifyError(@() u.set(1, eye(2)), ...
                            'Uniform:InvalidMaximum');
            obj.verifyResetUniform(u);
            
            u = Uniform();
            obj.verifyError(@() u.set(1, 'B'), ...
                            'Uniform:InvalidMaximum');
            obj.verifyResetUniform(u);
        end
        
        function testSetAGreaterEqualB(obj)
            u = Uniform();
            obj.verifyError(@() u.set(2, 2), ...
                            'Uniform:InvalidMinimum');
            obj.verifyResetUniform(u);
            
            u = Uniform();
            obj.verifyError(@() u.set(2, -1), ...
                            'Uniform:InvalidMinimum');
            obj.verifyResetUniform(u);
            
            u = Uniform();
            obj.verifyError(@() u.set([-1 2], [-2 3]), ...
                            'Uniform:InvalidMinimum');
            obj.verifyResetUniform(u);
        end
        
        
        function testDrawRndSamples(obj)
            a = [-1.5, 2]';
            b = [2.3, 2.5]';
            
            u = Uniform(a, b);
            
            n = 9;
            
            samples = u.drawRndSamples(n);
            
            obj.verifySize(samples, [2, n]);
            
            obj.verifyGreaterThanOrEqual(samples, repmat(a, 1, n));
            obj.verifyLessThanOrEqual(samples, repmat(b, 1, n));
        end
        
        function testDrawRndSamplesInvalidNumSamples(obj)
            u = Uniform(-1.5, 2.3);
            
            obj.verifyError(@() u.drawRndSamples(-3.4), ...
                            'Uniform:InvalidNumberOfSamples');
            
            obj.verifyError(@() u.drawRndSamples(eye(2)), ...
                            'Uniform:InvalidNumberOfSamples');
            
            obj.verifyError(@() u.drawRndSamples('alkdjf'), ...
                            'Uniform:InvalidNumberOfSamples');
        end
        
        
        function testLogPdf(obj)
            a = [-1.5, 2.0]';
            b = [ 2.3, 2.5]';
            u = Uniform(a, b);
            
            values = [1.2 -1.5 2.0 -3 -1 -3
                      2.1  2.3 2.5  2  5  4];
            
            logValues = u.logPdf(values);
            
            obj.verifySize(logValues, [1, 6]);
            
            c = -log((b(1) - a(1)) * (b(2) - a(2)));
            trueLogValues = [c c c -Inf -Inf -Inf];
            
            obj.verifyEqual(logValues, trueLogValues);
        end
        
        function testLogPdfInvalidValues(obj)
            u = Uniform(-1.5, 2.3);
            
            obj.verifyError(@() u.logPdf(ones(3, 10)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() u.logPdf(ones(2, 10, 5)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() u.logPdf('42kd'), ...
                            'Distribution:InvalidValues');
        end
    end
    
    methods (Access = 'private')
        function verifyUniform(obj, u, dim, a, b, mean, cov, covSqrt)
            tol = sqrt(eps);
            
            d = u.getDim();
            obj.verifyEqual(d, dim);
            
            [a_, b_] = u.getInterval();
            obj.verifyEqual(a_, a);
            obj.verifyEqual(b_, b);
            
            [m, c, cSqrt] = u.getMeanAndCov();
            obj.verifyEqual(m, mean, 'RelTol', tol);
            obj.verifyEqual(c, c');
            obj.verifyEqual(c, cov, 'RelTol', tol);
            obj.verifyEqual(cSqrt, covSqrt, 'RelTol', tol);
        end
        
        function verifyResetUniform(obj, u)
            obj.verifyUniform(u, 0, [], [], [], [], []);
        end
    end
end
