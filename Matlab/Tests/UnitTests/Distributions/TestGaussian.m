
classdef TestGaussian < matlab.unittest.TestCase
    % Provides unit tests for the Gaussian class.
    
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
            g = Gaussian();
            
            obj.verifyResetGaussian(g);
        end
        
        function testConstructorUncorrelated(obj)
            dim     = 3;
            mean    = [1 -2 0.3]';
            cov     = [1 0.2 3.5];
            covSqrt = chol(diag(cov))';
            
            g = Gaussian(mean, cov);
            
            obj.verifyGaussian(g, dim, mean, diag(cov), covSqrt);
        end
        
        function testConstructorCorrelated(obj)
            dim     = 2;
            mean    = [1 -2]';
            cov     = [2 0.5; 0.5 1.2];
            covSqrt = chol(cov)';
            
            g = Gaussian(mean, cov);
            
            obj.verifyGaussian(g, dim, mean, cov, covSqrt);
        end
        
        function testConstructorBlockCovariance(obj)
            dim     = 6;
            mean    = [1 -2 0 0 3 4]';
            cov3D   = cat(3, [2 0.5; 0.5 1.2], eye(2), 3 * eye(2));
            cov     = blkdiag(cov3D(:, :, 1), cov3D(:, :, 2), cov3D(:, :, 3));
            covSqrt = chol(cov)';
            
            g = Gaussian(mean, cov3D);
            
            obj.verifyGaussian(g, dim, mean, cov, covSqrt);
        end
        
        function testConstructorInvalidMean(obj)
            cov = [2 0.5; 0.5 1.2];
            
            obj.verifyError(@() Gaussian([1 -2], cov), ...
                            'Gaussian:InvalidMean');
            
            obj.verifyError(@() Gaussian('k2', cov), ...
                            'Gaussian:InvalidMean');
        end
        
        function testConstructorInvalidVariances(obj)
            mean = [1 -2]';
            
            obj.verifyError(@() Gaussian(mean, 1), ...
                            'Gaussian:InvalidCovariance');
            
            obj.verifyError(@() Gaussian(mean, [1.5 -1]), ...
                            'Gaussian:InvalidCovariance');
            
            obj.verifyError(@() Gaussian(mean, 'kl2'), ...
                            'Gaussian:InvalidCovariance');
        end
        
        function testConstructorInvalidCovariance(obj)
            mean = [1 -2]';
            
            obj.verifyError(@() Gaussian(mean, eye(3)), ...
                            'Gaussian:InvalidCovariance');
            
            obj.verifyError(@() Gaussian(mean, -eye(2)), ...
                            'Gaussian:InvalidCovariance');
        end
        
        function testConstructorInvalidBlockCovariance(obj)
            mean = [1 -2 4 2]';
            
            obj.verifyError(@() Gaussian(mean, cat(3, eye(3), 3 * eye(3))), ...
                            'Gaussian:InvalidCovariance');
            
            obj.verifyError(@() Gaussian(mean, cat(3, eye(2), ones(2, 2))), ...
                            'Gaussian:InvalidCovariance');
        end
        
        
        function testSetUncorrelated(obj)
            dim     = 3;
            mean    = [1 -2 0.3]';
            cov     = [1 0.2 3.5];
            covSqrt = chol(diag(cov))';
            
            g = Gaussian();
            
            g.set(mean, cov);
            
            obj.verifyGaussian(g, dim, mean, diag(cov), covSqrt);
        end
        
        function testSetCorrelated(obj)
            dim     = 2;
            mean    = [1 -2]';
            cov     = [2 0.5; 0.5 1.2];
            covSqrt = chol(cov)';
            
            g = Gaussian();
            
            g.set(mean, cov);
            
            obj.verifyGaussian(g, dim, mean, cov, covSqrt);
        end
        
        function testSetBlockCovariance(obj)
            dim     = 6;
            mean    = [1 -2 0 0 3 4]';
            cov3D   = cat(3, [2 0.5; 0.5 1.2], eye(2), 3 * eye(2));
            cov     = blkdiag(cov3D(:, :, 1), cov3D(:, :, 2), cov3D(:, :, 3));
            covSqrt = chol(cov)';
            
            g = Gaussian();
            
            g.set(mean, cov3D);
            
            obj.verifyGaussian(g, dim, mean, cov, covSqrt);
        end
        
        function testSetInvalidMean(obj)
            cov = [2 0.5; 0.5 1.2];
            
            g = Gaussian();
            obj.verifyError(@() g.set([1 -2], cov), ...
                            'Gaussian:InvalidMean');
            obj.verifyResetGaussian(g);
            
            g = Gaussian();
            obj.verifyError(@() g.set('k2', cov), ...
                            'Gaussian:InvalidMean');
           obj.verifyResetGaussian(g);
        end
        
        function testSetInvalidVariances(obj)
            mean = [1 -2]';
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, 1), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, [1.5 -1]), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, 'kl2'), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
        end
        
        function testSetInvalidCovariance(obj)
            mean = [1 -2]';
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, eye(3)), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, -eye(2)), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
        end
        
        function testSetInvalidBlockCovariance(obj)
            mean = [1 -2 4 2]';
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, cat(3, eye(3), 3 * eye(3))), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
            
            g = Gaussian();
            obj.verifyError(@() g.set(mean, cat(3, eye(2), ones(2, 2))), ...
                            'Gaussian:InvalidCovariance');
            obj.verifyResetGaussian(g);
        end
        
        
        function testDrawRndSamples(obj)
            mean = [1 -2]';
            cov  = [2 0.5; 0.5 1.2];
            
            g = Gaussian(mean, cov);
            
            samples = g.drawRndSamples(9);
            
            obj.verifySize(samples, [2, 9]);
        end
        
        function testDrawRndSamplesInvalidNumSamples(obj)
            mean = [1 -2]';
            cov  = [2 0.5; 0.5 1.2];
            
            g = Gaussian(mean, cov);
            
            obj.verifyError(@() g.drawRndSamples(-3.4), ...
                            'Gaussian:InvalidNumberOfSamples');
            
            obj.verifyError(@() g.drawRndSamples(eye(2)), ...
                            'Gaussian:InvalidNumberOfSamples');
            
            obj.verifyError(@() g.drawRndSamples('Dk23'), ...
                            'Gaussian:InvalidNumberOfSamples');
        end
        
        
        function testLogPdfUncorrelated(obj)
            mean   = [1 -2]';
            var    = [2 1.2];
            cov    = diag(var);
            values = [0.1 -0.5 13.4
                      0.9 -10   5  ];
            
            g = Gaussian(mean, var);
            
            logValues = g.logPdf(values);
            
            obj.verifySize(logValues, [1, 3]);
            
            trueLogValues = TestGaussian.computeLogValues(mean, cov, values);
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfCorrelated(obj)
            mean   = [1 -2]';
            cov    = [2 0.5; 0.5 1.2];
            values = [0.1 -0.5 13.4
                      0.9 -10   5  ];
            
            g = Gaussian(mean, cov);
            
            logValues = g.logPdf(values);
            
            obj.verifySize(logValues, [1, 3]);
            
            trueLogValues = TestGaussian.computeLogValues(mean, cov, values);
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfBlockCovariance(obj)
            mean   = [1 -2 3 4]';
            cov3D  = cat(3, [2 0.5; 0.5 1.2], 3 * eye(2));
            cov    = blkdiag(cov3D(:, :, 1), cov3D(:, :, 2));
            values = [ 0.1 -0.5  13.4
                       0.9 -10   5
                      -1.3  4.7 -10
                       1   -2    3];
            
            g = Gaussian(mean, cov3D);
            
            logValues = g.logPdf(values);
            
            obj.verifySize(logValues, [1, 3]);
            
            trueLogValues = TestGaussian.computeLogValues(mean, cov, values);
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfInvalidValues(obj)
            mean = [1 -2]';
            cov  = [2 0.5; 0.5 1.2];
            
            g = Gaussian(mean, cov);
            
            obj.verifyError(@() g.logPdf([0.1 -0.5 13.4]), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() g.logPdf(ones(3, 10)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() g.logPdf(ones(2, 10, 5)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() g.logPdf('k2j'), ...
                            'Distribution:InvalidValues');
        end
    end
    
    methods (Access = 'private')
        function verifyGaussian(obj, g, dim, mean, cov, covSqrt)
            d = g.getDim();
            obj.verifyEqual(d, dim);
            
            [m, c, cSqrt] = g.getMeanAndCov();
            obj.verifyEqual(m, mean);
            obj.verifyEqual(c, c');
            obj.verifyEqual(c, cov);
            obj.verifyEqual(cSqrt, covSqrt);
        end
        
        function verifyResetGaussian(obj, g)
            obj.verifyGaussian(g, 0, [], [], []);
        end
    end
    
    methods (Static, Access = 'private')
        function logValues = computeLogValues(mean, cov, values)
            dim    = length(mean);
            d      = det(cov);
            invCov = cov \ eye(dim);
            c      = (1 / ((2 * pi)^(dim/2) * sqrt(abs(d))));
            v      = bsxfun(@minus, values, mean);
            
            logValues = log(c * exp(-0.5 * diag(v' * invCov * v)'));
        end
    end
end
