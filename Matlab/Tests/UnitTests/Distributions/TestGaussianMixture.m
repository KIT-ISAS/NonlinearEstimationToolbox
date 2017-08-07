
classdef TestGaussianMixture < matlab.unittest.TestCase
    % Provides unit tests for the GaussianMixture class.
    
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
            gm = GaussianMixture();
            
            obj.verifyResetGM(gm);
        end
        
        function testConstructorMeanCovariance(obj)
            means = [1 -2]';
            covs  = [2 0.5; 0.5 1.2];
            
            gm = GaussianMixture(means, covs);
            
            dim      = 2;
            numComps = 1;
            weights  = 1;
            mean     = [1 -2]';
            cov      = [2 0.5; 0.5 1.2];
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testConstructorMeansCovariances(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.5;
            w2 = 0.5;
            
            means = [m1 m2];
            covs  = cat(3, c1, c2);
            
            gm = GaussianMixture(means, covs);
            
            dim      = 2;
            numComps = 2;
            weights  = [w1 w2];
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testConstructorMeansCovariancesWeights(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.31;
            w2 = 0.967;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            gm = GaussianMixture(means, covs, weights);
            
            dim      = 2;
            numComps = 2;
            weights  = weights / (w1 + w2);
            w1       = weights(1);
            w2       = weights(2);
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testConstructorInvalidMean(obj)
            covs = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            obj.verifyError(@() GaussianMixture(ones(2, 3, 3), covs), ...
                            'GaussianMixture:InvalidMeans');
            
            obj.verifyError(@() GaussianMixture('test', covs), ...
                            'GaussianMixture:InvalidMeans');
        end
        
        function testConstructorInvalidCovariances(obj)
            mean = [1 -2]';
            
            obj.verifyError(@() GaussianMixture(mean, 1), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(mean, ones(2, 2)), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(mean, eye(3)), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(mean, 'test'), ...
                            'GaussianMixture:InvalidCovariances');
            
            means = [1 -2; 0.4 3.4];
            
            obj.verifyError(@() GaussianMixture(means, eye(2)), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(means, ones(3, 2, 2)), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(means, cat(3, diag([-0.1 3]), [2 0.5; 0.5 1.2])), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(means, cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2], eye(2))), ...
                            'GaussianMixture:InvalidCovariances');
            
            obj.verifyError(@() GaussianMixture(means, 'test'), ...
                            'GaussianMixture:InvalidCovariances');
        end
        
        function testConstructorInvalidWeights(obj)
            means = [1 -2; 0.4 3.4];
            covs  = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            obj.verifyError(@() GaussianMixture(means, covs, 2), ...
                            'GaussianMixture:InvalidWeights');
            
            obj.verifyError(@() GaussianMixture(means, covs, eye(2)), ...
                            'GaussianMixture:InvalidWeights');
                        
            obj.verifyError(@() GaussianMixture(means, covs, ones(2, 1)), ...
                            'GaussianMixture:InvalidWeights');
            
            obj.verifyError(@() GaussianMixture(means, covs, ones(1, 2, 2)), ...
                            'GaussianMixture:InvalidWeights');
            
            obj.verifyError(@() GaussianMixture(means, covs, [0 0]), ...
                            'GaussianMixture:InvalidWeights');
            
            obj.verifyError(@() GaussianMixture(means, covs, [-1 2]), ...
                            'GaussianMixture:InvalidWeights');
            
            obj.verifyError(@() GaussianMixture(means, covs, 'test'), ...
                            'GaussianMixture:InvalidWeights');
        end
        
        
        function testSetMeanCovariance(obj)
            means = [1 -2]';
            covs  = [2 0.5; 0.5 1.2];
            
            gm = GaussianMixture();
            
            gm.set(means, covs);
            
            dim      = 2;
            numComps = 1;
            weights  = 1;
            mean     = [1 -2]';
            cov      = [2 0.5; 0.5 1.2];
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testSetMeansCovariances(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.5;
            w2 = 0.5;
            
            means = [m1 m2];
            covs  = cat(3, c1, c2);
            
            gm = GaussianMixture();
            
            gm.set(means, covs);
            
            dim      = 2;
            numComps = 2;
            weights  = [w1 w2];
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testSetMeansCovariancesWeights(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.31;
            w2 = 0.967;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            gm = GaussianMixture();
            
            gm.set(means, covs, weights);
            
            dim      = 2;
            numComps = 2;
            weights  = weights / (w1 + w2);
            w1       = weights(1);
            w2       = weights(2);
            mean     = w1 * m1 + w2 * m2;
            cov      = w1 * c1 + w2 * c2 + ...
                       w1 * (m1 - mean) * (m1 - mean)' + ...
                       w2 * (m2 - mean) * (m2 - mean)';
            covSqrt  = chol(cov)';
            
            obj.verifyGM(gm, dim, numComps, means, covs, weights, mean, cov, covSqrt);
        end
        
        function testSetInvalidMean(obj)
            covs = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(ones(2, 3, 3), covs), ...
                            'GaussianMixture:InvalidMeans');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() GaussianMixture('test', covs), ...
                            'GaussianMixture:InvalidMeans');
            obj.verifyResetGM(gm);
        end
        
        function testSetInvalidCovariances(obj)
            mean = [1 -2]';
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(mean, 1), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(mean, ones(2, 2)), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(mean, eye(3)), ...
                            'GaussianMixture:InvalidCovariances');
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(mean, 'test'), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            means = [1 -2; 0.4 3.4];
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, eye(2)), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, ones(3, 2, 2)), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, cat(3, diag([-0.1 3]), [2 0.5; 0.5 1.2])), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2], eye(2))), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, 'test'), ...
                            'GaussianMixture:InvalidCovariances');
            obj.verifyResetGM(gm);
        end
        
        function testSetInvalidWeights(obj)
            means = [1 -2; 0.4 3.4];
            covs  = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, 2), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, eye(2)), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, ones(2, 1)), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, ones(1, 2, 2)), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, [0 0]), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, [-1 2]), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
            
            gm = GaussianMixture();
            obj.verifyError(@() gm.set(means, covs, 'test'), ...
                            'GaussianMixture:InvalidWeights');
            obj.verifyResetGM(gm);
        end
        
        
        function testDrawRndSamples(obj)
            means   = [1 -2; 0.4 3.4];
            covs    = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            weights = [0.79 0.21];
            
            gm = GaussianMixture(means, covs, weights);
            
            numSamples = 1;
            
            [samples, compIds] = gm.drawRndSamples(numSamples);
            
            obj.verifySize(samples, [2, numSamples]);
            obj.verifySize(compIds, [1, numSamples]);
            obj.verifyGreaterThanOrEqual(compIds, 1);
            obj.verifyLessThanOrEqual(compIds, 2);
            
            numSamples = 19;
            
            [samples, compIds] = gm.drawRndSamples(numSamples);
            
            obj.verifySize(samples, [2, numSamples]);
            obj.verifySize(compIds, [1, numSamples]);
            obj.verifyGreaterThanOrEqual(compIds, 1);
            obj.verifyLessThanOrEqual(compIds, 2);
            
            % Check for monotonicity of component IDs
            lastId = compIds(1);
            
            for i = 2:numSamples
                obj.verifyGreaterThanOrEqual(compIds(i), lastId);
                
                lastId = compIds(i);
            end
        end
        
        function testDrawRndSamplesInvalidNumSamples(obj)
            means = [1 -2; 0.4 3.4];
            covs  = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            gm = GaussianMixture(means, covs);
            
            obj.verifyError(@() gm.drawRndSamples(-3.4), ...
                            'GaussianMixture:InvalidNumberOfSamples');
            
            obj.verifyError(@() gm.drawRndSamples(eye(2)), ...
                            'GaussianMixture:InvalidNumberOfSamples');
            
            obj.verifyError(@() gm.drawRndSamples('test'), ...
                            'GaussianMixture:InvalidNumberOfSamples');
        end
        
        
        function testLogPdfOneComponent(obj)
            mean   = [1 0.4]';
            cov    = [2 0.5; 0.5 1.2];
            weight = 1;
            
            gm = GaussianMixture(mean, cov, weight);
            
            values = [0.1 -0.5 13.4
                      0.9 -10   5  ];
            
            logValues = gm.logPdf(values);
            
            obj.verifySize(logValues, [1, 3]);
            
            v = obj.gaussian2d(mean, cov, values);
            
            trueLogValues = log(v);
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfTwoComponents(obj)
            m1 = [1 0.4]';
            m2 = [-2 3.4]';
            c1 = diag([0.1 3]);
            c2 = [2 0.5; 0.5 1.2];
            w1 = 0.3;
            w2 = 1.2;
            
            means   = [m1 m2];
            covs    = cat(3, c1, c2);
            weights = [w1 w2];
            
            gm = GaussianMixture(means, covs, weights);
            
            values = [0.1 -0.5 13.4
                      0.9 -10   5  ];
            
            logValues = gm.logPdf(values);
            
            obj.verifySize(logValues, [1, 3]);
            
            s  = w1 + w2;
            w1 = w1 / s;
            w2 = w2 / s;
            
            v1 = obj.gaussian2d(m1, c1, values);
            v2 = obj.gaussian2d(m2, c2, values);
            
            trueLogValues = log(w1 * v1 + w2 * v2);
            
            obj.verifyEqual(logValues, trueLogValues, 'AbsTol', 1e-10);
        end
        
        function testLogPdfInvalidValues(obj)
            means = [1 -2; 0.4 3.4];
            covs  = cat(3, diag([0.1 3]), [2 0.5; 0.5 1.2]);
            
            gm = GaussianMixture(means, covs);
            
            obj.verifyError(@() gm.logPdf([0.1 -0.5 13.4]), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() gm.logPdf(ones(3, 10)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() gm.logPdf(ones(2, 10, 5)), ...
                            'Distribution:InvalidValues');
            
            obj.verifyError(@() gm.logPdf('test'), ...
                            'Distribution:InvalidValues');
        end
    end
    
    methods (Static, Access = 'private')
        function values = gaussian2d(mean, cov, values)
            det    = cov(1, 1) * cov(2, 2) - cov(2, 1) * cov(1, 2);
            invCov = [cov(2, 2) -cov(1, 2); -cov(2, 1) cov(1, 1)] / det;
            c      = (1 / (2 * pi * sqrt(abs(det))));
            v      = bsxfun(@minus, values, mean);
            
            values = c * exp(-0.5 * diag(v' * invCov * v)');
        end
    end
    
    methods (Access = 'private')
        function verifyGM(obj, gm, dim, numComps, means, covs, weights, mean, cov, covSqrt)
            absTol = 1e-12;
            
            d = gm.getDim();
            obj.verifyEqual(d, dim);
            
            n = gm.getNumComponents();
            obj.verifyEqual(n, numComps);
            
            [m, c, w] = gm.getComponents();
            obj.verifyEqual(m, means);
            obj.verifyEqual(c, covs);
            obj.verifyEqual(w, weights);
            
            [m, c, cSqrt] = gm.getMeanAndCov();
            obj.verifyEqual(m, mean, 'AbsTol', absTol);
            obj.verifyEqual(c, c');
            obj.verifyEqual(c, cov, 'AbsTol', absTol);
            obj.verifyEqual(cSqrt, covSqrt, 'AbsTol', absTol);
        end
        
        function verifyResetGM(obj, gm)
            obj.verifyGM(gm, 0, 0, [], [], [], [], [], []);
        end
    end
end
