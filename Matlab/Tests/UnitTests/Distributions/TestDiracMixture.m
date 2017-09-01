
classdef TestDiracMixture < matlab.unittest.TestCase
    % Provides unit tests for the DiracMixture class.
    
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
            dm = DiracMixture();
            
            obj.verifyResetDM(dm);
        end
        
        function testConstructorSamples(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            dm = DiracMixture(samples);
            
            dim      = 3;
            numComps = 7;
            weights  = ones(1, 7) / 7;
            mean     = [-4 -4 -4]';
            cov      = diag(repmat(8 / 7, 1, 3));
            covSqrt  = chol(cov)';
            
            obj.verifyDM(dm, dim, numComps, samples, weights, mean, cov, covSqrt);
        end
        
        function testConstructorSamplesWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1];
            
            dm = DiracMixture(samples, weights);
            
            dim      = 3;
            numComps = 7;
            weights  = weights / 8;
            mean     = [-4 -4 -4]';
            cov      = eye(3);
            covSqrt  = chol(cov)';
            
            obj.verifyDM(dm, dim, numComps, samples, weights, mean, cov, covSqrt);
        end
        
        function testConstructorInvalidSamples(obj)
            obj.verifyError(@() DiracMixture(ones(2, 3, 3)), ...
                            'DiracMixture:InvalidSamples');
            
            obj.verifyError(@() DiracMixture('test'), ...
                            'DiracMixture:InvalidSamples');
        end
        
        function testConstructorInvalidWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            obj.verifyError(@() DiracMixture(samples, 2), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, ones(7, 1)), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, eye(2)), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, ones(1, 7, 2)), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, zeros(1, 7)), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, [1 2 3 4 5 6 -7]), ...
                            'DiracMixture:InvalidWeights');
            
            obj.verifyError(@() DiracMixture(samples, 'test'), ...
                            'DiracMixture:InvalidWeights');
        end
        
        
        function testSetSamples(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            dm = DiracMixture();
            
            dm.set(samples);
            
            dim      = 3;
            numComps = 7;
            weights  = ones(1, 7) / 7;
            mean     = [-4 -4 -4]';
            cov      = diag(repmat(8 / 7, 1, 3));
            covSqrt  = chol(cov)';
            
            obj.verifyDM(dm, dim, numComps, samples, weights, mean, cov, covSqrt);
        end
        
        function testSetSamplesWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1];
            
            dm = DiracMixture();
            
            dm.set(samples, weights);
            
            dim      = 3;
            numComps = 7;
            weights  = weights / 8;
            mean     = [-4 -4 -4]';
            cov      = eye(3);
            covSqrt  = chol(cov)';
            
            obj.verifyDM(dm, dim, numComps, samples, weights, mean, cov, covSqrt);
        end
        
        function testSetInvalidSamples(obj)
            dm = DiracMixture();
            obj.verifyError(@() dm.set(ones(2, 3, 3)), ...
                            'DiracMixture:InvalidSamples');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set('test'), ...
                            'DiracMixture:InvalidSamples');
            obj.verifyResetDM(dm);
        end
        
        function testSetInvalidWeights(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, 2), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, ones(7, 1)), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, eye(2)), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, ones(1, 7, 2)), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, zeros(1, 7)), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, [1 2 3 4 5 6 -7]), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
            
            dm = DiracMixture();
            obj.verifyError(@() dm.set(samples, 'test'), ...
                            'DiracMixture:InvalidWeights');
            obj.verifyResetDM(dm);
        end
        
        
        function testGetMeanAndCovInvalidCov(obj)
            samples = [-1 1
                        0 0];
            
            dm = DiracMixture(samples);
            
            obj.verifyError(@() obj.testInvalidCov(dm), ...
                            'DiracMixture:InvalidCovariance');
        end
        
        
        function testDrawRndSamples(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1] / 8;
            
            dm = DiracMixture(samples, weights);
            
            numSamples = 1;
            
            samples = dm.drawRndSamples(numSamples);
            
            obj.verifySize(samples, [3, numSamples]);
            
            numSamples = 19;
            
            samples = dm.drawRndSamples(numSamples);
            
            obj.verifySize(samples, [3, numSamples]);
        end
        
        function testDrawRndSamplesInvalidNumSamples(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1] / 8;
            
            dm = DiracMixture(samples, weights);
            
            obj.verifyError(@() dm.drawRndSamples(-3.4), ...
                            'DiracMixture:InvalidNumberOfSamples');
            
            obj.verifyError(@() dm.drawRndSamples(eye(2)), ...
                            'DiracMixture:InvalidNumberOfSamples');
            
            obj.verifyError(@() dm.drawRndSamples('test'), ...
                            'DiracMixture:InvalidNumberOfSamples');
        end
        
        
        function testLogPdf(obj)
            samples = [zeros(3, 1) 2 * eye(3) -2 * eye(3)];
            samples = bsxfun(@plus, samples, -4 * ones(3, 1));
            weights = [2 1 1 1 1 1 1] / 8;
            
            dm = DiracMixture(samples, weights);
            
            obj.verifyError(@() dm.logPdf(ones(3, 1)), ...
                            'DiracMixture:LogPdfNotSupported');
        end
    end
    
    methods (Access = 'private')
        function verifyDM(obj, dm, dim, numComps, samples, weights, mean, cov, covSqrt)
            absTol = 1e-12;
            
            d = dm.getDim();
            obj.verifyEqual(d, dim);
            
            n = dm.getNumComponents();
            obj.verifyEqual(n, numComps);
            
            [s, w] = dm.getComponents();
            obj.verifyEqual(s, samples);
            obj.verifyEqual(w, weights);
            
            [m, c, cSqrt] = dm.getMeanAndCov();
            obj.verifyEqual(m, mean, 'AbsTol', absTol);
            obj.verifyEqual(c, c');
            obj.verifyEqual(c, cov, 'AbsTol', absTol);
            obj.verifyEqual(cSqrt, covSqrt, 'AbsTol', absTol);
        end
        
        function verifyResetDM(obj, dm)
            obj.verifyDM(dm, 0, 0, [], [], [], [], []);
        end
        
        function testInvalidCov(~, dm)
            [~, ~, ~] = dm.getMeanAndCov();
        end
    end
end
