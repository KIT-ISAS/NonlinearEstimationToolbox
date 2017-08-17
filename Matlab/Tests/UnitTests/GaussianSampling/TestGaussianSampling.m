
classdef TestGaussianSampling < matlab.unittest.TestCase
    % Provides unit tests for the GaussianSampling class.
    
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
    
    methods (Access = 'protected')
        function testGetStdNormalSamples(obj, g, dim, trueNumSamples, tol)
            [samples, weights, numSamples] = g.getStdNormalSamples(dim);
            
            obj.verifyEqual(numSamples, trueNumSamples);
            obj.verifyEqual(size(samples), [dim numSamples]);
            obj.verifyEqual(size(weights), [1 numSamples]);
            obj.verifyEqual(sum(weights), 1, 'AbsTol', 1e-8);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, zeros(dim, 1), 'AbsTol', tol);
            obj.verifyEqual(cov, eye(dim), 'AbsTol', tol);
        end
        
        function testGetStdNormalSamplesEquallyWeighted(obj, g, dim, trueNumSamples, tol)
            [samples, weights, numSamples] = g.getStdNormalSamples(dim);
            
            obj.verifyEqual(numSamples, trueNumSamples);
            obj.verifyEqual(size(samples), [dim numSamples]);
            obj.verifyEqual(weights, 1 / numSamples);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, zeros(dim, 1), 'AbsTol', tol);
            obj.verifyEqual(cov, eye(dim), 'AbsTol', tol);
        end
        
        
        function testGetSamples(obj, g, gaussian, trueNumSamples, tol)
            [trueMean, trueCov] = gaussian.getMeanAndCov();
            dim = gaussian.getDim();
            
            [samples, weights, numSamples] = g.getSamples(gaussian);
            
            obj.verifyEqual(numSamples, trueNumSamples);
            obj.verifyEqual(size(samples), [dim numSamples]);
            obj.verifyEqual(size(weights), [1 numSamples]);
            obj.verifyEqual(sum(weights), 1, 'AbsTol', 1e-8);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', tol);
            obj.verifyEqual(cov, trueCov, 'AbsTol', tol);
        end
        
        function testGetSamplesEquallyWeighted(obj, g, gaussian, trueNumSamples, tol)
            [trueMean, trueCov] = gaussian.getMeanAndCov();
            dim = gaussian.getDim();
            
            [samples, weights, numSamples] = g.getSamples(gaussian);
            
            obj.verifyEqual(numSamples, trueNumSamples);
            obj.verifyEqual(size(samples), [dim numSamples]);
            obj.verifyEqual(weights, 1 / numSamples);
            
            [mean, cov] = Utils.getMeanAndCov(samples, weights);
            
            obj.verifyEqual(mean, trueMean, 'AbsTol', tol);
            obj.verifyEqual(cov, trueCov, 'AbsTol', tol);
        end
    end
    
    methods (Abstract, Access = 'protected')
        g = initSampling(obj);
    end
    
    properties (Constant, Access = 'protected')
        gaussian1D = Gaussian(-5, 3);
        gaussian3D = Gaussian([2 -1 0.5]', 10 * [ 2   -0.5 0
                                                 -0.5  1.3 0.5
                                                  0    0.5 sqrt(2)]);
    end
end
