
classdef TestGLCD < matlab.unittest.TestCase
    % Provides unit tests for the LCD-based Gaussian sampling (GLCD).
    
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
        function testDefaultEven(obj)
            dim        = 3;
            numSamples = 20;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples);
            
            obj.checkSymEven(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testDefaultOdd(obj)
            dim        = 3;
            numSamples = 21;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples);
            
            obj.checkSymOdd(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        
        function testSymmetricEven(obj)
            dim        = 3;
            numSamples = 20;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true);
            
            obj.checkSymEven(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testSymmetricOdd(obj)
            dim        = 3;
            numSamples = 21;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true);
            
            obj.checkSymOdd(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testAsymmetricEven(obj)
            dim        = 3;
            numSamples = 20;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testAsymmetricOdd(obj)
            dim        = 3;
            numSamples = 21;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        
        function testBMaxSymmetricEven(obj)
            dim        = 3;
            numSamples = 20;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true, 10);
            
            obj.checkSymEven(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testBMaxSymmetricOdd(obj)
            dim        = 3;
            numSamples = 21;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true, 10);
            
            obj.checkSymOdd(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testBMaxAsymmetricEven(obj)
            dim        = 3;
            numSamples = 20;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false, 10);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        function testBMaxAsymmetricOdd(obj)
            dim        = 3;
            numSamples = 21;
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false, 10);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
        end
        
        
        function testInitSamplesBMaxSymmetricEven(obj)
            dim            = 3;
            numSamples      = 20;
            provInitSamples = randn(3, 10);
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true, 10, provInitSamples);
            
            obj.checkSymEven(dim, numSamples, samples, weight, initSamples, optDist, covError);
            
            obj.verifyEqual(provInitSamples, initSamples);
        end
        
        function testInitSamplesBMaxSymmetricOdd(obj)
            dim             = 3;
            numSamples      = 21;
            provInitSamples = randn(3, 10);
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, true, 10, provInitSamples);
            
            obj.checkSymOdd(dim, numSamples, samples, weight, initSamples, optDist, covError);
            
            obj.verifyEqual(provInitSamples, initSamples);
        end
        
        function testInitSamplesBMaxAsymmetricEven(obj)
            dim             = 3;
            numSamples      = 20;
            provInitSamples = randn(3, 20);
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false, 10, provInitSamples);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
            
            obj.verifyEqual(provInitSamples, initSamples);
        end
        
        function testInitSamplesBMaxAsymmetricOdd(obj)
            dim             = 3;
            numSamples      = 21;
            provInitSamples = randn(3, 21);
            
            [samples, weight, initSamples, optDist, covError] = GLCD(dim, numSamples, false, 10, provInitSamples);
            
            obj.checkAsym(dim, numSamples, samples, weight, initSamples, optDist, covError);
            
            obj.verifyEqual(provInitSamples, initSamples);
        end
        
        
        function testInvalidDim(obj)
            obj.verifyError(@() GLCD(0, 10), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(-1, 10), 'GLCD:ComputationFailed');
        end
        
        function testInvalidNumSamples(obj)
            obj.verifyError(@() GLCD(5, 0), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, -1), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 8), 'GLCD:ComputationFailed');
        end
        
        function testInvalidSymmetricFlag(obj)
            obj.verifyError(@() GLCD(5, 50, eye(2)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 50, -1), 'GLCD:ComputationFailed');
        end
        
        function testInvalidBMax(obj)
            obj.verifyError(@() GLCD(5, 50, true, 0), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 50, true, eye(2)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 50, false, 0), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 50, false, eye(2)), 'GLCD:ComputationFailed');
        end
        
        function testInvalidInitSamples(obj)
            obj.verifyError(@() GLCD(5, 50, true, 10, randn(2, 25)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 50, true, 10, randn(5, 20)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 51, true, 10, randn(2, 25)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 51, true, 10, randn(5, 20)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 51, false, 10, randn(2, 51)), 'GLCD:ComputationFailed');
            obj.verifyError(@() GLCD(5, 51, false, 10, randn(5, 30)), 'GLCD:ComputationFailed');
        end
    end
    
    methods (Access = 'private')
        function checkSymEven(obj, dim, numSamples, samples, weight, initSamples, optDist, covError)
            obj.verifySize(samples, [dim numSamples]);
            obj.verifyEqual(samples(:, 1:2:end), -samples(:, 2:2:end));
            obj.verifyEqual(weight, 1 / numSamples);
            obj.verifySize(initSamples, [dim numSamples / 2]);
            obj.verifyGreaterThan(optDist, 0);
            obj.verifyGreaterThanOrEqual(covError, 0);
        end
        
        function checkSymOdd(obj, dim, numSamples, samples, weight, initSamples, optDist, covError)
            obj.verifySize(samples, [dim numSamples]);
            obj.verifyEqual(samples(:, 1:2:numSamples-1), -samples(:, 2:2:numSamples-1));
            obj.verifyEqual(samples(:, numSamples), zeros(dim, 1));
            obj.verifyEqual(weight, 1 / numSamples);
            obj.verifySize(initSamples, [dim floor(numSamples / 2)]);
            obj.verifyGreaterThan(optDist, 0);
            obj.verifyGreaterThanOrEqual(covError, 0);
        end
        
        function checkAsym(obj, dim, numSamples, samples, weight, initSamples, optDist, covError)
            obj.verifySize(samples, [dim numSamples]);
            obj.verifyEqual(weight, 1 / numSamples);
            obj.verifySize(initSamples, [dim numSamples]);
            obj.verifyGreaterThan(optDist, 0);
            obj.verifyGreaterThanOrEqual(covError, 0);
        end
    end
end
