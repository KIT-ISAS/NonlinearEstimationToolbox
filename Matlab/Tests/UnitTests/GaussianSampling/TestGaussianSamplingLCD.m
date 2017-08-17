
classdef TestGaussianSamplingLCD < TestGaussianSampling
    % Provides unit tests for the GaussianSamplingLCD class.
    
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
        function testConstructor(obj)
            g = obj.initSampling();
            
            [numSamplesAbs, ...
             numSamplesFactor] = g.getNumSamplesConfig();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 10);
            
            obj.verifyEqual(g.getSymmetricMode(), true);
        end
        
        
        function testSetNumSamples(obj)
            g = obj.initSampling();
            
            g.setNumSamples(201);
            
            [numSamplesAbs, ...
             numSamplesFactor] = g.getNumSamplesConfig();
            
            obj.verifyEqual(numSamplesAbs, 201);
            obj.verifyEmpty(numSamplesFactor);
        end
        
        function testSetNumSamplesByFactor(obj)
            g = obj.initSampling();
            
            g.setNumSamplesByFactor(5);
            
            [numSamplesAbs, ...
             numSamplesFactor] = g.getNumSamplesConfig();
            
            obj.verifyEmpty(numSamplesAbs);
            obj.verifyEqual(numSamplesFactor, 5);
        end
        
        function testSetSymmetricMode(obj)
            g = obj.initSampling();
            
            g.setSymmetricMode(false);
            
            obj.verifyEqual(g.getSymmetricMode(), false);
        end
        
        
        function testDefaultConfig(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1,  11, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5,  51, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 101, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D, 11, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 31, tol);
        end
        
        function testNumSamplesAbsoluteSymmetric(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumSamples(201);
            g.setSymmetricMode(true);
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1, 201, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5, 201, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 201, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D, 201, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 201, tol);
        end
        
        function testNumSamplesAbsoluteAsymmetric(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumSamples(201);
            g.setSymmetricMode(false);
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1, 201, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5, 201, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 201, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D, 201, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 201, tol);
        end
        
        function testNumSamplesFactorSymmetric(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumSamplesByFactor(5);
            g.setSymmetricMode(true);
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1,  5, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5, 25, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 51, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D,  5, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 15, tol);
        end
        
        function testNumSamplesFactorAsymmetric(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumSamplesByFactor(5);
            g.setSymmetricMode(false);
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1,  5, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5, 25, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 51, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D,  5, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 15, tol);
        end
    end
    
    methods (Access = 'protected')
        function g = initSampling(~)
            g = GaussianSamplingLCD();
        end
    end
end
