
classdef TestGaussianSamplingRUKF < TestGaussianSampling
    % Provides unit tests for the GaussianSamplingRUKF class.
    
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
            
            obj.verifyEqual(g.getNumIterations(), 5);
        end
        
        
        function testSetNumIterations(obj)
            g = obj.initSampling();
            
            g.setNumIterations(10);
            
            obj.verifyEqual(g.getNumIterations(), 10);
        end
        
        
        function testDefaultConfig(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            obj.testGetStdNormalSamples(g,  1,  11, tol);
            obj.testGetStdNormalSamples(g,  5,  51, tol);
            obj.testGetStdNormalSamples(g, 10, 101, tol);
            
            obj.testGetSamples(g, obj.gaussian1D, 11, tol);
            obj.testGetSamples(g, obj.gaussian3D, 31, tol);
        end
        
        function testNumIterationsTen(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumIterations(10);
            
            obj.testGetStdNormalSamples(g,  1,  21, tol);
            obj.testGetStdNormalSamples(g,  5, 101, tol);
            obj.testGetStdNormalSamples(g, 10, 201, tol);
            
            obj.testGetSamples(g, obj.gaussian1D, 21, tol);
            obj.testGetSamples(g, obj.gaussian3D, 61, tol);
        end
    end
    
    methods (Access = 'protected')
        function g = initSampling(~)
            g = GaussianSamplingRUKF();
        end
    end
end
