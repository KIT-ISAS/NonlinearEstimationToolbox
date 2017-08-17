
classdef TestGaussianSamplingGHQ < TestGaussianSampling
    % Provides unit tests for the GaussianSamplingGHQ class.
    
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
            
            obj.verifyEqual(g.getNumQuadraturePoints(), 2);
        end
        
        
        function testSetNumQuadraturePoints(obj)
            g = obj.initSampling();
            
            g.setNumQuadraturePoints(4);
            
            obj.verifyEqual(g.getNumQuadraturePoints(), 4);
        end
        
        
        function testDefaultConfig(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            obj.testGetStdNormalSamples(g,  1,    2, tol);
            obj.testGetStdNormalSamples(g,  5,   32, tol);
            obj.testGetStdNormalSamples(g, 10, 1024, tol);
            
            obj.testGetSamples(g, obj.gaussian1D, 2, tol);
            obj.testGetSamples(g, obj.gaussian3D, 8, tol);
        end
        
        function testNumQuadraturePointsThree(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumQuadraturePoints(3);
            
            obj.testGetStdNormalSamples(g,  1,     3, tol);
            obj.testGetStdNormalSamples(g,  5,   243, tol);
            obj.testGetStdNormalSamples(g, 10, 59049, tol);
            
            obj.testGetSamples(g, obj.gaussian1D,  3, tol);
            obj.testGetSamples(g, obj.gaussian3D, 27, tol);
        end
        
        function testNumQuadraturePointsFour(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            g.setNumQuadraturePoints(4);
            
            obj.testGetStdNormalSamples(g,  1,       4, tol);
            obj.testGetStdNormalSamples(g,  5,    1024, tol);
            obj.testGetStdNormalSamples(g, 10, 1048576, tol);
            
            obj.testGetSamples(g, obj.gaussian1D,  4, tol);
            obj.testGetSamples(g, obj.gaussian3D, 64, tol);
        end
    end
    
    methods (Access = 'protected')
        function g = initSampling(~)
            g = GaussianSamplingGHQ();
        end
    end
end
