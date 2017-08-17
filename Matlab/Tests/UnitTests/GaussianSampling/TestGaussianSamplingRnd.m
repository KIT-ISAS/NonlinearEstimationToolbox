
classdef TestGaussianSamplingRnd < TestGaussianSampling
    % Provides unit tests for the GaussianSamplingRnd class.
    
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
            
            obj.verifyEqual(g.getNumSamples(), 1000);
        end
        
        
        function testSetNumSamples(obj)
            g = obj.initSampling();
            
            g.setNumSamples(500);
            
            obj.verifyEqual(g.getNumSamples(), 500);
        end
        
        
        function testNumSamplesBillion(obj)
            g   = obj.initSampling();
            tol = 0.1;
            
            g.setNumSamples(1e7);
            
            obj.testGetStdNormalSamplesEquallyWeighted(g,  1, 1e7, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g,  5, 1e7, tol);
            obj.testGetStdNormalSamplesEquallyWeighted(g, 10, 1e7, tol);
            
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian1D, 1e7, tol);
            obj.testGetSamplesEquallyWeighted(g, obj.gaussian3D, 1e7, tol);
        end
    end
    
    methods (Access = 'protected')
        function g = initSampling(~)
            g = GaussianSamplingRnd();
        end
    end
end
