
classdef TestGaussianSamplingCKF < TestGaussianSampling
    % Provides unit tests for the GaussianSamplingCKF class.
    
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
        function testDefaultConfig(obj)
            g   = obj.initSampling();
            tol = 1e-12;
            
            obj.testGetStdNormalSamples(g,  1,   3, tol);
            obj.testGetStdNormalSamples(g,  5,  51, tol);
            obj.testGetStdNormalSamples(g, 10, 201, tol);
            
            obj.testGetSamples(g, obj.gaussian1D,  3, tol);
            obj.testGetSamples(g, obj.gaussian3D, 19, tol);
        end
    end
    
    methods (Access = 'protected')
        function g = initSampling(~)
            g = GaussianSamplingCKF();
        end
    end
end
