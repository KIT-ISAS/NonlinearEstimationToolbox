
classdef TestURUF < TestRecursiveUpdateFilter
    % Provides unit tests for the URUF class.
    
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
        function testSetSampleScalings(obj)
            f = obj.initFilter();
            
            f.setSampleScalings(1);
            
            [scalingPrediction, ...
             scalingUpdate] = f.getSampleScalings();
            
            obj.verifyEqual(scalingPrediction, 1);
            obj.verifyEqual(scalingUpdate, 1);
        end
        
        function testSetSampleScalingsBoth(obj)
            f = obj.initFilter();
            
            f.setSampleScalings(1, 0);
            
            [scalingPrediction, ...
             scalingUpdate] = f.getSampleScalings();
            
            obj.verifyEqual(scalingPrediction, 1);
            obj.verifyEqual(scalingUpdate, 0);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = URUF();
        end
        
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestRecursiveUpdateFilter(f);
            
            % UKF-related tests
            obj.verifyEqual(f.getName(), 'URUF');
            
            [scalingPrediction, ...
             scalingUpdate] = f.getSampleScalings();
            
            obj.verifyEqual(scalingPrediction, 0.5);
            obj.verifyEqual(scalingUpdate, 0.5);
        end
    end
end
