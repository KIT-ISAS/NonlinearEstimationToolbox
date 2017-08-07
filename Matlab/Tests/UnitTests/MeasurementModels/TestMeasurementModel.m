
classdef TestMeasurementModel < matlab.unittest.TestCase
    % Provides unit tests for the MeasurementModel class.
    
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
        function testSimulate(obj)
            measModel = MeasModel();
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            state = [0.3 -pi]';
            
            detMeas = measModel.measMatrix * state;
            
            measurement = measModel.simulate(state);
            
            obj.verifyEqual(size(measurement), [3 1]);
            obj.verifyGreaterThanOrEqual(measurement, detMeas);
            obj.verifyLessThanOrEqual(measurement, detMeas + 1);
        end
        
        
        function testDerivative(obj)
            measModel = MeasModel();
            
            nominalState = [-3 0.5]';
            nominalNoise = [1 -0.7 2.3]';
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = measModel.derivative(nominalState, nominalNoise);
            
            obj.verifyEqual(stateJacobian, measModel.measMatrix, 'AbsTol', 1e-8);
            obj.verifyEqual(noiseJacobian, eye(3), 'AbsTol', 1e-8);
            
            obj.verifyEqual(stateHessians, zeros(2, 2, 3), 'AbsTol', 1e-8);
            obj.verifyEqual(noiseHessians, zeros(3, 3, 3), 'AbsTol', 1e-8);
        end
    end
end
