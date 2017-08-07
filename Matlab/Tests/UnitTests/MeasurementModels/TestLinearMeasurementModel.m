
classdef TestLinearMeasurementModel < matlab.unittest.TestCase
    % Provides unit tests for the LinearMeasurementModel class.
    
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
            measModel = LinearMeasurementModel();
            
            obj.verifyEqual(measModel.measMatrix, []);
            obj.verifyEqual(measModel.noise, []);
        end
        
        function testConstructorMeasMatrix(obj)
            measModel = LinearMeasurementModel(ones(2, 3));
            
            obj.verifyEqual(measModel.measMatrix, ones(2, 3));
            obj.verifyEqual(measModel.noise, []);
        end
        
        
        function testSetMeasurementMatrix(obj)
            measModel = LinearMeasurementModel();
            
            measModel.setMeasurementMatrix(eye(3));
            
            obj.verifyEqual(measModel.measMatrix, eye(3));
            obj.verifyEqual(measModel.noise, []);
        end
        
        function testSetSetMeasurementMatrixEmpty(obj)
            measModel = LinearMeasurementModel();
            
            measModel.setMeasurementMatrix([]);
            
            obj.verifyEqual(measModel.measMatrix, []);
            obj.verifyEqual(measModel.noise, []);
        end
        
        
        function testSimulate(obj)
            measMatrix = [3    -4
                          pi/4  0
                          0.5   2];
            state      = [0.3 -pi]';
            
            measModel = LinearMeasurementModel(measMatrix);
            measModel.setNoise(Uniform([0 0 0], [1 1 1]));
            
            detMeas = measMatrix * state;
            
            measurement = measModel.simulate(state);
            
            obj.verifyEqual(size(measurement), [3 1]);
            obj.verifyGreaterThanOrEqual(measurement, detMeas);
            obj.verifyLessThanOrEqual(measurement, detMeas + 1);
        end
        
        
        function testDerivative(obj)
            measMatrix = [1 1 0
                          0 1 2];
            
            measModel = LinearMeasurementModel(measMatrix);
            
            [stateJacobian, stateHessians] = measModel.derivative([3 -2 1]');
            
            obj.verifyEqual(stateJacobian, measMatrix);
            obj.verifyEqual(stateHessians, zeros(3, 3, 2));
            
            obj.verifyError(@() measModel.derivative([3 -2]'), ...
                            'LinearMeasurementModel:IncompatibleMeasurementMatrix');
            
            measModel = LinearMeasurementModel();
            
            [stateJacobian, stateHessians] = measModel.derivative([3 -2 1]');
            
            obj.verifyEqual(stateJacobian, eye(3));
            obj.verifyEqual(stateHessians, zeros(3, 3, 3));
        end
    end
end
