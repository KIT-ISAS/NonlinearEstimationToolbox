
classdef TestLinearSystemModel < matlab.unittest.TestCase
    % Provides unit tests for the LinearSystemModel class.
    
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
            sysModel = LinearSystemModel();
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testConstructorSysMatrix(obj)
            sysModel = LinearSystemModel(eye(2));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(2));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testConstructorSysMatrixSysNoiseMatrix(obj)
            sysModel = LinearSystemModel(eye(2), 2 * ones(2, 3));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(2));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, 2 * ones(2, 3));
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemMatrix(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemMatrix(eye(3));
            
            obj.verifyEqual(sysModel.sysMatrix, eye(3));
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemMatrixEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemMatrix([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemInput(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemInput([1 2 -3]');
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, [1 2 -3]');
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemInputEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemInput([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSetSystemNoiseMatrix(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemNoiseMatrix(ones(3, 2));
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, ones(3, 2));
            obj.verifyEqual(sysModel.noise, []);
        end
        
        function testSetSystemNoiseMatrixEmpty(obj)
            sysModel = LinearSystemModel();
            
            sysModel.setSystemNoiseMatrix([]);
            
            obj.verifyEqual(sysModel.sysMatrix, []);
            obj.verifyEqual(sysModel.sysInput, []);
            obj.verifyEqual(sysModel.sysNoiseMatrix, []);
            obj.verifyEqual(sysModel.noise, []);
        end
        
        
        function testSimulate(obj)
            sysMatrix = [3 -4; 0 2];
            state     = [0.3 -pi]';
            
            sysModel = LinearSystemModel(sysMatrix);
            sysModel.setNoise(Uniform([0 0], [1 1]));
            
            detSimState = sysMatrix * state;
            
            simState = sysModel.simulate(state);
            
            obj.verifyEqual(size(simState), [2 1]);
            obj.verifyGreaterThanOrEqual(simState, detSimState);
            obj.verifyLessThanOrEqual(simState, detSimState + 1);
        end
        
        
        function testDerivative(obj)
            sysMatrix = [1 1 2
                         0 1 0
                         2 1 2];
            
            sysNoiseMatrix = [1 0
                              0 1
                              0 1];
            
            sysModel = LinearSystemModel(sysMatrix, sysNoiseMatrix);
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = sysModel.derivative([3 -2 1]', [0 0]');
            
            obj.verifyEqual(stateJacobian, sysMatrix);
            obj.verifyEqual(noiseJacobian, sysNoiseMatrix);
            obj.verifyEqual(stateHessians, zeros(3, 3, 3));
            obj.verifyEqual(noiseHessians, zeros(2, 2, 3));
            
            obj.verifyError(@() sysModel.derivative([3 -2]', [0 0]'), ...
                            'LinearSystemModel:IncompatibleSystemMatrix');
            obj.verifyError(@() sysModel.derivative([3 -2 1]', 0), ...
                            'LinearSystemModel:IncompatibleSystemNoiseMatrix');
            
            sysModel = LinearSystemModel([], sysNoiseMatrix);
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = sysModel.derivative([3 -2 1]', [0 0]');
            
            obj.verifyEqual(stateJacobian, eye(3));
            obj.verifyEqual(noiseJacobian, sysNoiseMatrix);
            obj.verifyEqual(stateHessians, zeros(3, 3, 3));
            obj.verifyEqual(noiseHessians, zeros(2, 2, 3));
            
            obj.verifyError(@() sysModel.derivative([3 -2]', [0 0]'), ...
                            'LinearSystemModel:IncompatibleSystemNoiseMatrix');
            
            sysModel = LinearSystemModel();
            
            [stateJacobian, ...
             noiseJacobian, ...
             stateHessians, ...
             noiseHessians] = sysModel.derivative([3 -2 1]', [0 0 0]');
            
            obj.verifyEqual(stateJacobian, eye(3));
            obj.verifyEqual(noiseJacobian, eye(3));
            obj.verifyEqual(stateHessians, zeros(3, 3, 3));
            obj.verifyEqual(noiseHessians, zeros(3, 3, 3));
            
            obj.verifyError(@() sysModel.derivative([3 -2 1]', 0), ...
                            'LinearSystemModel:IncompatibleSystemNoise');
        end
    end
end
